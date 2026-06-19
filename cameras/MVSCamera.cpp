#include "MVSCamera.hpp"
#include "utils/exposure_fusion.hpp"
#include <iostream>
#include <cstring>
#include <arpa/inet.h>

MVSCamera::MVSCamera(const CONFIG& config) : AICamera(config) {}

MVSCamera::~MVSCamera() {
    if (m_isOpened) close();
}


unsigned int ipToInt(const std::string& ip) {
    in_addr ip_addr;
    if (inet_pton(AF_INET, ip.c_str(), &ip_addr) != 1) {
        std::cerr << "Invalid IP address" << std::endl;
        return -1;
    }
    return ntohl(ip_addr.s_addr);
}

bool MVSCamera::open() {
    if (m_isOpened) return true;

    int nRet = MV_CC_Initialize();
    if (MV_OK != nRet) {
        std::cerr << "[MVSCamera] MV_CC_Initialize fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        return false;
    }

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet || stDeviceList.nDeviceNum == 0) {
        std::cerr << "[MVSCamera] No device found! nRet [0x" << std::hex << nRet << "]" << std::endl;
        MV_CC_Finalize();
        return false;
    }

    int nIndex = -1;
    if (!m_config.cam_sn.empty()) {
        bool found = false;
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
            MV_CC_DEVICE_INFO* pInfo = stDeviceList.pDeviceInfo[i];
            if (pInfo == nullptr) continue;
            std::string sn;
            if (pInfo->nTLayerType == MV_USB_DEVICE)
                sn = (char*)pInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
            else if (pInfo->nTLayerType == MV_GIGE_DEVICE)
                sn = (char*)pInfo->SpecialInfo.stGigEInfo.chSerialNumber;
            if (sn == m_config.cam_sn) { nIndex = i; found = true; break; }
        }
        if (!found) {
            std::cerr << "[MVSCamera] Camera with SN '" << m_config.cam_sn << "' not found, using first device" << std::endl;
        }
    }

    if (!m_config.cam_ip.empty()) {
        unsigned int cfg_ip = ipToInt(m_config.cam_ip);
        bool found = false;
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
            MV_CC_DEVICE_INFO* pInfo = stDeviceList.pDeviceInfo[i];
            if (pInfo == nullptr) continue;
            std::string sn;
            if (pInfo->nTLayerType == MV_GIGE_DEVICE && cfg_ip==pInfo->SpecialInfo.stGigEInfo.nCurrentIp)
            { nIndex = i; found = true; break; }
        }
        if (!found) {
            std::cerr << "[MVSCamera] Camera with SN '" << m_config.cam_sn << "' not found, using first device" << std::endl;
        }
    }

    if (nIndex<0)
    {
        return false;
    }


    nRet = MV_CC_CreateHandle(&m_handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet) {
        std::cerr << "[MVSCamera] MV_CC_CreateHandle fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        MV_CC_Finalize();
        return false;
    }

    nRet = MV_CC_OpenDevice(m_handle);
    if (MV_OK != nRet) {
        std::cerr << "[MVSCamera] MV_CC_OpenDevice fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        MV_CC_DestroyHandle(m_handle);
        m_handle = nullptr;
        MV_CC_Finalize();
        return false;
    }

    // 在取流前完成所有参数配置，取流后这些寄存器会变为只读
    MV_CC_SetEnumValue(m_handle, "TriggerMode", 0);
    MV_CC_SetEnumValue(m_handle, "ExposureAuto", 0);

    if (m_config.cam_width > 0 && m_config.cam_height > 0) {
        MV_CC_SetIntValueEx(m_handle, "Width", m_config.cam_width);
        MV_CC_SetIntValueEx(m_handle, "Height", m_config.cam_height);
    }

    MVCC_INTVALUE_EX stWidth;
    if (MV_OK == MV_CC_GetIntValueEx(m_handle, "Width", &stWidth))
        m_width = (int)stWidth.nCurValue;
    MVCC_INTVALUE_EX stHeight;
    if (MV_OK == MV_CC_GetIntValueEx(m_handle, "Height", &stHeight))
        m_height = (int)stHeight.nCurValue;

    if (m_width <= 0 || m_height <= 0) {
        m_width = m_config.cam_width > 0 ? m_config.cam_width : 1280;
        m_height = m_config.cam_height > 0 ? m_config.cam_height : 800;
    }

    nRet = MV_CC_StartGrabbing(m_handle);
    if (MV_OK != nRet) {
        std::cerr << "[MVSCamera] MV_CC_StartGrabbing fail! nRet [0x" << std::hex << nRet << "]" << std::endl;
        MV_CC_CloseDevice(m_handle);
        MV_CC_DestroyHandle(m_handle);
        m_handle = nullptr;
        MV_CC_Finalize();
        return false;
    }

    m_nDataSize = m_width * m_height * 3;
    m_pData = new unsigned char[m_nDataSize];

    m_isGrabbing = true;
    m_isOpened = true;
    std::cout << "[MVSCamera] opened, resolution: " << m_width << "x" << m_height << std::endl;


    // 打开相机后，立即设置心跳超时时间为更大的值（单位：毫秒）
    // 60000 = 60秒，足够你在断点处停留思考
    nRet = MV_CC_SetIntValue(m_handle, "GevHeartbeatTimeout", 60000);
    if (nRet != MV_OK)
    {
        printf("Warning: Set GevHeartbeatTimeout fail nRet [0x%x]!\n", nRet);
    }

    return true;
}

void MVSCamera::close() {
    if (!m_isOpened) return;

    if (m_handle) {
        MV_CC_StopGrabbing(m_handle);
        MV_CC_CloseDevice(m_handle);
        MV_CC_DestroyHandle(m_handle);
        m_handle = nullptr;
    }
    m_isGrabbing = false;
    MV_CC_Finalize();

    if (m_pData) {
        delete[] m_pData;
        m_pData = nullptr;
    }
    m_isOpened = false;
    std::cout << "[MVSCamera] closed" << std::endl;
}

cv::Mat MVSCamera::captureOneFrame(int timeoutMs) {
    if (!m_isOpened || !m_handle || !m_pData) return cv::Mat();

    MV_FRAME_OUT_INFO_EX stFrameInfo;
    memset(&stFrameInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    int nRet = MV_CC_GetImageForBGR(m_handle, m_pData, m_nDataSize, &stFrameInfo, timeoutMs);
    if (MV_OK != nRet) {
        if (MV_E_NODATA == nRet)
            MV_CC_ClearImageBuffer(m_handle);
        return cv::Mat();
    }

    int w = stFrameInfo.nExtendWidth;
    int h = stFrameInfo.nExtendHeight;
    if (w <= 0 || h <= 0) return cv::Mat();

    cv::Mat frame(h, w, CV_8UC3);
    memcpy(frame.data, m_pData, w * h * 3);
    return frame;
}

int MVSCamera::captureOneFrameWithDepth(cv::Mat& bgrImage, cv::Mat& depthImage, int timeoutMs) {
    bgrImage = captureOneFrame(timeoutMs);
    depthImage = cv::Mat();
    return bgrImage.empty() ? -1 : 0;
}

bool MVSCamera::setColorExposureTime(int exposureUs) {
    if (!m_isOpened || !m_handle) return false;

    bool wasGrabbing = m_isGrabbing;
    if (wasGrabbing) {
        MV_CC_StopGrabbing(m_handle);
        m_isGrabbing = false;
    }

    int nRet = MV_CC_SetFloatValue(m_handle, "ExposureTime", (float)exposureUs);

    if (wasGrabbing) {
        MV_CC_StartGrabbing(m_handle);
        m_isGrabbing = true;
    }

    if (MV_OK == nRet) {
        m_exposureCached = exposureUs;
        return true;
    }

    std::cerr << "[MVSCamera] setColorExposureTime(" << exposureUs << ") fail! nRet [0x"
              << std::hex << nRet << "]" << std::endl;
    return false;
}

int MVSCamera::getColorExposureTime() {
    if (!m_isOpened || !m_handle) return -1;
    MVCC_FLOATVALUE stValue;
    int nRet = MV_CC_GetFloatValue(m_handle, "ExposureTime", &stValue);
    if (MV_OK == nRet) return (int)stValue.fCurValue;
    return -1;
}

bool MVSCamera::setAutoExposure(bool enable) {
    if (!m_isOpened || !m_handle) return false;

    bool wasGrabbing = m_isGrabbing;
    if (wasGrabbing) {
        MV_CC_StopGrabbing(m_handle);
        m_isGrabbing = false;
    }

    unsigned int mode = enable ? 2 : 0;
    int nRet = MV_CC_SetEnumValue(m_handle, "ExposureAuto", mode);

    if (wasGrabbing) {
        MV_CC_StartGrabbing(m_handle);
        m_isGrabbing = true;
    }

    if (MV_OK == nRet) {
        m_autoExposureCached = enable;
        return true;
    }
    std::cerr << "[MVSCamera] setAutoExposure(" << enable << ") fail! nRet [0x"
              << std::hex << nRet << "]" << std::endl;
    return false;
}

bool MVSCamera::getAutoExposure() {
    if (!m_isOpened || !m_handle) return false;
    MVCC_ENUMVALUE stValue;
    int nRet = MV_CC_GetEnumValue(m_handle, "ExposureAuto", &stValue);
    if (MV_OK == nRet) return stValue.nCurValue != 0;
    return false;
}

bool MVSCamera::getExposureRange(int& minUs, int& maxUs) {
    if (!m_isOpened || !m_handle) return false;
    MVCC_FLOATVALUE stValue;
    
    MVCC_INTVALUE_EX stIntValue_lower = { 0 };
    int nRet = MV_CC_GetIntValueEx(m_handle, "AutoExposureTimeLowerLimit", &stIntValue_lower);
    if (MV_OK == nRet) {
        minUs = stIntValue_lower.nCurValue;
        maxUs = 1000000;//stIntValue_lower.nMax;
    }else{
        minUs =11;
        maxUs = 1000000;
    }

   
    
    return true;
    

}

bool MVSCamera::setColorGain(int gain) {
    if (!m_isOpened || !m_handle) return false;
    int nRet = MV_CC_SetFloatValue(m_handle, "Gain", (float)gain);
    return (MV_OK == nRet);
}

int MVSCamera::getColorGain() {
    if (!m_isOpened || !m_handle) return -1;
    MVCC_FLOATVALUE stValue;
    int nRet = MV_CC_GetFloatValue(m_handle, "Gain", &stValue);
    if (MV_OK == nRet) return (int)stValue.fCurValue;
    return -1;
}

bool MVSCamera::getGainRange(int& min, int& max) {
    if (!m_isOpened || !m_handle) return false;
    MVCC_FLOATVALUE stValue;
    int nRet = MV_CC_GetFloatValue(m_handle, "Gain", &stValue);
    if (MV_OK == nRet) {
        min = (int)stValue.fMin;
        max = (int)stValue.fMax;
        return true;
    }
    return false;
}

std::vector<cv::Mat> MVSCamera::captureMultiExposureImages(const std::vector<int>& exposureValuesUs) {
    return captureMultiExposureImagesImpl(exposureValuesUs);
}

cv::Mat MVSCamera::captureAndFuseMultiExposure(const std::vector<int>& exposureValuesUs,
                                                float contrastWeight,
                                                float saturationWeight,
                                                float wellExposedWeight) {
    std::vector<cv::Mat> images = captureMultiExposureImagesImpl(exposureValuesUs);
    if (images.empty()) return cv::Mat();

    ExposureFusion::Params params;
    params.contrastWeight = contrastWeight;
    params.saturationWeight = saturationWeight;
    params.wellExposedWeight = wellExposedWeight;
    return ExposureFusion::fuse(images, params);
}

cv::Mat MVSCamera::captureAndFuseMultiExposureFast(const std::vector<int>& exposureValuesUs) {
    std::vector<cv::Mat> images = captureMultiExposureImagesImpl(exposureValuesUs);
    if (images.empty()) return cv::Mat();
    return ExposureFusion::fuseFast(images);
}

std::vector<cv::Mat> MVSCamera::captureMultiExposureImagesImpl(const std::vector<int>& exposureValuesUs) {
    std::vector<cv::Mat> images;
    if (!m_isOpened) return images;

    bool wasGrabbing = m_isGrabbing;
    if (wasGrabbing) {
        MV_CC_StopGrabbing(m_handle);
        m_isGrabbing = false;
    }

    bool wasAuto = getAutoExposure();
    MV_CC_SetEnumValue(m_handle, "ExposureAuto", 0);
    int prevExposure = getColorExposureTime();

    for (int exposure : exposureValuesUs) {
        MV_CC_SetFloatValue(m_handle, "ExposureTime", (float)exposure);
        MV_CC_StartGrabbing(m_handle);
        cv::Mat frame = captureOneFrame(500);
        MV_CC_StopGrabbing(m_handle);
        if (!frame.empty()) images.push_back(frame);
    }

    MV_CC_SetFloatValue(m_handle, "ExposureTime", (float)prevExposure);
    if (wasAuto) MV_CC_SetEnumValue(m_handle, "ExposureAuto", 2);

    if (wasGrabbing) {
        MV_CC_StartGrabbing(m_handle);
        m_isGrabbing = true;
    }
    return images;
}

std::vector<std::vector<float>> MVSCamera::getCameraIntrinsics(const int imgWidth, const int imgHight) {

    return std::vector<std::vector<float>>();
}
