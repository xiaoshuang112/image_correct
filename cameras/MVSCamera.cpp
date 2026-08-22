#include "MVSCamera.hpp"
#include "mvs_error.hpp"
#include "utils/exposure_fusion.hpp"
#include "hand_eye_gui/DiagnosticLogger.hpp"

#include <iostream>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <limits>

namespace {

// GigE 心跳超时：调试断点期间不至于掉线，同时断线能在有限时间内被发现。
constexpr int kGevHeartbeatTimeoutMs = 10000;
// 曝光/增益切换后丢弃的帧数，避免把旧参数帧用于标定。
constexpr int kDiscardFramesAfterParamChange = 1;
// 连续取帧失败的日志限频：第 1 次必报，之后每 100 次报一次。
constexpr uint64_t kFailureLogInterval = 100;

}  // namespace

MVSCamera::MVSCamera(const CONFIG& config) : AICamera(config) {}

MVSCamera::~MVSCamera() {
    close();
}

void MVSCamera::setError(const char* stage, unsigned int sdkCode, const std::string& extra) {
    m_lastError.stage = stage ? stage : "unknown";
    m_lastError.sdk_code = sdkCode;
    m_lastError.message = mvsdiag::diagnostic(stage, sdkCode);
    if (!extra.empty()) {
        m_lastError.message += " ";
        m_lastError.message += extra;
    }
    APP_LOG_EVENT_ERROR("camera.mvs",
                        m_lastError.stage.c_str(),
                        "sdk_error",
                        {{"code", QString("0x%1").arg(sdkCode, 0, 16)},
                         {"name", QString::fromLatin1(mvsdiag::errorName(sdkCode))},
                         {"detail", QString::fromStdString(m_lastError.message)}});
    std::cerr << "[MVSCamera] " << m_lastError.message << std::endl;
}

void MVSCamera::clearError() {
    m_lastError = CameraErrorDetail{};
}

CameraErrorDetail MVSCamera::lastError() const {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    return m_lastError;
}

CameraDeviceAuditInfo MVSCamera::getDeviceAuditInfo() const {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    CameraDeviceAuditInfo info;
    info.serial = m_deviceSerial.empty() ? m_config.cam_sn : m_deviceSerial;
    info.model = m_deviceModel.empty() ? "MVS" : m_deviceModel;
    info.firmware = m_deviceFirmware;
    info.transport = mvsdiag::transportName(m_transport);
    if (m_transport == mvsdiag::MvsTransport::kGigE && m_deviceIp != 0)
        info.current_ip = mvsdiag::ipv4ToString(m_deviceIp);
    info.actual_width = m_width;
    info.actual_height = m_height;
    // 设备实际流格式（Mono8/Bayer8/...）；输出给上层的图像固定转为 BGR8
    info.pixel_format = m_devicePixelFormat.empty() ? "unknown" : m_devicePixelFormat;
    info.exposure_us = m_exposureUs;
    info.gain = m_gain;
    info.trigger_mode = m_triggerOn ? "on" : "off";
    info.sdk_version = m_sdkVersion;
    return info;
}

bool MVSCamera::open() {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (m_isOpened) return true;

    clearError();
    releaseResourcesLocked();

    // ---- 阶段 1: SDK 初始化 ----
    int nRet = MV_CC_Initialize();
    if (MV_OK != nRet) {
        setError("initialize", nRet);
        return false;
    }
    m_sdkInitialized = true;
    {
        const unsigned int ver = MV_CC_GetSDKVersion();
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
                      (ver >> 24) & 0xFF, (ver >> 16) & 0xFF,
                      (ver >> 8) & 0xFF, ver & 0xFF);
        m_sdkVersion = buf;
    }

    // ---- 阶段 2: 枚举设备 ----
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(stDeviceList));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        setError("enumerate", nRet);
        releaseResourcesLocked();
        return false;
    }

    std::vector<mvsdiag::MvsDeviceDescriptor> devices;
    devices.reserve(stDeviceList.nDeviceNum);
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; ++i) {
        const MV_CC_DEVICE_INFO* pInfo = stDeviceList.pDeviceInfo[i];
        if (!pInfo) continue;
        mvsdiag::MvsDeviceDescriptor d;
        if (pInfo->nTLayerType == MV_GIGE_DEVICE) {
            d.transport = mvsdiag::MvsTransport::kGigE;
            d.serial = reinterpret_cast<const char*>(pInfo->SpecialInfo.stGigEInfo.chSerialNumber);
            d.model = reinterpret_cast<const char*>(pInfo->SpecialInfo.stGigEInfo.chModelName);
            d.ip = pInfo->SpecialInfo.stGigEInfo.nCurrentIp;
        } else if (pInfo->nTLayerType == MV_USB_DEVICE) {
            d.transport = mvsdiag::MvsTransport::kUsb;
            d.serial = reinterpret_cast<const char*>(pInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
            d.model = reinterpret_cast<const char*>(pInfo->SpecialInfo.stUsb3VInfo.chModelName);
        } else {
            d.transport = mvsdiag::MvsTransport::kOther;
        }
        devices.push_back(std::move(d));
    }

    // ---- 阶段 3: 固定策略选机（serial 精确 -> IP 精确 -> 唯一设备 -> 拒绝） ----
    const mvsdiag::MvsSelectionOutcome selection =
        mvsdiag::selectMvsDevice(m_config.cam_sn, m_config.cam_ip, devices);
    if (selection.status != mvsdiag::MvsSelectionStatus::kSelected) {
        m_lastError.stage = "select";
        m_lastError.sdk_code = 0;
        m_lastError.message = selection.message;
        APP_LOG_EVENT_ERROR("camera.mvs",
                            "select",
                            "fail",
                            {{"configured_sn", QString::fromStdString(m_config.cam_sn)},
                             {"configured_ip", QString::fromStdString(m_config.cam_ip)},
                             {"enumerated", QString::number(devices.size())},
                             {"detail", QString::fromStdString(selection.message)}});
        std::cerr << "[MVSCamera] " << selection.message << std::endl;
        releaseResourcesLocked();
        return false;
    }

    const int nIndex = selection.index;
    MV_CC_DEVICE_INFO* selectedInfo = stDeviceList.pDeviceInfo[nIndex];
    const mvsdiag::MvsDeviceDescriptor& selectedDesc = devices[nIndex];
    m_transport = selectedDesc.transport;
    m_deviceSerial = selectedDesc.serial;
    m_deviceModel = selectedDesc.model;
    m_deviceIp = selectedDesc.ip;

    APP_LOG_EVENT_INFO("camera.mvs",
                       "select",
                       "success",
                       {{"index", QString::number(nIndex)},
                        {"serial", QString::fromStdString(m_deviceSerial)},
                        {"model", QString::fromStdString(m_deviceModel)},
                        {"transport", QString::fromLatin1(mvsdiag::transportName(m_transport))},
                        {"ip", QString::fromStdString(
                             m_transport == mvsdiag::MvsTransport::kGigE
                                 ? mvsdiag::ipv4ToString(m_deviceIp) : std::string())},
                        {"enumerated", QString::number(devices.size())}});

    // ---- 阶段 4: 创建句柄 ----
    nRet = MV_CC_CreateHandle(&m_handle, selectedInfo);
    if (MV_OK != nRet) {
        setError("create_handle", nRet);
        releaseResourcesLocked();
        return false;
    }
    m_handleCreated = true;

    // ---- 阶段 5: 打开设备（独占访问） ----
    nRet = MV_CC_OpenDevice(m_handle, MV_ACCESS_Exclusive, 0);
    if (MV_OK != nRet) {
        setError("open_device", nRet,
                 "serial=" + m_deviceSerial +
                 (m_deviceIp ? " ip=" + mvsdiag::ipv4ToString(m_deviceIp) : ""));
        releaseResourcesLocked();
        return false;
    }
    m_deviceOpened = true;

    // 固件版本（尽力而为，失败不阻断）
    MVCC_STRINGVALUE firmwareValue{};
    if (MV_OK == MV_CC_GetStringValue(m_handle, "DeviceFirmwareVersion", &firmwareValue))
        m_deviceFirmware = firmwareValue.chCurValue;

    // ---- 阶段 6: 参数配置（取流前完成，取流后部分节点只读） ----
    if (!configureDeviceLocked()) {
        releaseResourcesLocked();
        return false;
    }

    // ---- 阶段 7: 开始取流 ----
    nRet = MV_CC_StartGrabbing(m_handle);
    if (MV_OK != nRet) {
        setError("start_grabbing", nRet);
        releaseResourcesLocked();
        return false;
    }
    m_isGrabbing = true;

    // ---- 阶段 8: 取帧缓冲区按设备实际负载分配 ----
    MVCC_INTVALUE_EX stPayload{};
    unsigned long long payloadSize = 0;
    if (MV_OK == MV_CC_GetIntValueEx(m_handle, "PayloadSize", &stPayload) &&
        stPayload.nCurValue > 0) {
        payloadSize = stPayload.nCurValue;
    } else {
        payloadSize = static_cast<unsigned long long>(m_width) *
                      static_cast<unsigned long long>(m_height) * 3ULL;
    }
    if (!ensureBufferCapacityLocked(static_cast<size_t>(payloadSize))) {
        setError("allocate_buffer", MV_E_RESOURCE,
                 "payload=" + std::to_string(payloadSize));
        releaseResourcesLocked();
        return false;
    }

    readbackActualParamsLocked();

    m_frameOkCount = 0;
    m_frameFailCount = 0;
    m_consecutiveFailures = 0;
    m_isOpened = true;

    APP_LOG_EVENT_INFO("camera.mvs",
                       "open",
                       "success",
                       {{"serial", QString::fromStdString(m_deviceSerial)},
                        {"model", QString::fromStdString(m_deviceModel)},
                        {"firmware", QString::fromStdString(m_deviceFirmware)},
                        {"transport", QString::fromLatin1(mvsdiag::transportName(m_transport))},
                        {"ip", QString::fromStdString(
                             m_deviceIp ? mvsdiag::ipv4ToString(m_deviceIp) : std::string())},
                        {"actual_resolution", QString("%1x%2").arg(m_width).arg(m_height)},
                        {"requested_resolution", QString("%1x%2").arg(m_config.cam_width).arg(m_config.cam_height)},
                        {"device_pixel_format", QString::fromStdString(m_devicePixelFormat)},
                        {"payload_bytes", QString::number(payloadSize)},
                        {"sdk_version", QString::fromStdString(m_sdkVersion)}});
    std::cout << "[MVSCamera] opened: serial=" << m_deviceSerial
              << " model=" << m_deviceModel
              << " actual=" << m_width << "x" << m_height << std::endl;
    return true;
}

bool MVSCamera::configureDeviceLocked() {
    // 触发模式关闭（连续采集）
    int nRet = MV_CC_SetEnumValue(m_handle, "TriggerMode", 0);
    if (MV_OK != nRet) {
        setError("configure", nRet, "node=TriggerMode value=off");
        return false;
    }
    // 默认关闭自动曝光，曝光由产线显式设置
    nRet = MV_CC_SetEnumValue(m_handle, "ExposureAuto", 0);
    if (MV_OK != nRet) {
        setError("configure", nRet, "node=ExposureAuto value=off");
        return false;
    }

    // 分辨率：设置后必须回读实际值。设备按步进取整时不算失败，
    // 但必须记录实际值，绝不能继续假装使用请求分辨率。
    if (m_config.cam_width > 0 && m_config.cam_height > 0) {
        nRet = MV_CC_SetIntValueEx(m_handle, "Width", m_config.cam_width);
        if (MV_OK != nRet) {
            setError("configure", nRet,
                     "node=Width value=" + std::to_string(m_config.cam_width));
            return false;
        }
        nRet = MV_CC_SetIntValueEx(m_handle, "Height", m_config.cam_height);
        if (MV_OK != nRet) {
            setError("configure", nRet,
                     "node=Height value=" + std::to_string(m_config.cam_height));
            return false;
        }
    }

    MVCC_INTVALUE_EX stWidth{};
    MVCC_INTVALUE_EX stHeight{};
    nRet = MV_CC_GetIntValueEx(m_handle, "Width", &stWidth);
    if (MV_OK != nRet) {
        setError("configure", nRet, "node=Width readback");
        return false;
    }
    nRet = MV_CC_GetIntValueEx(m_handle, "Height", &stHeight);
    if (MV_OK != nRet) {
        setError("configure", nRet, "node=Height readback");
        return false;
    }
    m_width = static_cast<int>(stWidth.nCurValue);
    m_height = static_cast<int>(stHeight.nCurValue);
    if (m_width <= 0 || m_height <= 0) {
        setError("configure", MV_E_PARAMETER,
                 "readback resolution invalid: " + std::to_string(m_width) +
                 "x" + std::to_string(m_height));
        return false;
    }
    if (m_config.cam_width > 0 && m_config.cam_height > 0 &&
        (m_width != m_config.cam_width || m_height != m_config.cam_height)) {
        APP_LOG_EVENT_WARNING("camera.mvs",
                              "configure",
                              "resolution_fallback",
                              {{"requested", QString("%1x%2").arg(m_config.cam_width).arg(m_config.cam_height)},
                               {"actual", QString("%1x%2").arg(m_width).arg(m_height)}});
        std::cerr << "[MVSCamera] resolution fallback: requested "
                  << m_config.cam_width << "x" << m_config.cam_height
                  << ", actual " << m_width << "x" << m_height << std::endl;
    }

    // 设备端原始像素格式（审计用）
    MVCC_ENUMVALUE stPixelFormat{};
    if (MV_OK == MV_CC_GetEnumValue(m_handle, "PixelFormat", &stPixelFormat)) {
        MVCC_ENUMENTRY stEntry{};
        stEntry.nValue = stPixelFormat.nCurValue;
        if (MV_OK == MV_CC_GetEnumEntrySymbolic(m_handle, "PixelFormat", &stEntry))
            m_devicePixelFormat = stEntry.chSymbolic;
    }

    // GigE 专属：最佳包大小 + 心跳。USB 设备不得写 GigE 节点。
    if (m_transport == mvsdiag::MvsTransport::kGigE) {
        const int packetSize = MV_CC_GetOptimalPacketSize(m_handle);
        if (packetSize > 0) {
            nRet = MV_CC_SetIntValue(m_handle, "GevSCPSPacketSize", packetSize);
            if (MV_OK != nRet) {
                APP_LOG_EVENT_WARNING("camera.mvs",
                                      "configure",
                                      "packet_size_failed",
                                      {{"code", QString("0x%1").arg(nRet, 0, 16)},
                                       {"packet_size", QString::number(packetSize)}});
            }
        }
        nRet = MV_CC_SetIntValue(m_handle, "GevHeartbeatTimeout", kGevHeartbeatTimeoutMs);
        if (MV_OK != nRet) {
            APP_LOG_EVENT_WARNING("camera.mvs",
                                  "configure",
                                  "heartbeat_failed",
                                  {{"code", QString("0x%1").arg(nRet, 0, 16)},
                                   {"timeout_ms", QString::number(kGevHeartbeatTimeoutMs)}});
        }
    }

    return true;
}

bool MVSCamera::readbackActualParamsLocked() {
    MVCC_FLOATVALUE stExposure{};
    if (MV_OK == MV_CC_GetFloatValue(m_handle, "ExposureTime", &stExposure))
        m_exposureUs = static_cast<int>(stExposure.fCurValue);
    MVCC_FLOATVALUE stGain{};
    if (MV_OK == MV_CC_GetFloatValue(m_handle, "Gain", &stGain))
        m_gain = stGain.fCurValue;
    MVCC_ENUMVALUE stTrigger{};
    if (MV_OK == MV_CC_GetEnumValue(m_handle, "TriggerMode", &stTrigger))
        m_triggerOn = (stTrigger.nCurValue != 0);
    return true;
}

bool MVSCamera::ensureBufferCapacityLocked(size_t required) {
    if (required == 0)
        return false;
    // 溢出护栏：单帧不可能超过 512MiB
    constexpr size_t kMaxFrameBytes = 512ULL * 1024 * 1024;
    if (required > kMaxFrameBytes)
        return false;
    if (m_buffer.size() < required) {
        try {
            m_buffer.resize(required);
        } catch (const std::bad_alloc&) {
            return false;
        }
    }
    return true;
}

void MVSCamera::releaseResourcesLocked() {
    if (m_handle) {
        if (m_isGrabbing) {
            MV_CC_StopGrabbing(m_handle);
            m_isGrabbing = false;
        }
        if (m_deviceOpened) {
            MV_CC_CloseDevice(m_handle);
            m_deviceOpened = false;
        }
        if (m_handleCreated) {
            MV_CC_DestroyHandle(m_handle);
            m_handleCreated = false;
        }
        m_handle = nullptr;
    }
    if (m_sdkInitialized) {
        MV_CC_Finalize();
        m_sdkInitialized = false;
    }
    m_buffer.clear();
    m_buffer.shrink_to_fit();
    m_isOpened = false;
}

void MVSCamera::close() {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!m_isOpened && !m_handle && !m_sdkInitialized)
        return;

    APP_LOG_EVENT_INFO("camera.mvs",
                       "close",
                       "start",
                       {{"serial", QString::fromStdString(m_deviceSerial)},
                        {"frames_ok", QString::number(m_frameOkCount)},
                        {"frames_failed", QString::number(m_frameFailCount)}});
    releaseResourcesLocked();
    std::cout << "[MVSCamera] closed" << std::endl;
    APP_LOG_EVENT_INFO("camera.mvs", "close", "success", {});
}

cv::Mat MVSCamera::captureOneFrame(int timeoutMs) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!m_isOpened || !m_handle || m_buffer.empty())
        return cv::Mat();

    MV_FRAME_OUT_INFO_EX stFrameInfo;
    memset(&stFrameInfo, 0, sizeof(stFrameInfo));

    // 原始取流 + 显式转 BGR8。不使用 MV_CC_GetImageForBGR：
    // 该旧接口要求缓冲区容纳转换后的 BGR 数据（w*h*3），而 PayloadSize
    // 对 Mono8/Bayer8 只有 w*h*1，会返回 MV_E_BUFOVER 导致永远取不到帧。
    int nRet = MV_CC_GetOneFrameTimeout(m_handle, m_buffer.data(),
                                        static_cast<unsigned int>(m_buffer.size()),
                                        &stFrameInfo, timeoutMs);
    if (MV_OK != nRet) {
        if (MV_E_NODATA == nRet)
            MV_CC_ClearImageBuffer(m_handle);
        // 实际帧大于缓冲区（设备侧分辨率被外部改变）时安全扩容并重试一次
        if (MV_E_NOENOUGH_BUF == nRet) {
            MVCC_INTVALUE_EX stPayload{};
            if (MV_OK == MV_CC_GetIntValueEx(m_handle, "PayloadSize", &stPayload) &&
                stPayload.nCurValue > m_buffer.size() &&
                ensureBufferCapacityLocked(static_cast<size_t>(stPayload.nCurValue))) {
                memset(&stFrameInfo, 0, sizeof(stFrameInfo));
                nRet = MV_CC_GetOneFrameTimeout(m_handle, m_buffer.data(),
                                                static_cast<unsigned int>(m_buffer.size()),
                                                &stFrameInfo, timeoutMs);
                if (MV_OK == nRet)
                    goto frame_ok;
            }
        }
        ++m_frameFailCount;
        ++m_consecutiveFailures;
        // 取帧失败也更新 lastError（限频打印，避免超时风暴刷屏）
        {
            m_lastError.stage = "capture";
            m_lastError.sdk_code = nRet;
            m_lastError.message = mvsdiag::diagnostic("capture", nRet) +
                " timeout_ms=" + std::to_string(timeoutMs) +
                " consecutive=" + std::to_string(m_consecutiveFailures);
        }
        if (m_consecutiveFailures == 1 ||
            m_consecutiveFailures % kFailureLogInterval == 0) {
            std::cerr << "[MVSCamera] " << m_lastError.message << std::endl;
            APP_LOG_EVENT_WARNING("camera.mvs",
                                  "capture",
                                  "fail",
                                  {{"code", QString("0x%1").arg(nRet, 0, 16)},
                                   {"name", QString::fromLatin1(mvsdiag::errorName(nRet))},
                                   {"timeout_ms", QString::number(timeoutMs)},
                                   {"consecutive", QString::number(m_consecutiveFailures)},
                                   {"total_failed", QString::number(m_frameFailCount)},
                                   {"total_ok", QString::number(m_frameOkCount)}});
        }
        return cv::Mat();
    }

frame_ok:
    {
        const int w = static_cast<int>(stFrameInfo.nWidth);
        const int h = static_cast<int>(stFrameInfo.nHeight);
        const unsigned int srcType = static_cast<unsigned int>(stFrameInfo.enPixelType);
        const uint32_t frameLen = stFrameInfo.nFrameLen;
        if (w <= 0 || h <= 0 || frameLen == 0 ||
            frameLen > m_buffer.size()) {
            ++m_frameFailCount;
            ++m_consecutiveFailures;
            m_lastError.stage = "capture";
            m_lastError.sdk_code = 0;
            m_lastError.message = "帧头非法: w=" + std::to_string(w) +
                " h=" + std::to_string(h) +
                " len=" + std::to_string(frameLen) +
                " buffer=" + std::to_string(m_buffer.size());
            APP_LOG_EVENT_ERROR("camera.mvs",
                                "capture",
                                "invalid_frame",
                                {{"frame", QString("%1x%2").arg(w).arg(h)},
                                 {"len", QString::number(frameLen)},
                                 {"buffer", QString::number(m_buffer.size())}});
            return cv::Mat();
        }

        // 输出拥有独立内存的 CV_8UC3 BGR，SDK 缓冲区生命周期不外泄
        cv::Mat frame(h, w, CV_8UC3);
        const uint64_t bgrBytes = static_cast<uint64_t>(w) *
                                  static_cast<uint64_t>(h) * 3ULL;
        if (srcType == PixelType_Gvsp_BGR8_Packed) {
            if (frameLen < bgrBytes) {
                ++m_frameFailCount;
                ++m_consecutiveFailures;
                return cv::Mat();
            }
            memcpy(frame.data, m_buffer.data(), static_cast<size_t>(bgrBytes));
        } else {
            MV_CC_PIXEL_CONVERT_PARAM_EX cvt;
            memset(&cvt, 0, sizeof(cvt));
            cvt.nWidth = static_cast<unsigned int>(w);
            cvt.nHeight = static_cast<unsigned int>(h);
            cvt.enSrcPixelType = stFrameInfo.enPixelType;
            cvt.pSrcData = m_buffer.data();
            cvt.nSrcDataLen = frameLen;
            cvt.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            cvt.pDstBuffer = frame.data;
            cvt.nDstBufferSize = static_cast<unsigned int>(bgrBytes);
            nRet = MV_CC_ConvertPixelTypeEx(m_handle, &cvt);
            if (MV_OK != nRet) {
                ++m_frameFailCount;
                ++m_consecutiveFailures;
                m_lastError.stage = "convert";
                m_lastError.sdk_code = nRet;
                m_lastError.message = mvsdiag::diagnostic("convert", nRet) +
                    " src_type=0x" + ([&]{ char b[16]; std::snprintf(b, sizeof(b), "%08x", srcType); return std::string(b); }()) +
                    " frame=" + std::to_string(w) + "x" + std::to_string(h);
                std::cerr << "[MVSCamera] " << m_lastError.message << std::endl;
                APP_LOG_EVENT_ERROR("camera.mvs",
                                    "capture",
                                    "convert_fail",
                                    {{"code", QString("0x%1").arg(nRet, 0, 16)},
                                     {"name", QString::fromLatin1(mvsdiag::errorName(nRet))},
                                     {"src_type", QString("0x%1").arg(srcType, 0, 16)},
                                     {"frame", QString("%1x%2").arg(w).arg(h)}});
                return cv::Mat();
            }
        }

        ++m_frameOkCount;
        if (m_consecutiveFailures > 0) {
            APP_LOG_EVENT_INFO("camera.mvs",
                               "capture",
                               "recovered",
                               {{"previous_consecutive_failures", QString::number(m_consecutiveFailures)}});
            m_consecutiveFailures = 0;
        }
        if (m_frameOkCount == 1 || m_frameOkCount % 300 == 0) {
            const uint64_t devTs = (static_cast<uint64_t>(stFrameInfo.nDevTimeStampHigh) << 32) |
                                   stFrameInfo.nDevTimeStampLow;
            APP_LOG_EVENT_DEBUG("camera.mvs",
                                "capture",
                                "frame",
                                {{"frame_num", QString::number(stFrameInfo.nFrameNum)},
                                 {"device_ts", QString::number(devTs)},
                                 {"size", QString("%1x%2").arg(w).arg(h)},
                                 {"src_type", QString("0x%1").arg(srcType, 0, 16)},
                                 {"host_output_format", "BGR8"},
                                 {"frames_ok", QString::number(m_frameOkCount)}});
        }
        return frame;
    }
}

int MVSCamera::captureOneFrameWithDepth(cv::Mat& bgrImage, cv::Mat& depthImage, int timeoutMs) {
    bgrImage = captureOneFrame(timeoutMs);
    depthImage = cv::Mat();
    return bgrImage.empty() ? -1 : 0;
}

void MVSCamera::discardFramesLocked(int count, int timeoutMs) {
    if (!m_isGrabbing)
        return;
    MV_CC_ClearImageBuffer(m_handle);
    for (int i = 0; i < count; ++i) {
        MV_FRAME_OUT_INFO_EX stDiscard{};
        MV_CC_GetOneFrameTimeout(m_handle, m_buffer.data(),
                                 static_cast<unsigned int>(m_buffer.size()),
                                 &stDiscard, timeoutMs);
    }
}

bool MVSCamera::setExposureLocked(int exposureUs) {
    const bool wasGrabbing = m_isGrabbing;
    if (wasGrabbing) {
        MV_CC_StopGrabbing(m_handle);
        m_isGrabbing = false;
    }

    int nRet = MV_CC_SetFloatValue(m_handle, "ExposureTime", (float)exposureUs);

    if (wasGrabbing) {
        const int startRet = MV_CC_StartGrabbing(m_handle);
        if (MV_OK != startRet) {
            setError("set_exposure", startRet, "restart grabbing failed");
            return false;
        }
        m_isGrabbing = true;
    }

    if (MV_OK != nRet) {
        setError("set_exposure", nRet, "value_us=" + std::to_string(exposureUs));
        return false;
    }

    // 清空旧缓冲并丢弃一帧，避免旧曝光帧进入标定
    discardFramesLocked(kDiscardFramesAfterParamChange, 200);

    // 回读实际值
    MVCC_FLOATVALUE stValue{};
    if (MV_OK == MV_CC_GetFloatValue(m_handle, "ExposureTime", &stValue)) {
        m_exposureUs = static_cast<int>(stValue.fCurValue);
        if (m_exposureUs != exposureUs) {
            APP_LOG_EVENT_WARNING("camera.mvs",
                                  "set_exposure",
                                  "value_adjusted",
                                  {{"requested_us", QString::number(exposureUs)},
                                   {"actual_us", QString::number(m_exposureUs)}});
        }
    }
    m_exposureCached = m_exposureUs;
    APP_LOG_EVENT_INFO("camera.mvs",
                       "set_exposure",
                       "success",
                       {{"requested_us", QString::number(exposureUs)},
                        {"actual_us", QString::number(m_exposureUs)}});
    return true;
}

bool MVSCamera::setColorExposureTime(int exposureUs) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!m_isOpened || !m_handle) return false;
    return setExposureLocked(exposureUs);
}

int MVSCamera::getColorExposureTime() {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!m_isOpened || !m_handle) return -1;
    MVCC_FLOATVALUE stValue{};
    int nRet = MV_CC_GetFloatValue(m_handle, "ExposureTime", &stValue);
    if (MV_OK == nRet) return (int)(stValue.fCurValue + 0.5f);
    return -1;
}

bool MVSCamera::setAutoExposureLocked(bool enable) {
    const bool wasGrabbing = m_isGrabbing;
    if (wasGrabbing) {
        MV_CC_StopGrabbing(m_handle);
        m_isGrabbing = false;
    }

    const unsigned int mode = enable ? 2 : 0;
    int nRet = MV_CC_SetEnumValue(m_handle, "ExposureAuto", mode);

    if (wasGrabbing) {
        const int startRet = MV_CC_StartGrabbing(m_handle);
        if (MV_OK != startRet) {
            setError("set_auto_exposure", startRet, "restart grabbing failed");
            return false;
        }
        m_isGrabbing = true;
    }

    if (MV_OK != nRet) {
        setError("set_auto_exposure", nRet, enable ? "enable" : "disable");
        return false;
    }

    discardFramesLocked(kDiscardFramesAfterParamChange, 200);
    m_autoExposureCached = enable;
    APP_LOG_EVENT_INFO("camera.mvs",
                       "set_auto_exposure",
                       "success",
                       {{"enable", enable ? "true" : "false"}});
    return true;
}

bool MVSCamera::setAutoExposure(bool enable) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!m_isOpened || !m_handle) return false;
    return setAutoExposureLocked(enable);
}

bool MVSCamera::getAutoExposure() {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!m_isOpened || !m_handle) return false;
    MVCC_ENUMVALUE stValue{};
    int nRet = MV_CC_GetEnumValue(m_handle, "ExposureAuto", &stValue);
    if (MV_OK == nRet) return stValue.nCurValue != 0;
    return false;
}

bool MVSCamera::getExposureRange(int& minUs, int& maxUs) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!m_isOpened || !m_handle) return false;
    MVCC_FLOATVALUE stValue{};
    int nRet = MV_CC_GetFloatValue(m_handle, "ExposureTime", &stValue);
    if (MV_OK == nRet) {
        minUs = static_cast<int>(stValue.fMin);
        maxUs = static_cast<int>(stValue.fMax);
        return true;
    }
    return false;
}

bool MVSCamera::setColorGain(int gain) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!m_isOpened || !m_handle) return false;
    int nRet = MV_CC_SetFloatValue(m_handle, "Gain", (float)gain);
    if (MV_OK != nRet) {
        setError("set_gain", nRet, "value=" + std::to_string(gain));
        return false;
    }
    discardFramesLocked(kDiscardFramesAfterParamChange, 200);
    MVCC_FLOATVALUE stValue{};
    if (MV_OK == MV_CC_GetFloatValue(m_handle, "Gain", &stValue))
        m_gain = stValue.fCurValue;
    APP_LOG_EVENT_INFO("camera.mvs",
                       "set_gain",
                       "success",
                       {{"requested", QString::number(gain)},
                        {"actual", QString::number(m_gain, 'f', 2)}});
    return true;
}

int MVSCamera::getColorGain() {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!m_isOpened || !m_handle) return -1;
    MVCC_FLOATVALUE stValue{};
    int nRet = MV_CC_GetFloatValue(m_handle, "Gain", &stValue);
    // 设备浮点增益可能量化（如设 5.0 应用为 4.999），四舍五入避免界面显示掉档
    if (MV_OK == nRet) return (int)(stValue.fCurValue + 0.5f);
    return -1;
}

bool MVSCamera::getGainRange(int& min, int& max) {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!m_isOpened || !m_handle) return false;
    MVCC_FLOATVALUE stValue{};
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
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
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
        m_isGrabbing = true;
        // 丢弃第一帧，避免拿到旧曝光帧
        discardFramesLocked(1, 200);
        cv::Mat frame = captureOneFrame(500);
        MV_CC_StopGrabbing(m_handle);
        m_isGrabbing = false;
        if (!frame.empty()) images.push_back(frame);
    }

    MV_CC_SetFloatValue(m_handle, "ExposureTime", (float)prevExposure);
    if (wasAuto) MV_CC_SetEnumValue(m_handle, "ExposureAuto", 2);

    if (wasGrabbing) {
        MV_CC_StartGrabbing(m_handle);
        m_isGrabbing = true;
        discardFramesLocked(1, 200);
    }
    return images;
}

std::vector<std::vector<float>> MVSCamera::getCameraIntrinsics(const int imgWidth, const int imgHight) {
    return std::vector<std::vector<float>>();
}
