#include "OrbbecCamera.hpp"
#include "exposure_fusion.hpp"
#include <iostream>
#include <thread>
#include <chrono>

OrbbecCameraV2::OrbbecCameraV2(const CONFIG& config)
    : AICamera(config) {
}

OrbbecCameraV2::~OrbbecCameraV2() {
    close();
}

bool OrbbecCameraV2::open() {
    if (m_isOpened) {
        return true;
    }

    try {
        // 1. 创建 Context（只创建一次）
        m_context = std::make_shared<ob::Context>();
        m_context->enableNetDeviceEnumeration(true);
        
        // 给设备一些时间进行网络广播
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 2. 查找并连接设备（最多尝试10次）
        std::shared_ptr<ob::DeviceList> deviceList;
        bool deviceFound = false;
        
        for (int attempt = 0; attempt < 10 && !deviceFound; attempt++) {
            deviceList = m_context->queryDeviceList();
            
            if (deviceList && deviceList->deviceCount() > 0) {
                // 打印所有发现的设备
                std::cout << "Found " << deviceList->deviceCount() << " device(s):" << std::endl;
                for (uint32_t i = 0; i < deviceList->deviceCount(); i++) {
                    auto device = deviceList->getDevice(i);
                    auto info = device->getDeviceInfo();
                    std::cout << "  Device " << i << ": " 
                              << info->name() << " (SN: " << info->serialNumber() << ")" << std::endl;
                }
                
                // 由于无法直接获取IP，尝试连接所有设备直到找到正确的
                // 或者通过设备序号/序列号来识别
                for (uint32_t i = 0; i < deviceList->deviceCount(); i++) {
                    auto device = deviceList->getDevice(i);
                    auto info = device->getDeviceInfo();
                    
                    // 如果配置中指定了设备序列号，用序列号匹配
                    // 否则直接使用第一个网络设备
                    if (m_config.cam_sn.empty()) {
                        // 没有指定序列号，使用第一个设备
                        m_device = device;
                        deviceFound = true;
                        std::cout << "Connected to camera (first device): " << info->name() 
                                  << ", SN: " << info->serialNumber() << std::endl;
                        break;
                    } else if (info->serialNumber() == m_config.cam_sn) {
                        // 通过序列号匹配
                        m_device = device;
                        deviceFound = true;
                        std::cout << "Connected to camera: " << m_config.cam_ip 
                                  << ", SN: " << info->serialNumber() << std::endl;
                        break;
                    }
                }
            }
            
            if (!deviceFound && attempt < 9) {
                std::cout << "Waiting for device... (attempt " << attempt + 1 << "/10)" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        
        if (!deviceFound || !m_device) {
            std::cerr << "Failed to find camera";
            if (!m_config.cam_sn.empty()) {
                std::cerr << " with SN: " << m_config.cam_sn;
            }
            std::cerr << std::endl;
            std::cerr << "Please check:" << std::endl;
            std::cerr << "  1. Camera is powered on" << std::endl;
            std::cerr << "  2. Network cable is connected" << std::endl;
            std::cerr << "  3. Camera and PC are in the same subnet" << std::endl;
            std::cerr << "  4. Orbbec SDK version supports network devices" << std::endl;
            return false;
        }
        
        // 3. 验证设备连接并打印设备信息
        auto deviceInfo = m_device->getDeviceInfo();
        std::cout << "Device Info: " << deviceInfo->name() 
                  << ", PID: 0x" << std::hex << deviceInfo->pid() << std::dec
                  << ", SN: " << deviceInfo->serialNumber()
                  << ", FW: " << deviceInfo->firmwareVersion() << std::endl;
        
        // 4. 创建 Pipeline
        m_pipeline = std::make_shared<ob::Pipeline>(m_device);
        
        // 5. 配置流
        m_configPtr = std::make_shared<ob::Config>();
        
        // 获取彩色流配置列表
        auto colorProfiles = m_pipeline->getStreamProfileList(OB_SENSOR_COLOR);
        if (!colorProfiles || colorProfiles->count() == 0) {
            std::cerr << "No color stream profiles available!" << std::endl;
            return false;
        }
        
        // 打印所有可用的彩色流配置
        std::cout << "Available color profiles:" << std::endl;
        for (uint32_t i = 0; i < colorProfiles->count(); i++) {
            auto profile = colorProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
            if (profile) {
                std::cout << "  " << profile->width() << "x" << profile->height() 
                          << " @ " << profile->fps() << "fps, Format: " << profile->format() << std::endl;
            }
        }
        
        // 尝试精确匹配
        auto colorProfile = colorProfiles->getVideoStreamProfile(
            m_config.cam_width, m_config.cam_height, OB_FORMAT_MJPEG, m_config.fps);
        
        if (!colorProfile) {
            // 如果 MJPEG 匹配失败，尝试RGB888
            colorProfile = colorProfiles->getVideoStreamProfile(
                m_config.cam_width, m_config.cam_height, OB_FORMAT_RGB888, m_config.fps);
        }
        
        if (!colorProfile) {
            // 如果精确匹配失败，使用第一个可用的配置
            std::cout << "Exact match not found, using default profile" << std::endl;
            colorProfile = colorProfiles->getProfile(OB_PROFILE_DEFAULT)
                ->as<ob::VideoStreamProfile>();
        }
        
        if (colorProfile) {
            m_configPtr->enableStream(colorProfile);
            auto videoProfile = colorProfile->as<ob::VideoStreamProfile>();
            std::cout << "Using color: " << videoProfile->width() << "x" << videoProfile->height() 
                      << " @ " << videoProfile->fps() << "fps, Format: " << videoProfile->format() << std::endl;
        } else {
            std::cerr << "Failed to get any color profile!" << std::endl;
            return false;
        }
        
        // 可选：启用深度流
        if (m_config.enableDepth) {
            auto depthProfiles = m_pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
            if (depthProfiles && depthProfiles->count() > 0) {
                auto depthProfile = depthProfiles->getVideoStreamProfile(
                    m_config.cam_width, m_config.cam_height, OB_FORMAT_Y16, m_config.fps);
                if (depthProfile) {
                    m_configPtr->enableStream(depthProfile);
                    std::cout << "Depth stream enabled" << std::endl;
                }
            }
        }
        
        // Create a format converter filter.
        m_formatConverter = std::make_shared<ob::FormatConvertFilter>();

        // 6. 设置帧聚合模式
        m_configPtr->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
        
        // 7. 启动 Pipeline
        std::cout << "Starting pipeline..." << std::endl;
        m_pipeline->start(m_configPtr);
        
        // 等待 pipeline 稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 8. 验证启动：等待第一帧数据
        bool frameReceived = false;
        for (int retry = 0; retry < 5; retry++) {
            auto frameSet = m_pipeline->waitForFrameset(3000);
            if (frameSet) {
                auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                if (colorFrame) {
                    frameReceived = true;
                    std::cout << "First frame received successfully" << std::endl;
                    break;
                }
            }
            if (retry < 4) {
                std::cout << "Waiting for first frame... (attempt " << retry + 1 << "/5)" << std::endl;
            }
        }
        
        if (!frameReceived) {
            std::cerr << "Pipeline started but no frame received within timeout" << std::endl;
            close();
            return false;
        }
        
        m_isOpened = true;
        std::cout << "Camera started: " << m_config.cam_width << "x" << m_config.cam_height
                  << " @ " << m_config.fps << "fps" << std::endl;
        
        return true;
    }
    catch (const ob::Error& e) {
        std::cerr << "Open failed: " << e.getMessage() << std::endl;
        close();
        return false;
    }
}


void OrbbecCameraV2::close() {
    if (!m_isOpened) return;

    try {
        if (m_pipeline) {
            m_pipeline->stop();
            m_pipeline.reset();
            m_pipeline = nullptr;
        }
        m_formatConverter.reset();
        m_configPtr.reset();
        m_device.reset();
        m_context.reset();

        m_isOpened = false;
        std::cout << "Camera closed" << std::endl;
    }
    catch (const ob::Error& e) {
        std::cerr << "Close error: " << e.getMessage() << std::endl;
    }
}

cv::Mat OrbbecCameraV2::captureOneFrame(int timeoutMs) {
    if (!m_isOpened) {
        std::cerr << "Camera not opened" << std::endl;
        return cv::Mat();
    }

    try {
        // 等待帧集合
        auto frameSet = m_pipeline->waitForFrameset(timeoutMs);
        if (!frameSet) {
            std::cerr << "Wait for frameset timeout" << std::endl;
            return cv::Mat();
        }

        // 获取彩色帧
        auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
        if (!colorFrame) {
            std::cerr << "Failed to get color frame" << std::endl;
            return cv::Mat();
        }

        // Convert the color frame to RGB format.
        if (colorFrame->format() != OB_FORMAT_RGB) {
            if (colorFrame->format() == OB_FORMAT_MJPG) {
                m_formatConverter->setFormatConvertType(FORMAT_MJPG_TO_RGB);
            }
            else if (colorFrame->format() == OB_FORMAT_UYVY) {
                m_formatConverter->setFormatConvertType(FORMAT_UYVY_TO_RGB);
            }
            else if (colorFrame->format() == OB_FORMAT_YUYV) {
                m_formatConverter->setFormatConvertType(FORMAT_YUYV_TO_RGB);
            }
            else {
                std::cout << "Color format is not support!" << std::endl;
            }
            colorFrame = m_formatConverter->process(colorFrame)->as<ob::ColorFrame>();
        }
        // Processed the color frames to BGR format, use OpenCV to save to disk.
        m_formatConverter->setFormatConvertType(FORMAT_RGB_TO_BGR);
        colorFrame = m_formatConverter->process(colorFrame)->as<ob::ColorFrame>();


        // 获取帧数据
        auto videoFrame = colorFrame->as<const ob::VideoFrame>();
        int width = videoFrame->getWidth();
        int height = videoFrame->getHeight();
        void* data = videoFrame->getData();

        if (!data || width <= 0 || height <= 0) {
            std::cerr << "Invalid frame data" << std::endl;
            return cv::Mat();
        }

        // 转换为 OpenCV Mat (RGB 格式)
        cv::Mat bgrImage(cv::Size(width, height), CV_8UC3, data);

        // 深拷贝
        return bgrImage.clone();

    }
    catch (const ob::Error& e) {
        std::cerr << "Capture error: " << e.getMessage() << std::endl;
        return cv::Mat();
    }
}



int OrbbecCameraV2::captureOneFrameWithDepth(cv::Mat& bgrImage, cv::Mat& depthImage, int timeoutMs) {
    if (!m_isOpened) {
        std::cerr << "Camera not opened" << std::endl;
        return -1;
    }

    try {
        std::shared_ptr<ob::Frame> colorFrame = nullptr;
        std::shared_ptr<ob::Frame> depthFrame = nullptr;
        for (size_t loop = 0; loop < 3; loop++)
        {
            // 等待帧集合
            auto frameSet = m_pipeline->waitForFrameset(timeoutMs);
            if (!frameSet) {
                std::cerr << "Wait for frameset timeout" << std::endl;
                continue;
            }

            // 获取彩色帧
            if (nullptr == colorFrame)
            {
                colorFrame = frameSet->getFrame(OB_FRAME_COLOR);

                // Convert the color frame to RGB format.
                if (colorFrame->format() != OB_FORMAT_RGB) {
                    if (colorFrame->format() == OB_FORMAT_MJPG) {
                        m_formatConverter->setFormatConvertType(FORMAT_MJPG_TO_RGB);
                    }
                    else if (colorFrame->format() == OB_FORMAT_UYVY) {
                        m_formatConverter->setFormatConvertType(FORMAT_UYVY_TO_RGB);
                    }
                    else if (colorFrame->format() == OB_FORMAT_YUYV) {
                        m_formatConverter->setFormatConvertType(FORMAT_YUYV_TO_RGB);
                    }
                    else {
                        std::cout << "Color format is not support!" << std::endl;
                    }
                    colorFrame = m_formatConverter->process(colorFrame)->as<ob::ColorFrame>();
                }
                // Processed the color frames to BGR format, use OpenCV to save to disk.

                // m_formatConverter->setFormatConvertType(FORMAT_RGB_TO_BGR);
                // colorFrame = m_formatConverter->process(colorFrame)->as<ob::ColorFrame>();

                if (colorFrame) {
                    auto videoFrame = colorFrame->as<const ob::VideoFrame>();
                    int width = videoFrame->getWidth();
                    int height = videoFrame->getHeight();
                    void* data = videoFrame->getData();

                    if (data && width > 0 && height > 0) {
                        cv::Mat rgbImage(cv::Size(width, height), CV_8UC3, data);
                        // rgbImage = rgbImage.clone();
                        cv::cvtColor(rgbImage, bgrImage, cv::COLOR_RGB2BGR);
                    }
                }
            }

            // 获取深度帧
            if (nullptr == depthFrame)
            {
                depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
            }

            if (nullptr == depthFrame || nullptr == colorFrame)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            else
            {
                std::shared_ptr<ob::DepthFrame> depthData = depthFrame->as<ob::DepthFrame>();
                cv::Mat depthMat(depthData->height(), depthData->width(), CV_16UC1, depthData->data());
                // float scale = depthData->getValueScale();
                // std::cout<< "scale  "<< scale<<std::endl;
                depthImage = depthMat.clone();
                break;
            }
        }
        return 0;
    }
    catch (const ob::Error& e) {
        std::cerr << "Capture with depth error: " << e.getMessage() << std::endl;
        return -1;
    }
}



cv::Mat decodeMJPEGToBGR(const uint8_t* data, int dataSize) {
    // 将 MJPEG 数据编码为 vector，然后用 OpenCV 解码
    std::vector<uchar> jpegData(data, data + dataSize);
    cv::Mat img = cv::imdecode(jpegData, cv::IMREAD_COLOR);
    return img;  // cv::imdecode 直接返回 BGR 格式
}

// ========== 内参接口 ==========
std::vector<std::vector<float>> OrbbecCameraV2::getCameraIntrinsics(const int imgWidth, const int imgHight) {
    // 直接指定大小并初始化为0
    std::vector<std::vector<float>> matrix(3, std::vector<float>(3, 0.0f));

    //# [608.212,   0,      648.738]
    //# [0,      608.21,   392.18 ]
    //# [0,        0,        1    ]

    matrix[0][0] = 608.212;
    matrix[1][1] = 608.21;

    matrix[0][2] = 648.738;
    matrix[1][2] = 392.18;

    matrix[2][2] = 1;

    return matrix;
}

// ========== 曝光控制相关函数实现（基于 SDK v2.7.6 的简化版）==========

bool OrbbecCameraV2::setColorExposureTime(int exposureUs) {
    if (!m_isOpened || !m_device) {
        std::cerr << "Camera not opened or device not available" << std::endl;
        m_exposureCached = exposureUs;
        return false;
    }
    
    try {
        // 检查设备是否支持曝光时间设置
        if (!m_device->isPropertySupported(OB_PROP_COLOR_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
            std::cerr << "Device does not support color exposure setting" << std::endl;
            m_exposureCached = exposureUs;
            return true;
        }
        
        // 获取曝光范围（可选，用于验证）
        if (m_device->isPropertySupported(OB_PROP_COLOR_EXPOSURE_INT, OB_PERMISSION_READ)) {
            auto range = m_device->getIntPropertyRange(OB_PROP_COLOR_EXPOSURE_INT);
            if (exposureUs < range.min || exposureUs > range.max) {
                std::cerr << "Exposure value " << exposureUs << " us out of range [" 
                          << range.min << ", " << range.max << "]" << std::endl;
                return false;
            }
        }
        
        // 设置曝光时间
        m_device->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, exposureUs);
        m_exposureCached = exposureUs;
        std::cout << "Exposure time set to " << exposureUs << " us" << std::endl;
        return true;
    }
    catch (const ob::Error& e) {
        std::cerr << "Set exposure time error: " << e.getMessage() << std::endl;
        m_exposureCached = exposureUs;
        return false;
    }
}

int OrbbecCameraV2::getColorExposureTime() {
    if (!m_isOpened || !m_device) {
        std::cerr << "Camera not opened" << std::endl;
        return m_exposureCached > 0 ? m_exposureCached : -1;
    }
    
    try {
        if (!m_device->isPropertySupported(OB_PROP_COLOR_EXPOSURE_INT, OB_PERMISSION_READ)) {
            std::cerr << "Device does not support reading color exposure" << std::endl;
            return m_exposureCached > 0 ? m_exposureCached : 10000;
        }
        
        int value = m_device->getIntProperty(OB_PROP_COLOR_EXPOSURE_INT);
        m_exposureCached = value;
        return value;
    }
    catch (const ob::Error& e) {
        std::cerr << "Get exposure time error: " << e.getMessage() << std::endl;
        return m_exposureCached > 0 ? m_exposureCached : -1;
    }
}

bool OrbbecCameraV2::setAutoExposure(bool enable) {
    if (!m_isOpened || !m_device) {
        std::cerr << "Camera not opened" << std::endl;
        m_autoExposureCached = enable;
        return false;
    }
    
    try {
        if (!m_device->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
            std::cerr << "Device does not support auto exposure setting" << std::endl;
            m_autoExposureCached = enable;
            return true;
        }
        
        m_device->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, enable);
        m_autoExposureCached = enable;
        std::cout << "Auto exposure " << (enable ? "enabled" : "disabled") << std::endl;
        return true;
    }
    catch (const ob::Error& e) {
        std::cerr << "Set auto exposure error: " << e.getMessage() << std::endl;
        m_autoExposureCached = enable;
        return false;
    }
}

bool OrbbecCameraV2::getAutoExposure() {
    if (!m_isOpened || !m_device) {
        return m_autoExposureCached;
    }
    
    try {
        if (!m_device->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_READ)) {
            return m_autoExposureCached;
        }
        
        return m_device->getBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL);
    }
    catch (const ob::Error& e) {
        std::cerr << "Get auto exposure status error: " << e.getMessage() << std::endl;
        return m_autoExposureCached;
    }
}

bool OrbbecCameraV2::getExposureRange(int& minUs, int& maxUs) {
    if (!m_isOpened || !m_device) {
        std::cerr << "Camera not opened" << std::endl;
        minUs = 50;
        maxUs = 1999;
        return true;
    }
    
    try {
        if (!m_device->isPropertySupported(OB_PROP_COLOR_EXPOSURE_INT, OB_PERMISSION_READ)) {
            minUs = 50;
            maxUs = 1999;
            return true;
        }
        
        auto range = m_device->getIntPropertyRange(OB_PROP_COLOR_EXPOSURE_INT);
        minUs = range.min;
        maxUs = range.max;
        return true;
    }
    catch (const ob::Error& e) {
        std::cerr << "Get exposure range error: " << e.getMessage() << std::endl;
        minUs = 50;
        maxUs = 1999;
        return true;
    }
}

bool OrbbecCameraV2::setColorGain(int gain) {
    if (!m_isOpened || !m_device) {
        std::cerr << "Camera not opened" << std::endl;
        return false;
    }
    try {
        if (!m_device->isPropertySupported(OB_PROP_COLOR_GAIN_INT, OB_PERMISSION_WRITE)) {
            std::cerr << "Device does not support color gain setting" << std::endl;
            return false;
        }
        m_device->setIntProperty(OB_PROP_COLOR_GAIN_INT, gain);
        std::cout << "Color gain set to " << gain << std::endl;
        return true;
    }
    catch (const ob::Error& e) {
        std::cerr << "Set color gain error: " << e.getMessage() << std::endl;
        return false;
    }
}

int OrbbecCameraV2::getColorGain() {
    if (!m_isOpened || !m_device) {
        std::cerr << "Camera not opened" << std::endl;
        return -1;
    }
    try {
        if (!m_device->isPropertySupported(OB_PROP_COLOR_GAIN_INT, OB_PERMISSION_READ)) {
            std::cerr << "Device does not support reading color gain" << std::endl;
            return -1;
        }
        return m_device->getIntProperty(OB_PROP_COLOR_GAIN_INT);
    }
    catch (const ob::Error& e) {
        std::cerr << "Get color gain error: " << e.getMessage() << std::endl;
        return -1;
    }
}

bool OrbbecCameraV2::getGainRange(int& min, int& max) {
    if (!m_isOpened || !m_device) {
        std::cerr << "Camera not opened" << std::endl;
        min = 0; max = 255;
        return true;
    }
    try {
        if (!m_device->isPropertySupported(OB_PROP_COLOR_GAIN_INT, OB_PERMISSION_READ)) {
            min = 0; max = 255;
            return true;
        }
        auto range = m_device->getIntPropertyRange(OB_PROP_COLOR_GAIN_INT);
        min = range.min;
        max = range.max;
        return true;
    }
    catch (const ob::Error& e) {
        std::cerr << "Get gain range error: " << e.getMessage() << std::endl;
        min = 0; max = 255;
        return true;
    }
}

// ========== 多曝光采集与融合 ==========

// 实现多曝光采集的虚函数
std::vector<cv::Mat> OrbbecCameraV2::captureMultiExposureImagesImpl(const std::vector<int>& exposureValuesUs) {
    return captureMultiExposureImages(exposureValuesUs);
}

std::vector<cv::Mat> OrbbecCameraV2::captureMultiExposureImages(const std::vector<int>& exposureValuesUs) {
    std::vector<cv::Mat> images;
    
    if (!m_isOpened) {
        std::cerr << "Camera not opened" << std::endl;
        return images;
    }
    
    // 保存原始自动曝光状态
    bool originalAutoExposure = getAutoExposure();
    int originalExposure = getColorExposureTime();
    
    // 关闭自动曝光
    setAutoExposure(false);
    
    for (int exposureUs : exposureValuesUs) {
        // 设置曝光时间
        if (setColorExposureTime(exposureUs)) {
            // 等待曝光稳定（根据曝光时间动态调整）
            int waitMs = std::min(std::max(exposureUs / 1000, 50), 200);
            std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
            
            // 采集一帧
            cv::Mat frame = captureOneFrame(2000);
            if (!frame.empty()) {
                images.push_back(frame);
                std::cout << "Captured at exposure: " << exposureUs << " us" << std::endl;
            } else {
                std::cerr << "Failed to capture at exposure: " << exposureUs << " us" << std::endl;
            }
        }
    }
    
    // 恢复原始设置
    if (originalAutoExposure) {
        setAutoExposure(true);
    } else {
        setAutoExposure(false);
        setColorExposureTime(originalExposure);
    }
    
    std::cout << "Captured " << images.size() << " images for multi-exposure fusion" << std::endl;
    return images;
}

cv::Mat OrbbecCameraV2::captureAndFuseMultiExposure(const std::vector<int>& exposureValuesUs,
                                                    float contrastWeight,
                                                    float saturationWeight,
                                                    float wellExposedWeight) {
    // 采集多曝光图像
    std::vector<cv::Mat> images = captureMultiExposureImages(exposureValuesUs);
    
    if (images.size() < 2) {
        std::cerr << "Need at least 2 images for fusion, got " << images.size() << std::endl;
        return cv::Mat();
    }
    
    // 创建融合参数
    ExposureFusion::Params params;
    params.contrastWeight = contrastWeight;
    params.saturationWeight = saturationWeight;
    params.wellExposedWeight = wellExposedWeight;
    
    // 执行融合
    std::cout << "Performing exposure fusion..." << std::endl;
    cv::Mat result = ExposureFusion::fuse(images, params);
    
    return result;
}

cv::Mat OrbbecCameraV2::captureAndFuseMultiExposureFast(const std::vector<int>& exposureValuesUs) {
    // 采集多曝光图像
    std::vector<cv::Mat> images = captureMultiExposureImages(exposureValuesUs);
    
    if (images.size() < 2) {
        std::cerr << "Need at least 2 images for fusion, got " << images.size() << std::endl;
        return cv::Mat();
    }
    
    // 快速融合
    std::cout << "Performing fast exposure fusion..." << std::endl;
    cv::Mat result = ExposureFusion::fuseFast(images);
    
    return result;
}