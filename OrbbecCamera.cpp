#include "OrbbecCamera.hpp"
#include <iostream>

OrbbecCameraV2::OrbbecCameraV2(const CONFIG& config)
    : m_config(config) {
}

OrbbecCameraV2::~OrbbecCameraV2() {
    close();
}

bool OrbbecCameraV2::open() {
    if (m_isOpened) {
        return true;
    }

    try {
        // 1. 创建 Context
        m_context = std::make_shared<ob::Context>();

        // 2. 连接网络设备
        m_device = m_context->createNetDevice(m_config.cam_ip.c_str(), m_config.cam_port);
        if (!m_device) {
            std::cerr << "Failed to connect to camera: " << m_config.cam_ip << std::endl;
            return false;
        }
        std::cout << "Connected to camera: " << m_config.cam_ip << std::endl;

        // 3. 创建 Pipeline (绑定到指定设备)
        m_pipeline = std::make_shared<ob::Pipeline>(m_device);

        // 4. 配置彩色流
        m_configPtr = std::make_shared<ob::Config>();

        // 获取彩色流配置列表
        auto colorProfiles = m_pipeline->getStreamProfileList(OB_SENSOR_COLOR);
        if (colorProfiles) {
            auto colorProfile = colorProfiles->getVideoStreamProfile(
                m_config.cam_width, m_config.cam_height, OB_FORMAT_RGB888, m_config.fps);
            if (colorProfile) {
                m_configPtr->enableStream(colorProfile);
            }
            else {
                // 如果找不到精确匹配，使用默认配置
                auto defaultProfile = colorProfiles->getProfile(OB_PROFILE_DEFAULT)
                    ->as<ob::VideoStreamProfile>();
                m_configPtr->enableStream(defaultProfile);
                std::cout << "Using default color profile" << std::endl;
            }
        }

        // 5. 可选：启用深度流
        if (m_config.enableDepth) {
            auto depthProfiles = m_pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
            if (depthProfiles) {
                auto depthProfile = depthProfiles->getProfile(OB_PROFILE_DEFAULT)
                    ->as<ob::VideoStreamProfile>();
                m_configPtr->enableStream(depthProfile);
            }
        }

        // 6. 启动 Pipeline
        m_pipeline->start(m_configPtr);

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
        }
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
        cv::Mat rgbImage(cv::Size(width, height), CV_8UC3, data);
        cv::Mat bgrImage;
        cv::cvtColor(rgbImage, bgrImage, cv::COLOR_RGB2BGR);

        // 深拷贝
        return bgrImage.clone();

    }
    catch (const ob::Error& e) {
        std::cerr << "Capture error: " << e.getMessage() << std::endl;
        return cv::Mat();
    }
}