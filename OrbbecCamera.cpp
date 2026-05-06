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

        const int verifyTimeoutMs = 5000; // 建议设置一个较长的超时时间，如2秒
        std::shared_ptr<ob::FrameSet> frameSet = nullptr;

        for (size_t i = 0; i < 5; i++)
        {
            // 1. 创建 Context
            m_context = std::make_shared<ob::Context>();

            // 2. 连接网络设备
            m_device = m_context->createNetDevice(m_config.cam_ip.c_str(), m_config.cam_port);
            if (!m_device) {
                std::cerr << "Failed to connect to camera: " << m_config.cam_ip << std::endl;
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            std::cout << "Connected to camera: " << m_config.cam_ip << std::endl;

            // 3. 创建 Pipeline (绑定到指定设备)
            m_pipeline = std::make_shared<ob::Pipeline>(m_device);

            // 4. 配置彩色流
            m_configPtr = std::make_shared<ob::Config>();

            // 获取彩色流配置列表
            auto colorProfiles = m_pipeline->getStreamProfileList(OB_SENSOR_COLOR);
            if (colorProfiles) {
                auto colorProfile = colorProfiles->getVideoStreamProfile(
                    m_config.cam_width, m_config.cam_height, OB_FORMAT_MJPG, m_config.fps);
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
                    auto depthProfile = depthProfiles->getVideoStreamProfile(
                        m_config.cam_width, m_config.cam_height, OB_FORMAT_Y16, m_config.fps);

                    if (depthProfile)
                        m_configPtr->enableStream(depthProfile);
                }
            }

            // Create a format converter filter.
            m_formatConverter = std::make_shared<ob::FormatConvertFilter>();

            // 在 m_pipeline->start(m_configPtr) 之前添加
            m_configPtr->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
            m_pipeline->start(m_configPtr);


            // Drop several frames
            for (int i = 0; i < 15; ++i) {
                auto lost = m_pipeline->waitForFrameset(100);
            }


            // 7. 验证启动：尝试等待一帧数据
            frameSet = m_pipeline->waitForFrameset(verifyTimeoutMs);
            if (frameSet != nullptr) {
                break;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                close();
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }

            std::cout << "waitForFrameset: " << i << std::endl;
        }

        if (frameSet == nullptr) {
            std::cerr << "Pipeline started but failed to capture the first frame within "
                << verifyTimeoutMs << " ms" << std::endl;
            // 此时可以根据情况选择重试或关闭清理
            close();
        }
        else
        {
            m_isOpened = true;
            std::cout << "Camera started: " << m_config.cam_width << "x" << m_config.cam_height
                << " @ " << m_config.fps << "fps" << std::endl;

        }

        return m_isOpened;

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
                m_formatConverter->setFormatConvertType(FORMAT_RGB_TO_BGR);
                colorFrame = m_formatConverter->process(colorFrame)->as<ob::ColorFrame>();

                if (colorFrame) {
                    auto videoFrame = colorFrame->as<const ob::VideoFrame>();
                    int width = videoFrame->getWidth();
                    int height = videoFrame->getHeight();
                    void* data = videoFrame->getData();

                    if (data && width > 0 && height > 0) {
                        cv::Mat rgbImage(cv::Size(width, height), CV_8UC3, data);
                        bgrImage = rgbImage.clone();
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
