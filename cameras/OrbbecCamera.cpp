#include "OrbbecCamera.hpp"
#include "utils/exposure_fusion.hpp"
#include "hand_eye_gui/DiagnosticLogger.hpp"
#include <iostream>
#include <thread>
#include <chrono>

OrbbecCameraV2::OrbbecCameraV2(const CONFIG& config)
    : AICamera(config) {
    APP_LOG_INFO("orbbec", QString("instance_created endpoint=%1:%2 serial=%3 requested=%4x%5@%6 depth=%7")
        .arg(QString::fromStdString(m_config.cam_ip)).arg(m_config.cam_port)
        .arg(QString::fromStdString(m_config.cam_sn))
        .arg(m_config.cam_width).arg(m_config.cam_height).arg(m_config.fps)
        .arg(m_config.enableDepth));
}

OrbbecCameraV2::~OrbbecCameraV2() {
    APP_LOG_INFO("orbbec", "instance_destroying");
    OrbbecCameraV2::close();
}

bool OrbbecCameraV2::open() {
    if (m_isOpened) {
        APP_LOG_INFO("orbbec", "open_skipped reason=already_open");
        return true;
    }

    APP_LOG_INFO("orbbec", QString("open_begin requested=%1x%2@%3 serial=%4")
        .arg(m_config.cam_width).arg(m_config.cam_height).arg(m_config.fps)
        .arg(QString::fromStdString(m_config.cam_sn)));
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
                    if (!device) {
                        APP_LOG_WARNING("orbbec", QString("device_entry_invalid index=%1 reason=null_device")
                            .arg(i));
                        continue;
                    }
                    auto info = device->getDeviceInfo();
                    if (!info) {
                        APP_LOG_WARNING("orbbec", QString("device_entry_invalid index=%1 reason=null_info")
                            .arg(i));
                        continue;
                    }
                    std::cout << "  Device " << i << ": "
                              << info->name() << " (SN: " << info->serialNumber() << ")" << std::endl;
                }
                
                // 由于无法直接获取IP，尝试连接所有设备直到找到正确的
                // 或者通过设备序号/序列号来识别
                for (uint32_t i = 0; i < deviceList->deviceCount(); i++) {
                    auto device = deviceList->getDevice(i);
                    if (!device)
                        continue;
                    auto info = device->getDeviceInfo();
                    if (!info)
                        continue;
                    
                    // 如果配置中指定了设备序列号，用序列号匹配
                    // 否则直接使用第一个网络设备
                    if (m_config.cam_sn.empty()) {
                        // 没有指定序列号，使用第一个设备
                        m_device = device;
                        deviceFound = true;
                        std::cout << "Connected to camera (first device): " << info->name()
                                  << ", SN: " << info->serialNumber() << std::endl;
                        APP_LOG_INFO("orbbec", QString("device_selected index=%1 name=%2 serial=%3 selection=first")
                            .arg(i).arg(QString::fromStdString(info->name()))
                            .arg(QString::fromStdString(info->serialNumber())));
                        break;
                    } else if (info->serialNumber() == m_config.cam_sn) {
                        // 通过序列号匹配
                        m_device = device;
                        deviceFound = true;
                        std::cout << "Connected to camera: " << m_config.cam_ip
                                  << ", SN: " << info->serialNumber() << std::endl;
                        APP_LOG_INFO("orbbec", QString("device_selected index=%1 name=%2 serial=%3 selection=serial")
                            .arg(i).arg(QString::fromStdString(info->name()))
                            .arg(QString::fromStdString(info->serialNumber())));
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
            APP_LOG_ERROR("orbbec", QString("device_not_found attempts=10 requested_serial=%1")
                .arg(QString::fromStdString(m_config.cam_sn)));
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
        if (!deviceInfo) {
            APP_LOG_ERROR("orbbec", "open_failed reason=null_device_info");
            std::cerr << "Failed to read camera device information" << std::endl;
            return false;
        }
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
            auto streamProfile = colorProfiles->getProfile(i);
            if (!streamProfile) {
                APP_LOG_WARNING("orbbec", QString("color_profile_invalid index=%1 reason=null_profile")
                    .arg(i));
                continue;
            }
            auto profile = streamProfile->as<ob::VideoStreamProfile>();
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
            auto defaultProfile = colorProfiles->getProfile(OB_PROFILE_DEFAULT);
            if (defaultProfile)
                colorProfile = defaultProfile->as<ob::VideoStreamProfile>();
        }

        if (colorProfile) {
            m_configPtr->enableStream(colorProfile);
            auto videoProfile = colorProfile->as<ob::VideoStreamProfile>();
            if (!videoProfile) {
                APP_LOG_ERROR("orbbec", "open_failed reason=color_profile_cast_failed");
                return false;
            }
            m_videoStreamProfile = videoProfile;
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
            APP_LOG_ERROR("orbbec", "pipeline_validation_failed attempts=5 timeout_ms=3000");
            std::cerr << "Pipeline started but no frame received within timeout" << std::endl;
            close();
            return false;
        }
        
        m_isOpened = true;
        m_captureAttempts = 0;
        m_captureSuccesses = 0;
        m_consecutiveCaptureFailures = 0;
        std::cout << "Camera started: " << m_config.cam_width << "x" << m_config.cam_height
                  << " @ " << m_config.fps << "fps" << std::endl;

        // 诊断：打印曝光相关属性支持情况
        std::cout << "Exposure property support:" << std::endl;
        std::cout << "  OB_PROP_COLOR_EXPOSURE_INT read: "
                  << (m_device->isPropertySupported(OB_PROP_COLOR_EXPOSURE_INT, OB_PERMISSION_READ) ? "YES" : "NO")
                  << ", write: "
                  << (m_device->isPropertySupported(OB_PROP_COLOR_EXPOSURE_INT, OB_PERMISSION_WRITE) ? "YES" : "NO")
                  << std::endl;
        std::cout << "  OB_PROP_COLOR_AUTO_EXPOSURE_BOOL read: "
                  << (m_device->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_READ) ? "YES" : "NO")
                  << ", write: "
                  << (m_device->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE) ? "YES" : "NO")
                  << std::endl;

        APP_LOG_INFO("orbbec", QString("open_complete color=%1x%2@%3 format=%4 depth=%5")
            .arg(m_videoStreamProfile ? m_videoStreamProfile->width() : 0)
            .arg(m_videoStreamProfile ? m_videoStreamProfile->height() : 0)
            .arg(m_videoStreamProfile ? m_videoStreamProfile->fps() : 0)
            .arg(m_videoStreamProfile ? static_cast<int>(m_videoStreamProfile->format()) : -1)
            .arg(m_config.enableDepth));
        return true;
    }
    catch (const ob::Error& e) {
        APP_LOG_ERROR("orbbec", QString("open_exception type=orbbec message=%1 function=%2 args=%3")
            .arg(QString::fromUtf8(e.getMessage()))
            .arg(QString::fromUtf8(e.getFunction()))
            .arg(QString::fromUtf8(e.getArgs())));
        std::cerr << "Open failed: " << e.getMessage() << std::endl;
        close();
        return false;
    }
    catch (const std::exception& e) {
        APP_LOG_ERROR("orbbec", QString("open_exception type=std message=%1")
            .arg(QString::fromUtf8(e.what())));
        std::cerr << "Open failed: " << e.what() << std::endl;
        close();
        return false;
    }
    catch (...) {
        APP_LOG_ERROR("orbbec", "open_exception type=unknown");
        std::cerr << "Open failed with an unknown error" << std::endl;
        close();
        return false;
    }
}

void OrbbecCameraV2::close() {
    const bool wasOpened = m_isOpened;
    APP_LOG_INFO("orbbec", QString("close_begin opened=%1 attempts=%2 successes=%3 consecutive_failures=%4")
        .arg(wasOpened).arg(m_captureAttempts.load()).arg(m_captureSuccesses.load())
        .arg(m_consecutiveCaptureFailures.load()));
    try {
        if (m_pipeline) {
            m_pipeline->stop();
        }
    }
    catch (const ob::Error& e) {
        APP_LOG_ERROR("orbbec", QString("close_exception type=orbbec message=%1")
            .arg(QString::fromUtf8(e.getMessage())));
        std::cerr << "Close error: " << e.getMessage() << std::endl;
    }
    catch (const std::exception& e) {
        APP_LOG_ERROR("orbbec", QString("close_exception type=std message=%1")
            .arg(QString::fromUtf8(e.what())));
        std::cerr << "Close error: " << e.what() << std::endl;
    }
    catch (...) {
        APP_LOG_ERROR("orbbec", "close_exception type=unknown");
        std::cerr << "Close failed with an unknown error" << std::endl;
    }

    // Always release partially-created resources, including open() failures.
    m_pipeline.reset();
    m_formatConverter.reset();
    m_videoStreamProfile.reset();
    m_configPtr.reset();
    m_device.reset();
    m_context.reset();
    m_isOpened = false;
    std::cout << "Camera closed" << std::endl;
    APP_LOG_INFO("orbbec", QString("close_complete was_opened=%1").arg(wasOpened));
}

cv::Mat OrbbecCameraV2::captureOneFrame(int timeoutMs) {
    const uint64_t attempt = ++m_captureAttempts;
    if (!m_isOpened || !m_pipeline) {
        const uint64_t failures = ++m_consecutiveCaptureFailures;
        if (failures == 1 || failures % 100 == 0)
            APP_LOG_ERROR("orbbec", QString("capture_rejected reason=not_open attempt=%1 consecutive_failures=%2 pipeline=%3")
                .arg(attempt).arg(failures)
                .arg(reinterpret_cast<quintptr>(m_pipeline.get()), 0, 16));
        return cv::Mat();
    }

    try {
        // 等待帧集合
        auto frameSet = m_pipeline->waitForFrameset(timeoutMs);
        if (!frameSet) {
            const uint64_t failures = ++m_consecutiveCaptureFailures;
            if (failures == 1 || failures % 100 == 0)
                APP_LOG_WARNING("orbbec", QString("capture_timeout timeout_ms=%1 attempt=%2 consecutive_failures=%3")
                    .arg(timeoutMs).arg(attempt).arg(failures));
            return cv::Mat();
        }

        // 获取彩色帧
        auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
        if (!colorFrame) {
            APP_LOG_WARNING("orbbec", QString("capture_missing_color_frame attempt=%1")
                .arg(attempt));
            std::cerr << "Failed to get color frame" << std::endl;
            return cv::Mat();
        }

        // Convert the color frame to RGB format.
        if (!m_formatConverter) {
            APP_LOG_ERROR("orbbec", QString("capture_converter_unavailable attempt=%1")
                .arg(attempt));
            std::cerr << "Format converter is not available" << std::endl;
            return cv::Mat();
        }

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
                APP_LOG_ERROR("orbbec", QString("capture_unsupported_color_format attempt=%1 format=%2")
                    .arg(attempt).arg(static_cast<int>(colorFrame->format())));
                std::cout << "Color format is not support!" << std::endl;
                return cv::Mat();
            }
            auto converted = m_formatConverter->process(colorFrame);
            if (!converted) {
                APP_LOG_ERROR("orbbec", QString("capture_color_conversion_returned_null attempt=%1 stage=to_rgb format=%2")
                    .arg(attempt).arg(static_cast<int>(colorFrame->format())));
                std::cerr << "Failed to convert color frame to RGB" << std::endl;
                return cv::Mat();
            }
            colorFrame = converted->as<ob::ColorFrame>();
            if (!colorFrame) {
                APP_LOG_ERROR("orbbec", QString("capture_color_cast_failed attempt=%1 stage=to_rgb")
                    .arg(attempt));
                return cv::Mat();
            }
        }
        // Processed the color frames to BGR format, use OpenCV to save to disk.
        m_formatConverter->setFormatConvertType(FORMAT_RGB_TO_BGR);
        auto converted = m_formatConverter->process(colorFrame);
        if (!converted) {
            APP_LOG_ERROR("orbbec", QString("capture_color_conversion_returned_null attempt=%1 stage=rgb_to_bgr")
                .arg(attempt));
            std::cerr << "Failed to convert RGB frame to BGR" << std::endl;
            return cv::Mat();
        }
        colorFrame = converted->as<ob::ColorFrame>();
        if (!colorFrame) {
            APP_LOG_ERROR("orbbec", QString("capture_color_cast_failed attempt=%1 stage=rgb_to_bgr")
                .arg(attempt));
            return cv::Mat();
        }


        // 获取帧数据
        auto videoFrame = colorFrame->as<const ob::VideoFrame>();
        if (!videoFrame) {
            APP_LOG_ERROR("orbbec", QString("capture_video_frame_cast_failed attempt=%1")
                .arg(attempt));
            return cv::Mat();
        }
        int width = videoFrame->getWidth();
        int height = videoFrame->getHeight();
        void* data = videoFrame->getData();

        if (!data || width <= 0 || height <= 0) {
            APP_LOG_ERROR("orbbec", QString("capture_invalid_frame_data attempt=%1 data=%2 size=%3x%4")
                .arg(attempt)
                .arg(reinterpret_cast<quintptr>(data), 0, 16)
                .arg(width).arg(height));
            std::cerr << "Invalid frame data" << std::endl;
            return cv::Mat();
        }

        // 转换为 OpenCV Mat (RGB 格式)
        cv::Mat bgrImage(cv::Size(width, height), CV_8UC3, data);

        // 深拷贝
        cv::Mat result = bgrImage.clone();
        const uint64_t failuresBeforeSuccess = m_consecutiveCaptureFailures.exchange(0);
        const uint64_t success = ++m_captureSuccesses;
        if (success == 1 || success % 500 == 0 || failuresBeforeSuccess >= 10) {
            APP_LOG_DEBUG("orbbec", QString("capture_success success=%1 attempt=%2 recovered_failures=%3 "
                                             "size=%4x%5 timeout_ms=%6")
                .arg(success).arg(attempt).arg(failuresBeforeSuccess)
                .arg(result.cols).arg(result.rows).arg(timeoutMs));
        }
        return result;

    }
    catch (const ob::Error& e) {
        ++m_consecutiveCaptureFailures;
        APP_LOG_ERROR("orbbec", QString("capture_exception type=orbbec attempt=%1 message=%2 function=%3 args=%4")
            .arg(attempt)
            .arg(QString::fromUtf8(e.getMessage()))
            .arg(QString::fromUtf8(e.getFunction()))
            .arg(QString::fromUtf8(e.getArgs())));
        std::cerr << "Capture error: " << e.getMessage() << std::endl;
        return cv::Mat();
    }
    catch (const std::exception& e) {
        ++m_consecutiveCaptureFailures;
        APP_LOG_ERROR("orbbec", QString("capture_exception type=std attempt=%1 message=%2")
            .arg(attempt).arg(QString::fromUtf8(e.what())));
        std::cerr << "Capture error: " << e.what() << std::endl;
        return cv::Mat();
    }
    catch (...) {
        ++m_consecutiveCaptureFailures;
        APP_LOG_ERROR("orbbec", QString("capture_exception type=unknown attempt=%1").arg(attempt));
        std::cerr << "Capture failed with an unknown error" << std::endl;
        return cv::Mat();
    }
}



int OrbbecCameraV2::captureOneFrameWithDepth(cv::Mat& bgrImage, cv::Mat& depthImage, int timeoutMs) {
    bgrImage.release();
    depthImage.release();
    const uint64_t attempt = ++m_captureAttempts;

    if (!m_isOpened || !m_pipeline) {
        APP_LOG_ERROR("orbbec", QString("capture_depth_rejected attempt=%1 reason=camera_not_open pipeline=%2")
            .arg(attempt)
            .arg(reinterpret_cast<quintptr>(m_pipeline.get()), 0, 16));
        std::cerr << "Camera not opened" << std::endl;
        return -1;
    }

    if (!m_formatConverter) {
        APP_LOG_ERROR("orbbec", QString("capture_depth_rejected attempt=%1 reason=converter_unavailable")
            .arg(attempt));
        return -1;
    }

    try {
        std::shared_ptr<ob::Frame> colorFrame = nullptr;
        std::shared_ptr<ob::Frame> depthFrame = nullptr;
        int framesetTimeouts = 0;
        int missingColorFrames = 0;
        int missingDepthFrames = 0;
        for (size_t loop = 0; loop < 5; loop++)
        {
            // 等待帧集合
            auto frameSet = m_pipeline->waitForFrameset(timeoutMs);
            if (!frameSet) {
                ++framesetTimeouts;
                std::cerr << "Wait for frameset timeout" << std::endl;
                continue;
            }

            // 获取彩色帧
            if (nullptr == colorFrame)
            {
                auto candidateColorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                if (candidateColorFrame)
                {
                    // Convert the color frame to RGB format.
                    if (candidateColorFrame->format() != OB_FORMAT_RGB) {
                        if (candidateColorFrame->format() == OB_FORMAT_MJPG) {
                            m_formatConverter->setFormatConvertType(FORMAT_MJPG_TO_RGB);
                        }
                        else if (candidateColorFrame->format() == OB_FORMAT_UYVY) {
                            m_formatConverter->setFormatConvertType(FORMAT_UYVY_TO_RGB);
                        }
                        else if (candidateColorFrame->format() == OB_FORMAT_YUYV) {
                            m_formatConverter->setFormatConvertType(FORMAT_YUYV_TO_RGB);
                        }
                        else {
                            APP_LOG_ERROR("orbbec", QString("capture_depth_unsupported_color_format attempt=%1 loop=%2 format=%3")
                                .arg(attempt).arg(loop)
                                .arg(static_cast<int>(candidateColorFrame->format())));
                            std::cout << "Color format is not support!" << std::endl;
                            return -1;
                        }

                        auto converted = m_formatConverter->process(candidateColorFrame);
                        if (!converted) {
                            APP_LOG_WARNING("orbbec", QString("capture_depth_color_conversion_returned_null attempt=%1 loop=%2 format=%3")
                                .arg(attempt).arg(loop)
                                .arg(static_cast<int>(candidateColorFrame->format())));
                            ++missingColorFrames;
                            continue;
                        }
                        candidateColorFrame = converted->as<ob::ColorFrame>();
                        if (!candidateColorFrame) {
                            APP_LOG_WARNING("orbbec", QString("capture_depth_color_cast_failed attempt=%1 loop=%2")
                                .arg(attempt).arg(loop));
                            ++missingColorFrames;
                            continue;
                        }
                    }

                    auto videoFrame = candidateColorFrame->as<const ob::VideoFrame>();
                    if (!videoFrame) {
                        APP_LOG_WARNING("orbbec", QString("capture_depth_video_cast_failed attempt=%1 loop=%2")
                            .arg(attempt).arg(loop));
                        ++missingColorFrames;
                    } else {
                        int width = videoFrame->getWidth();
                        int height = videoFrame->getHeight();
                        void* data = videoFrame->getData();

                        if (data && width > 0 && height > 0) {
                            cv::Mat rgbImage(cv::Size(width, height), CV_8UC3, data);
                            cv::cvtColor(rgbImage, bgrImage, cv::COLOR_RGB2BGR);
                            colorFrame = candidateColorFrame;
                        } else {
                            APP_LOG_WARNING("orbbec", QString("capture_depth_invalid_color_data attempt=%1 loop=%2 data=%3 size=%4x%5")
                                .arg(attempt).arg(loop)
                                .arg(reinterpret_cast<quintptr>(data), 0, 16)
                                .arg(width).arg(height));
                            ++missingColorFrames;
                        }
                    }
                } else {
                    ++missingColorFrames;
                }
            }

            // 获取深度帧
            if (nullptr == depthFrame)
            {
                depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                if (!depthFrame)
                    ++missingDepthFrames;
            }

            if (nullptr == depthFrame || nullptr == colorFrame)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            else
            {
                std::shared_ptr<ob::DepthFrame> depthData = depthFrame->as<ob::DepthFrame>();
                if (nullptr==depthData)
                {
                    APP_LOG_WARNING("orbbec", QString("capture_depth_frame_cast_failed attempt=%1 loop=%2")
                        .arg(attempt).arg(loop));
                    depthFrame = nullptr;
                    ++missingDepthFrames;
                }
                else
                {
                    const int width = depthData->width();
                    const int height = depthData->height();
                    void* data = depthData->data();
                    if (!data || width <= 0 || height <= 0) {
                        APP_LOG_WARNING("orbbec", QString("capture_depth_invalid_depth_data attempt=%1 loop=%2 data=%3 size=%4x%5")
                            .arg(attempt).arg(loop)
                            .arg(reinterpret_cast<quintptr>(data), 0, 16)
                            .arg(width).arg(height));
                        depthFrame = nullptr;
                        ++missingDepthFrames;
                        continue;
                    }
                    cv::Mat depthMat(height, width, CV_16UC1, data);
                    depthImage = depthMat.clone();
                    APP_LOG_DEBUG("orbbec", QString("capture_depth_success attempt=%1 loop=%2 color=%3x%4 depth=%5x%6 "
                                                     "frameset_timeouts=%7 missing_color=%8 missing_depth=%9")
                        .arg(attempt).arg(loop)
                        .arg(bgrImage.cols).arg(bgrImage.rows)
                        .arg(depthImage.cols).arg(depthImage.rows)
                        .arg(framesetTimeouts).arg(missingColorFrames).arg(missingDepthFrames));
                    break;
                }

            }
        }
        if (bgrImage.empty() || depthImage.empty()) {
            APP_LOG_ERROR("orbbec", QString("capture_depth_failed attempt=%1 loops=5 timeout_ms=%2 "
                                             "frameset_timeouts=%3 missing_color=%4 missing_depth=%5 "
                                             "color_empty=%6 depth_empty=%7")
                .arg(attempt).arg(timeoutMs).arg(framesetTimeouts)
                .arg(missingColorFrames).arg(missingDepthFrames)
                .arg(bgrImage.empty()).arg(depthImage.empty()));
            return -1;
        }
        return 0;
    }
    catch (const ob::Error& e) {
        APP_LOG_ERROR("orbbec", QString("capture_depth_exception type=orbbec attempt=%1 message=%2 function=%3 args=%4")
            .arg(attempt)
            .arg(QString::fromUtf8(e.getMessage()))
            .arg(QString::fromUtf8(e.getFunction()))
            .arg(QString::fromUtf8(e.getArgs())));
        std::cerr << "Capture with depth error: " << e.getMessage() << std::endl;
        return -1;
    }
    catch (const std::exception& e) {
        APP_LOG_ERROR("orbbec", QString("capture_depth_exception type=std attempt=%1 message=%2")
            .arg(attempt).arg(QString::fromUtf8(e.what())));
        std::cerr << "Capture with depth error: " << e.what() << std::endl;
        return -1;
    }
    catch (...) {
        APP_LOG_ERROR("orbbec", QString("capture_depth_exception type=unknown attempt=%1")
            .arg(attempt));
        std::cerr << "Capture with depth error: " << std::endl;
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


    if (nullptr==m_videoStreamProfile)
    {
        matrix[0][0] = 608.212;
        matrix[1][1] = 608.21;

        matrix[0][2] = 648.738;
        matrix[1][2] = 392.18;

        matrix[2][2] = 1;


        // # Distortion: k1:-0.0321069 k2:0.03709 k3:-0.0132376 k4:0 k5:0 k6:0 p1:-0.000181011 p2:0.000430064
        std::vector<float> distortion = {-0.0321069, 0.03709,-0.000181011, 0.000430064, -0.0132376};

        matrix.push_back(distortion);
    }
    else
    {
        auto intrinsics = m_videoStreamProfile->getIntrinsic();
        matrix[0][0] = intrinsics.fx;
        matrix[1][1] = intrinsics.fy;

        matrix[0][2] = intrinsics.cx;
        matrix[1][2] = intrinsics.cy;

        matrix[2][2] = 1;

        auto d = m_videoStreamProfile->getDistortion();  
        
        // # Distortion: k1:-0.0321069 k2:0.03709 k3:-0.0132376 k4:0 k5:0 k6:0 p1:-0.000181011 p2:0.000430064
        std::vector<float> distortion = {d.k1, d.k2,d.p1, d.p2, d.k3};
        matrix.push_back(distortion);
        
    }

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
        // 关闭自动曝光
        if (m_device->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
            m_device->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, false);
        }

        // 检查设备是否支持曝光时间设置
        if (!m_device->isPropertySupported(OB_PROP_COLOR_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
            std::cerr << "Device does not support color exposure setting" << std::endl;
            return false;
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

        // 部分奥比中光固件在 setIntProperty 后会重新打开自动曝光，再次确保关闭
        if (m_device->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
            m_device->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, false);
        }

        m_exposureCached = exposureUs;
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
            return false;
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
        return false;
    }

    try {
        if (!m_device->isPropertySupported(OB_PROP_COLOR_EXPOSURE_INT, OB_PERMISSION_READ)) {
            std::cerr << "Device does not support reading exposure range" << std::endl;
            return false;
        }

        auto range = m_device->getIntPropertyRange(OB_PROP_COLOR_EXPOSURE_INT);
        minUs = range.min;
        maxUs = range.max;
        return true;
    }
    catch (const ob::Error& e) {
        std::cerr << "Get exposure range error: " << e.getMessage() << std::endl;
        return false;
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
