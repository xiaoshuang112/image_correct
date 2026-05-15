#include "OAKCamera.hpp"
#include <iostream>
#include <chrono>

// 解码 MJPEG 编码的帧数据为 BGR 图像
static cv::Mat decodeMjpegFrame(const std::shared_ptr<dai::ImgFrame>& frame) {
    if (!frame) return cv::Mat();
    const auto& data = frame->getData();
    if (data.empty()) return cv::Mat();
    return cv::imdecode(cv::Mat(1, static_cast<int>(data.size()), CV_8UC1, data.data()), cv::IMREAD_COLOR);
}

OAKCamera::OAKCamera(const CONFIG& config)
    : AICamera(config) {
}

OAKCamera::~OAKCamera() {
    close();
}

bool OAKCamera::open() {
    if (m_isOpened) {
        return true;
    }

    try {
        // 1. 设备发现与连接
        // 优先通过 cam_sn / cam_ip 连接特定设备，否则自动检测
        if (!m_config.cam_sn.empty() || !m_config.cam_ip.empty()) {
            std::string deviceId = m_config.cam_sn.empty() ? m_config.cam_ip : m_config.cam_sn;
            auto [found, deviceInfo] = dai::DeviceBase::getDeviceById(deviceId);
            if (!found) {
                // 尝试作为 IP 或名称查找
                auto allDevices = dai::DeviceBase::getAllAvailableDevices();
                bool matched = false;
                for (const auto& dev : allDevices) {
                    if (dev.name == deviceId || dev.deviceId == deviceId) {
                        deviceInfo = dev;
                        matched = true;
                        break;
                    }
                }
                if (!matched) {
                    std::cerr << "Device not found: " << deviceId << std::endl;
                    return false;
                }
            }
            auto device = std::make_shared<dai::Device>(deviceInfo);
            m_pipeline = std::make_shared<dai::Pipeline>(device);
            std::cout << "Connected to device: " << deviceInfo.name
                      << " (ID: " << deviceInfo.deviceId << ")" << std::endl;
        } else {
            // 自动检测第一个可用设备
            m_pipeline = std::make_shared<dai::Pipeline>();
            auto defaultDevice = m_pipeline->getDefaultDevice();
            auto cameras = defaultDevice->getConnectedCameras();
            if (cameras.empty()) {
                std::cerr << "No cameras found on device" << std::endl;
                return false;
            }
            std::cout << "Auto-detected device with " << cameras.size() << " camera(s)" << std::endl;
        }

        // 2. 创建彩色相机节点
        uint32_t sensorW = static_cast<uint32_t>(m_config.cam_width);
        uint32_t sensorH = static_cast<uint32_t>(m_config.cam_height);
        float sensorFps = static_cast<float>(m_config.fps);
        m_colorCam = m_pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A,
                                                                     std::make_pair(sensorW, sensorH),
                                                                     sensorFps);
        // NV12：VideoEncoder MJPEG 要求的输入格式
        auto* colorOutput = m_colorCam->requestOutput({sensorW, sensorH},
                                                       dai::ImgFrame::Type::NV12,
                                                       dai::ImgResizeMode::CROP,
                                                       sensorFps);

        // MJPEG 编码器：将原始 BGR 流压缩为 MJPEG，减少 USB/PoE 带宽占用
        m_videoEnc = m_pipeline->create<dai::node::VideoEncoder>();
        m_videoEnc->setDefaultProfilePreset(sensorFps, dai::VideoEncoderProperties::Profile::MJPEG);
        m_videoEnc->setQuality(80);
        colorOutput->link(m_videoEnc->input);
        m_colorQueue = m_videoEnc->bitstream.createOutputQueue();

        // 大分辨率下增大帧池，避免丢帧（4056x3040 BGR ≈ 37MB/帧）
        if (sensorW > 1920 || sensorH > 1080) {
            size_t frameBytes = static_cast<size_t>(sensorW) * sensorH * 3;
            size_t poolSize = frameBytes * 4;  // 4 帧的缓冲
            m_colorCam->setIspNumFramesPool(3);
            m_colorCam->setMaxSizePoolIsp(static_cast<int>(poolSize));
            m_colorCam->setOutputsNumFramesPool(3);
            m_colorCam->setOutputsMaxSizePool(static_cast<int>(poolSize));
        }

        // 运行时曝光控制队列
        m_camControlQueue = m_colorCam->inputControl.createInputQueue();

        // 3. 可选：深度流
        if (m_config.enableDepth) {
            auto connectedCameras = m_pipeline->getDefaultDevice()->getConnectedCameras();
            bool hasStereo = false;
            for (const auto& cam : connectedCameras) {
                if (cam == dai::CameraBoardSocket::CAM_B) hasStereo = true;
            }

            if (hasStereo) {
                // 与彩色相机保持相同帧率，确保立体匹配帧同步
                m_monoLeft = m_pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B,
                                                                             std::make_pair(640U, 400U),
                                                                             sensorFps);
                m_monoRight = m_pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C,
                                                                              std::make_pair(640U, 400U),
                                                                              sensorFps);

                // 显式指定输出类型为 GRAY8，确保与 StereoDepth 节点输入匹配
                auto* leftOut = m_monoLeft->requestOutput({640, 400},
                                                           dai::ImgFrame::Type::GRAY8,
                                                           dai::ImgResizeMode::CROP,
                                                           sensorFps);
                auto* rightOut = m_monoRight->requestOutput({640, 400},
                                                             dai::ImgFrame::Type::GRAY8,
                                                             dai::ImgResizeMode::CROP,
                                                             sensorFps);

                m_stereo = m_pipeline->create<dai::node::StereoDepth>();
                leftOut->link(m_stereo->left);
                rightOut->link(m_stereo->right);

                // 参照 v3.5.0 rgbd.cpp 的 StereoDepth 配置
                m_stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);
                m_stereo->setLeftRightCheck(true);
                m_stereo->setExtendedDisparity(false);
                m_stereo->setSubpixel(true);
                m_stereo->setRectifyEdgeFillColor(0);
                m_stereo->enableDistortionCorrection(true);
                m_stereo->initialConfig->setMedianFilter(dai::StereoDepthConfig::MedianFilter::KERNEL_7x7);
                m_stereo->initialConfig->setLeftRightCheckThreshold(10);
                m_stereo->initialConfig->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER);
                m_stereo->initialConfig->postProcessing.thresholdFilter.maxRange = 2000;  // 最远 2m

                m_depthQueue = m_stereo->depth.createOutputQueue();
                m_disparityQueue = m_stereo->disparity.createOutputQueue();
                std::cout << "Depth stream enabled" << std::endl;
            } else {
                std::cout << "No stereo cameras (CAM_B/CAM_C) detected, depth disabled" << std::endl;
            }
        }

        // 4. 配置 BoardConfig（watchdog 等）并启动 Pipeline
#ifdef WATCH_DOG
        {
            auto boardCfg = m_pipeline->getBoardConfig();
            boardCfg.watchdogTimeoutMs = 120000;  // 120 秒
            m_pipeline->setBoardConfig(boardCfg);
        }
#endif

        m_pipeline->start();

        // 5. 验证：获取首帧（彩色）
        bool frameReceived = false;
        int maxRetries = (sensorW > 1920 || sensorH > 1080) ? 30 : 10;
        int retryWaitMs = (sensorW > 1920 || sensorH > 1080) ? 200 : 100;
        for (int retry = 0; retry < maxRetries; retry++) {
            auto frame = m_colorQueue->tryGet<dai::ImgFrame>();
            if (frame && !frame->getData().empty()) {
                frameReceived = true;
                std::cout << "First color frame received successfully" << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(retryWaitMs));
        }

        if (!frameReceived) {
            std::cerr << "Pipeline started but no color frame received" << std::endl;
            close();
            return false;
        }

        // 6. 深度管线预热：等待立体匹配算法收敛，产生有效深度帧
        if (m_depthQueue) {
            std::cout << "Warming up stereo depth pipeline..." << std::endl;
            bool depthReady = false;
            for (int retry = 0; retry < 30; retry++) {
                bool dt = false;
                auto depthFrame = m_depthQueue->get<dai::ImgFrame>(
                    std::chrono::milliseconds(500), dt);
                if (depthFrame && !dt) {
                    cv::Mat depth = depthFrame->getCvFrame();
                    double dmin, dmax;
                    cv::minMaxLoc(depth, &dmin, &dmax);
                    if (dmax > 0) {
                        depthReady = true;
                        std::cout << "Depth pipeline ready, depth range: ["
                                  << dmin << ", " << dmax << "] mm" << std::endl;
                        break;
                    }
                }
            }
            if (!depthReady) {
                std::cerr << "Warning: depth pipeline may not be producing valid depth data" << std::endl;
            }
        }

        m_isOpened = true;
        std::cout << "OAK Camera started: " << m_config.cam_width << "x" << m_config.cam_height
                  << " @ " << m_config.fps << "fps" << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Open failed: " << e.what() << std::endl;
        close();
        return false;
    }
}

void OAKCamera::close() {
    if (!m_isOpened) return;

    try {
        if (m_pipeline) {
            m_pipeline->stop();
            m_pipeline.reset();
        }
        m_colorQueue.reset();
        m_depthQueue.reset();
        m_disparityQueue.reset();
        m_camControlQueue.reset();
        m_colorCam.reset();
        m_monoLeft.reset();
        m_monoRight.reset();
        m_stereo.reset();
        m_videoEnc.reset();
        m_isOpened = false;
        std::cout << "OAK Camera closed" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Close error: " << e.what() << std::endl;
        m_isOpened = false;
    }
}

cv::Mat OAKCamera::captureOneFrame(int timeoutMs) {
    if (!m_isOpened || !m_colorQueue) {
        std::cerr << "Camera not opened" << std::endl;
        return cv::Mat();
    }

    try {
        bool hasTimedout = false;
        auto frame = m_colorQueue->get<dai::ImgFrame>(
            std::chrono::milliseconds(timeoutMs), hasTimedout);
        if (!frame || hasTimedout) {
            return cv::Mat();
        }
        return decodeMjpegFrame(frame);
    }
    catch (const std::exception& e) {
        std::cerr << "Capture error: " << e.what() << std::endl;
        return cv::Mat();
    }
}

int OAKCamera::captureOneFrameWithDepth(cv::Mat& bgrImage, cv::Mat& depthImage, int timeoutMs) {
    if (!m_isOpened || !m_colorQueue) {
        std::cerr << "Camera not opened" << std::endl;
        return -1;
    }

    try {
        // 丢弃少量旧帧，确保拿到最新的同步帧对
        int discardedColor = 0, discardedDepth = 0;
        for (int i = 0; i < 3; i++) {
            bool timedout = false;
            auto cf = m_colorQueue->get<dai::ImgFrame>(std::chrono::milliseconds(100), timedout);
            if (!timedout && cf) discardedColor++;
            if (m_depthQueue) {
                bool dt = false;
                auto df = m_depthQueue->get<dai::ImgFrame>(std::chrono::milliseconds(100), dt);
                if (!dt && df) discardedDepth++;
            }
        }
        std::cout << "Discarded frames: color=" << discardedColor << " depth=" << discardedDepth << std::endl;

        // 获取彩色帧
        bool colorTimedout = false;
        auto colorFrame = m_colorQueue->get<dai::ImgFrame>(
            std::chrono::milliseconds(timeoutMs), colorTimedout);
        if (!colorFrame || colorTimedout) {
            std::cerr << "Color frame timeout" << std::endl;
            return -1;
        }
        bgrImage = decodeMjpegFrame(colorFrame);
        std::cout << "Color frame: " << bgrImage.cols << "x" << bgrImage.rows
                  << " type=" << bgrImage.type() << std::endl;

        // 获取深度帧
        if (m_depthQueue) {
            cv::Mat rawDepth;
            bool gotValidDepth = false;
            for (int attempt = 0; attempt < 10; attempt++) {
                bool depthTimedout = false;
                auto depthFrame = m_depthQueue->get<dai::ImgFrame>(
                    std::chrono::milliseconds(500), depthTimedout);
                if (!depthFrame || depthTimedout) {
                    std::cerr << "Depth attempt " << attempt << ": timeout" << std::endl;
                    continue;
                }
                rawDepth = depthFrame->getCvFrame();
                if (rawDepth.empty()) {
                    std::cerr << "Depth attempt " << attempt << ": getCvFrame() returned empty" << std::endl;
                    continue;
                }
                double dmin, dmax;
                cv::minMaxLoc(rawDepth, &dmin, &dmax);
                std::cout << "Depth attempt " << attempt << ": "
                          << rawDepth.cols << "x" << rawDepth.rows
                          << " type=" << rawDepth.type()
                          << " min=" << dmin << " max=" << dmax << std::endl;
                if (dmax > 0) {
                    gotValidDepth = true;
                    cv::imwrite("debug_depth_raw_before_resize.png", rawDepth);
                    break;
                }
                std::cerr << "Depth frame all zeros, retrying..." << std::endl;
            }

            if (gotValidDepth) {
                depthImage = rawDepth;
            } else {
                std::cerr << "Failed to get valid depth frame after 10 attempts" << std::endl;
                depthImage = cv::Mat::zeros(bgrImage.size(), CV_16UC1);
            }
        } else {
            std::cerr << "No depth queue available" << std::endl;
            depthImage = cv::Mat::zeros(bgrImage.size(), CV_16UC1);
        }

        // 将深度图缩放到与彩色图一致的尺寸（使用 INTER_NEAREST 保持原始深度值）
        if (!depthImage.empty() && depthImage.size() != bgrImage.size()) {
            cv::resize(depthImage, depthImage, bgrImage.size(), 0, 0, cv::INTER_NEAREST);
            std::cout << "Depth resized to: " << depthImage.cols << "x" << depthImage.rows
                      << " type=" << depthImage.type() << std::endl;
        }

        cv::imwrite("debug_depth_raw.png", depthImage);
        return bgrImage.empty() ? -1 : 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Capture with depth error: " << e.what() << std::endl;
        return -1;
    }
}

// ========== 曝光控制 ==========

bool OAKCamera::setColorExposureTime(int exposureUs) {
    if (!m_isOpened || !m_camControlQueue) {
        m_exposureCached = exposureUs;
        return false;
    }

    try {
        if (exposureUs < 1 || exposureUs > 33000) {
            std::cerr << "Exposure value " << exposureUs << " us out of range [1, 33000]" << std::endl;
            return false;
        }

        auto ctrl = std::make_shared<dai::CameraControl>();
        ctrl->setManualExposure(static_cast<uint32_t>(exposureUs), 800);
        m_camControlQueue->send(ctrl);

        m_exposureCached = exposureUs;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Set exposure time error: " << e.what() << std::endl;
        m_exposureCached = exposureUs;
        return false;
    }
}

int OAKCamera::getColorExposureTime() {
    return m_exposureCached > 0 ? m_exposureCached : -1;
}

bool OAKCamera::setAutoExposure(bool enable) {
    if (!m_isOpened || !m_camControlQueue) {
        m_autoExposureCached = enable;
        return false;
    }

    try {
        auto ctrl = std::make_shared<dai::CameraControl>();
        if (enable) {
            ctrl->setAutoExposureEnable();
        } else {
            int us = m_exposureCached > 0 ? m_exposureCached : 10000;
            ctrl->setManualExposure(static_cast<uint32_t>(us), 800);
        }
        m_camControlQueue->send(ctrl);

        m_autoExposureCached = enable;
        std::cout << "Auto exposure " << (enable ? "enabled" : "disabled") << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Set auto exposure error: " << e.what() << std::endl;
        m_autoExposureCached = enable;
        return false;
    }
}

bool OAKCamera::getAutoExposure() {
    return m_autoExposureCached;
}

bool OAKCamera::getExposureRange(int& minUs, int& maxUs) {
    minUs = 1;
    maxUs = 33000;
    return true;
}

bool OAKCamera::setColorGain(int gain) {
    if (!m_isOpened || !m_camControlQueue) {
        return false;
    }
    try {
        int expUs = (m_exposureCached > 0) ? m_exposureCached : 10000;
        auto ctrl = std::make_shared<dai::CameraControl>();
        ctrl->setManualExposure(static_cast<uint32_t>(expUs), static_cast<uint32_t>(gain));
        m_camControlQueue->send(ctrl);
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Set color gain error: " << e.what() << std::endl;
        return false;
    }
}

int OAKCamera::getColorGain() {
    return -1; // OAK SDK does not support reading current gain
}

bool OAKCamera::getGainRange(int& min, int& max) {
    min = 100;
    max = 1600;
    return true;
}

// ========== 多曝光采集与融合 ==========

std::vector<cv::Mat> OAKCamera::captureMultiExposureImagesImpl(const std::vector<int>& exposureValuesUs) {
    return captureMultiExposureImages(exposureValuesUs);
}

std::vector<cv::Mat> OAKCamera::captureMultiExposureImages(const std::vector<int>& exposureValuesUs) {
    std::vector<cv::Mat> images;

    if (!m_isOpened) {
        std::cerr << "Camera not opened" << std::endl;
        return images;
    }

    bool originalAutoExposure = getAutoExposure();
    int originalExposure = getColorExposureTime();

    setAutoExposure(false);

    for (int exposureUs : exposureValuesUs) {
        if (setColorExposureTime(exposureUs)) {
            int waitMs = std::min(std::max(exposureUs / 1000, 50), 200);
            std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));

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
        if (originalExposure > 0) {
            setColorExposureTime(originalExposure);
        }
    }

    std::cout << "Captured " << images.size() << " images for multi-exposure fusion" << std::endl;
    return images;
}

cv::Mat OAKCamera::captureAndFuseMultiExposure(const std::vector<int>& exposureValuesUs,
                                                float contrastWeight,
                                                float saturationWeight,
                                                float wellExposedWeight) {
    std::vector<cv::Mat> images = captureMultiExposureImages(exposureValuesUs);

    if (images.size() < 2) {
        std::cerr << "Need at least 2 images for fusion, got " << images.size() << std::endl;
        return cv::Mat();
    }

    ExposureFusion::Params params;
    params.contrastWeight = contrastWeight;
    params.saturationWeight = saturationWeight;
    params.wellExposedWeight = wellExposedWeight;

    std::cout << "Performing exposure fusion..." << std::endl;
    return ExposureFusion::fuse(images, params);
}

cv::Mat OAKCamera::captureAndFuseMultiExposureFast(const std::vector<int>& exposureValuesUs) {
    std::vector<cv::Mat> images = captureMultiExposureImages(exposureValuesUs);

    if (images.size() < 2) {
        std::cerr << "Need at least 2 images for fusion, got " << images.size() << std::endl;
        return cv::Mat();
    }

    std::cout << "Performing fast exposure fusion..." << std::endl;
    return ExposureFusion::fuseFast(images);
}

// ========== 内参接口 ==========

std::vector<std::vector<float>> OAKCamera::getCameraIntrinsics(const int imgWidth, const int imgHight) {
    if (!m_isOpened || !m_pipeline) {
        std::cerr << "Camera not opened" << std::endl;
        return {};
    }

    try {
        auto device = m_pipeline->getDefaultDevice();
        auto calibration = device->readCalibration();
        std::vector<std::vector<float>> matrix = calibration.getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, imgWidth, imgHight);

        // # Distortion: k1:-0.0321069 k2:0.03709 k3:-0.0132376 k4:0 k5:0 k6:0 p1:-0.000181011 p2:0.000430064
        std::vector<float> distortion = {0,0,0,0,0};

        matrix.push_back(distortion);
        return matrix;
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to get camera intrinsics: " << e.what() << std::endl;
        return {};
    }
}

// ========== OAK 特有功能 ==========

bool OAKCamera::setLaserDotBrightness(int brightness) {
    if (!m_isOpened || !m_pipeline || !m_config.enableDepth) return false;

    try {
        auto device = m_pipeline->getDefaultDevice();
        float intensity = std::max(0.0f, std::min(1.0f, static_cast<float>(brightness) / 255.0f));
        device->setIrLaserDotProjectorIntensity(intensity);
        return true;
    } catch (...) {
        return false;
    }
}
