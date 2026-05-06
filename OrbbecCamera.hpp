#ifndef ORBBEC_CAMERA_V2_HPP
#define ORBBEC_CAMERA_V2_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include "my_base.h"

// 禁用 MSVC 安全警告
#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

// SDK v2 核心头文件
#include "libobsensor/ObSensor.hpp"


class OrbbecCameraV2 {
public:
    explicit OrbbecCameraV2(const CONFIG& config);
    ~OrbbecCameraV2();

    OrbbecCameraV2(const OrbbecCameraV2&) = delete;
    OrbbecCameraV2& operator=(const OrbbecCameraV2&) = delete;

    bool open();
    void close();
    cv::Mat captureOneFrame(int timeoutMs = 1000);
    int captureOneFrameWithDepth(cv::Mat& bgrImage, cv::Mat& depthImage, int timeoutMs = 2000);
    bool isOpened() const { return m_isOpened; }
    const CONFIG& getConfig() const { return m_config; }

private:
    CONFIG m_config;
    bool m_isOpened = false;

    std::shared_ptr<ob::Context> m_context;
    std::shared_ptr<ob::Device> m_device;
    std::shared_ptr<ob::Pipeline> m_pipeline;
    std::shared_ptr<ob::Config> m_configPtr;
    std::shared_ptr<ob::FormatConvertFilter> m_formatConverter;
};

#endif