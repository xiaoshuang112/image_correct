#ifndef ORBBEC_CAMERA_V2_HPP
#define ORBBEC_CAMERA_V2_HPP

#include "AICamera.hpp"
#include "exposure_fusion.hpp"

// 禁用 MSVC 安全警告
#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

// SDK v2 核心头文件
#include "libobsensor/ObSensor.hpp"

class OrbbecCameraV2 : public AICamera {
public:
    explicit OrbbecCameraV2(const CONFIG& config);
    ~OrbbecCameraV2() override;

    // 核心相机操作接口实现
    bool open() override;
    void close() override;
    cv::Mat captureOneFrame(int timeoutMs = 1000) override;
    int captureOneFrameWithDepth(cv::Mat& bgrImage, cv::Mat& depthImage, int timeoutMs = 2000) override;
    bool isOpened() const override { return m_isOpened; }

    // 曝光控制接口实现
    bool setColorExposureTime(int exposureUs) override;
    int getColorExposureTime() override;
    bool setAutoExposure(bool enable) override;
    bool getAutoExposure() override;
    bool getExposureRange(int& minUs, int& maxUs) override;

    // 多曝光融合接口实现
    std::vector<cv::Mat> captureMultiExposureImages(const std::vector<int>& exposureValuesUs) override;
    cv::Mat captureAndFuseMultiExposure(const std::vector<int>& exposureValuesUs,
                                        float contrastWeight = 1.0f,
                                        float saturationWeight = 1.0f,
                                        float wellExposedWeight = 1.0f) override;
    cv::Mat captureAndFuseMultiExposureFast(const std::vector<int>& exposureValuesUs) override;

    // 内参接口
    std::vector<std::vector<float>> getCameraIntrinsics(const int imgWidth, const int imgHight) override;


protected:
    std::vector<cv::Mat> captureMultiExposureImagesImpl(const std::vector<int>& exposureValuesUs) override;

private:
    bool m_isOpened = false;

    std::shared_ptr<ob::Context> m_context;
    std::shared_ptr<ob::Device> m_device;
    std::shared_ptr<ob::Pipeline> m_pipeline;
    std::shared_ptr<ob::Config> m_configPtr;
    std::shared_ptr<ob::FormatConvertFilter> m_formatConverter;
};

#endif // ORBBEC_CAMERA_V2_HPP