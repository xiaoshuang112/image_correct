#ifndef MVS_CAMERA_HPP
#define MVS_CAMERA_HPP

#include "AICamera.hpp"
#include "MvCameraControl.h"

class MVSCamera : public AICamera {
public:
    explicit MVSCamera(const CONFIG& config);
    ~MVSCamera() override;

    bool open() override;
    void close() override;
    cv::Mat captureOneFrame(int timeoutMs = 1000) override;
    int captureOneFrameWithDepth(cv::Mat& bgrImage, cv::Mat& depthImage, int timeoutMs = 2000) override;
    bool isOpened() const override { return m_isOpened; }

    bool setColorExposureTime(int exposureUs) override;
    int getColorExposureTime() override;
    bool setAutoExposure(bool enable) override;
    bool getAutoExposure() override;
    bool getExposureRange(int& minUs, int& maxUs) override;

    bool setColorGain(int gain) override;
    int getColorGain() override;
    bool getGainRange(int& min, int& max) override;

    std::vector<cv::Mat> captureMultiExposureImages(const std::vector<int>& exposureValuesUs) override;
    cv::Mat captureAndFuseMultiExposure(const std::vector<int>& exposureValuesUs,
                                        float contrastWeight = 1.0f,
                                        float saturationWeight = 1.0f,
                                        float wellExposedWeight = 1.0f) override;
    cv::Mat captureAndFuseMultiExposureFast(const std::vector<int>& exposureValuesUs) override;

    std::vector<std::vector<float>> getCameraIntrinsics(const int imgWidth, const int imgHight) override;

protected:
    std::vector<cv::Mat> captureMultiExposureImagesImpl(const std::vector<int>& exposureValuesUs) override;

private:
    bool m_isOpened = false;
    void* m_handle = nullptr;
    unsigned char* m_pData = nullptr;
    unsigned int m_nDataSize = 0;
    int m_width = 0;
    int m_height = 0;
};

#endif // MVS_CAMERA_HPP
