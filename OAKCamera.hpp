#ifndef OAK_CAMERA_HPP
#define OAK_CAMERA_HPP

#define DEPTHAI_HAVE_OPENCV_SUPPORT

#include "AICamera.hpp"
#include "exposure_fusion.hpp"
#include <depthai/depthai.hpp>
#include <memory>
#include <thread>

/**
 * @brief OAK深度相机封装类（适配 DepthAI SDK v3.5.0）
 * 支持USB和PoE连接的OAK相机（OAK-D, OAK-D-Lite, OAK-1等）
 */
class OAKCamera : public AICamera {
public:
    explicit OAKCamera(const CONFIG& config);
    ~OAKCamera() override;

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

    // 增益控制接口实现
    bool setColorGain(int gain) override;
    int getColorGain() override;
    bool getGainRange(int& min, int& max) override;

    // 多曝光融合接口实现
    std::vector<cv::Mat> captureMultiExposureImages(const std::vector<int>& exposureValuesUs) override;
    cv::Mat captureAndFuseMultiExposure(const std::vector<int>& exposureValuesUs,
                                        float contrastWeight = 1.0f,
                                        float saturationWeight = 1.0f,
                                        float wellExposedWeight = 1.0f) override;
    cv::Mat captureAndFuseMultiExposureFast(const std::vector<int>& exposureValuesUs) override;

    // 内参接口
    std::vector<std::vector<float>> getCameraIntrinsics(const int imgWidth, const int imgHight) override;

    // OAK特有功能
    bool setLaserDotBrightness(int brightness);

protected:
    std::vector<cv::Mat> captureMultiExposureImagesImpl(const std::vector<int>& exposureValuesUs) override;

private:
    bool m_isOpened = false;

    // v3.5.0 Pipeline（管理 Device 生命周期）
    std::shared_ptr<dai::Pipeline> m_pipeline;

    // 相机节点（统一使用 Camera 节点）
    std::shared_ptr<dai::node::Camera> m_colorCam;
    std::shared_ptr<dai::node::Camera> m_monoLeft;
    std::shared_ptr<dai::node::Camera> m_monoRight;
    std::shared_ptr<dai::node::StereoDepth> m_stereo;

    // MJPEG 编码器（减少带宽占用）
    std::shared_ptr<dai::node::VideoEncoder> m_videoEnc;

    // 输出队列
    std::shared_ptr<dai::MessageQueue> m_colorQueue;
    std::shared_ptr<dai::MessageQueue> m_depthQueue;
    std::shared_ptr<dai::MessageQueue> m_disparityQueue;  // 调试用

    // 运行时曝光控制（通过 InputQueue 发送 CameraControl）
    std::shared_ptr<dai::InputQueue> m_camControlQueue;
};

#endif // OAK_CAMERA_HPP
