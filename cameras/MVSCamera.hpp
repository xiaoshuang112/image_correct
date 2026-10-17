#ifndef MVS_CAMERA_HPP
#define MVS_CAMERA_HPP

#include "AICamera.hpp"
#include "MvCameraControl.h"
#include "mvs_device_selector.hpp"

#include <mutex>
#include <vector>

class MVSCamera : public AICamera {
public:
    explicit MVSCamera(const CONFIG& config);
    ~MVSCamera() override;

    bool open() override;
    void close() override;
    cv::Mat captureOneFrame(int timeoutMs = 1000) override;
    int captureOneFrameWithDepth(cv::Mat& bgrImage, cv::Mat& depthImage, int timeoutMs = 2000) override;
    bool isOpened() const override { return m_isOpened; }
    CameraDeviceAuditInfo getDeviceAuditInfo() const override;
    std::string backendName() const override { return "mvs"; }
    CameraErrorDetail lastError() const override;

    bool setColorExposureTime(int exposureUs) override;
    int getColorExposureTime() override;
    bool setAutoExposure(bool enable) override;
    bool getAutoExposure() override;
    bool getExposureRange(int& minUs, int& maxUs) override;

    bool setColorGain(int gain) override;
    int getColorGain() override;
    bool getGainRange(int& min, int& max) override;

    std::vector<cv::Mat> captureMultiExposureImages(const std::vector<int>& exposureValuesUs) override;

    // MVS SDK 不提供镜头内参，返回空是正确行为；
    // 内参安全门禁要求从外部 camera_params.yaml 加载并校验身份。
    std::vector<std::vector<float>> getCameraIntrinsics(const int imgWidth, const int imgHight) override;

private:
    // ---- 生命周期阶段（用于幂等清理与诊断） ----
    bool m_sdkInitialized = false;
    bool m_handleCreated = false;
    bool m_deviceOpened = false;
    bool m_isGrabbing = false;
    bool m_isOpened = false;
    void* m_handle = nullptr;

    // open/close/capture/参数设置互斥；多曝光序列会嵌套调用 captureOneFrame，
    // 因此使用递归互斥。
    mutable std::recursive_mutex m_mutex;

    // ---- 取帧缓冲区（按设备 PayloadSize 分配，可安全扩容） ----
    std::vector<unsigned char> m_buffer;

    // ---- 设备身份与实际参数（打开时回读） ----
    mvsdiag::MvsTransport m_transport = mvsdiag::MvsTransport::kOther;
    unsigned int m_deviceIp = 0;
    std::string m_deviceSerial;
    std::string m_deviceModel;
    std::string m_deviceFirmware;
    std::string m_devicePixelFormat;   // 设备端原始像素格式符号（如 BayerRG8）
    std::string m_sdkVersion;
    int m_width = 0;                   // 回读的实际流分辨率
    int m_height = 0;
    int m_exposureUs = -1;
    float m_gain = -1.0f;
    bool m_triggerOn = false;

    // ---- 错误与帧统计 ----
    CameraErrorDetail m_lastError;
    uint64_t m_frameOkCount = 0;
    uint64_t m_frameFailCount = 0;
    uint64_t m_consecutiveFailures = 0;

    void setError(const char* stage, unsigned int sdkCode, const std::string& extra = "");
    void clearError();
    void releaseResourcesLocked();
    bool configureDeviceLocked();
    bool readbackActualParamsLocked();
    bool ensureBufferCapacityLocked(size_t required);
    bool setExposureLocked(int exposureUs);
    bool setAutoExposureLocked(bool enable);
    void discardFramesLocked(int count, int timeoutMs);
};

#endif // MVS_CAMERA_HPP
