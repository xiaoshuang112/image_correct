#ifndef AI_CAMERA_HPP
#define AI_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include <vector>
#include "my_base.h"

struct CameraDeviceAuditInfo {
    std::string serial;
    std::string model;
    std::string firmware;
    // 以下为生产审计扩展字段；SDK 提供不了的派生类保持默认空值即可。
    std::string transport;      // gige / usb 等
    std::string current_ip;     // 设备当前 IP（无 IP 的设备为空）
    int actual_width = 0;       // 打开后回读的实际流分辨率
    int actual_height = 0;
    std::string pixel_format;   // 输出给上层的像素格式（如 BGR8）
    int exposure_us = -1;       // 打开后回读的实际曝光（微秒）
    float gain = -1.0f;         // 打开后回读的实际增益
    std::string trigger_mode;   // off / on
    std::string sdk_version;    // 相机 SDK 运行时版本
};

// Qt 无关的相机错误详情：Controller 与审计日志通过它获取失败阶段和 SDK 错误码。
struct CameraErrorDetail {
    std::string stage;          // initialize / enumerate / select / create_handle / ...
    unsigned int sdk_code = 0;  // SDK 原始错误码（无 SDK 语义时为 0）
    std::string message;        // 人类可读诊断
};

/**
 * @brief 相机抽象基类
 * 定义所有AI相机的通用接口
 */
class AICamera {
public:
    explicit AICamera(const CONFIG& config) : m_config(config) {}
    virtual ~AICamera() = default;

    // 禁止拷贝
    AICamera(const AICamera&) = delete;
    AICamera& operator=(const AICamera&) = delete;

    // ========== 核心相机操作接口 ==========
    
    /**
     * @brief 打开相机
     * @return true 成功，false 失败
     */
    virtual bool open() = 0;
    
    /**
     * @brief 关闭相机
     */
    virtual void close() = 0;
    
    /**
     * @brief 捕获单帧图像
     * @param timeoutMs 超时时间（毫秒）
     * @return BGR格式的OpenCV Mat，失败返回空Mat
     */
    virtual cv::Mat captureOneFrame(int timeoutMs = 1000) = 0;
    
    /**
     * @brief 捕获彩色图+深度图
     * @param bgrImage 输出BGR彩色图像
     * @param depthFrame 输出深度帧（具体类型由子类定义）
     * @param timeoutMs 超时时间（毫秒）
     * @return 0成功，-1失败
     */
    virtual int captureOneFrameWithDepth(cv::Mat& bgrImage, cv::Mat& depthImage, int timeoutMs = 2000) = 0;
    
    /**
     * @brief 检查相机是否已打开
     */
    virtual bool isOpened() const = 0;
    
    /**
     * @brief 获取相机配置
     */
    const CONFIG& getConfig() const { return m_config; }

    // 设备实际身份。派生类在 SDK 能提供时覆盖；默认仅返回配置中的序列号。
    virtual CameraDeviceAuditInfo getDeviceAuditInfo() const {
        return {m_config.cam_sn, "", ""};
    }

    // 相机后端名，用于内参安全门禁与审计。
    virtual std::string backendName() const { return "unknown"; }

    // 最近一次失败的阶段与 SDK 错误码；无错误时返回默认空值。
    virtual CameraErrorDetail lastError() const { return CameraErrorDetail{}; }

    // ========== 曝光控制接口 ==========
    
    /**
     * @brief 设置彩色相机曝光时间
     * @param exposureUs 曝光时间，单位：微秒(μs)
     * @return true 设置成功，false 设置失败
     */
    virtual bool setColorExposureTime(int exposureUs) = 0;
    
    /**
     * @brief 获取当前彩色相机曝光时间
     * @return 曝光时间(微秒)，失败返回 -1
     */
    virtual int getColorExposureTime() = 0;
    
    /**
     * @brief 设置彩色相机自动曝光开关
     * @param enable true: 开启自动曝光, false: 关闭自动曝光
     * @return true 设置成功，false 设置失败
     */
    virtual bool setAutoExposure(bool enable) = 0;
    
    /**
     * @brief 获取彩色相机自动曝光状态
     * @return true: 自动曝光开启, false: 关闭
     */
    virtual bool getAutoExposure() = 0;
    
    /**
     * @brief 获取曝光时间范围
     * @param minUs 输出最小曝光时间(微秒)
     * @param maxUs 输出最大曝光时间(微秒)
     * @return true 获取成功
     */
    virtual bool getExposureRange(int& minUs, int& maxUs) = 0;

    // ========== 多曝光采集接口 ==========
    
    /**
     * @brief 采集多组不同曝光的原始图像
     * @param exposureValuesUs 曝光时间列表（微秒）
     * @return 采集到的图像序列
     */
    virtual std::vector<cv::Mat> captureMultiExposureImages(const std::vector<int>& exposureValuesUs) = 0;

protected:
    CONFIG m_config;
};

#endif // AI_CAMERA_HPP
