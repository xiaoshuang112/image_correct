#ifndef AI_CAMERA_HPP
#define AI_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include <vector>
#include "my_base.h"

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

    // ========== 增益（曝光度）控制接口 ==========

    /**
     * @brief 设置彩色相机增益
     * @param gain 增益值（单位由设备定义）
     * @return true 设置成功
     */
    virtual bool setColorGain(int gain) = 0;

    /**
     * @brief 获取当前彩色相机增益
     * @return 增益值，失败返回 -1
     */
    virtual int getColorGain() = 0;

    /**
     * @brief 获取增益范围
     * @param min 输出最小增益
     * @param max 输出最大增益
     * @return true 获取成功
     */
    virtual bool getGainRange(int& min, int& max) = 0;

    // ========== 多曝光融合接口 ==========
    
    /**
     * @brief 采集多组不同曝光的原始图像
     * @param exposureValuesUs 曝光时间列表（微秒）
     * @return 采集到的图像序列
     */
    virtual std::vector<cv::Mat> captureMultiExposureImages(const std::vector<int>& exposureValuesUs) = 0;

    /**
     * @brief 采集多组不同曝光的图像并融合（Mertens算法）
     * @param exposureValuesUs 曝光时间列表（微秒）
     * @param contrastWeight 对比度权重系数 (默认1.0)
     * @param saturationWeight 饱和度权重系数 (默认1.0)
     * @param wellExposedWeight 良好曝光权重系数 (默认1.0)
     * @return 融合后的BGR图像
     */
    virtual cv::Mat captureAndFuseMultiExposure(const std::vector<int>& exposureValuesUs,
                                                float contrastWeight = 1.0f,
                                                float saturationWeight = 1.0f,
                                                float wellExposedWeight = 1.0f) = 0;

    /**
     * @brief 快速多曝光融合（速度优化版本）
     * @param exposureValuesUs 曝光时间列表（微秒）
     * @return 融合后的BGR图像
     */
    virtual cv::Mat captureAndFuseMultiExposureFast(const std::vector<int>& exposureValuesUs) = 0;

    // 获取内参矩阵 (3x3矩阵)
    // 如果想得到特定分辨率下的内参，可以传入宽和高，例如 (1280, 720)
    virtual std::vector<std::vector<float>> getCameraIntrinsics(const int imgWidth, const int imgHight)=0;

protected:
    CONFIG m_config;
    
    // 缓存值（供子类使用）
    bool m_autoExposureCached = true;
    int m_exposureCached = -1;
    
    // 通用的多曝光采集逻辑
    virtual std::vector<cv::Mat> captureMultiExposureImagesImpl(const std::vector<int>& exposureValuesUs) = 0;


};

#endif // AI_CAMERA_HPP
