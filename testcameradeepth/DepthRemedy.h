#ifndef DEPTH_REMEDY_HPP
#define DEPTH_REMEDY_HPP

#include <vector>
#include <string>
#include <memory>
#include "cameras/AICamera.hpp"
#include "DepthTestConfig.h"

// 点位深度历史管理 + 陈旧帧检测
struct DepthHistory {
    std::vector<double> valid_depths;
    double default_depth = 0.0;
    int stale_count = 0;
    int total_count = 0;

    // 将新深度加入历史，自动剔除离群点后更新 default_depth
    void addValidDepth(double depth_mm, double outlier_sigma);

    double getDefaultDepth() const { return default_depth; }

    // 检测是否为陈旧帧
    bool isStaleFrame(double depth_mm,
                      double depth_min, double depth_max,
                      double other_depth_min, double other_depth_max) const;
};

// 补救执行：检测到陈旧帧后的重采 + 默认深度回退
// camera        — 相机（用于重采）
// history       — 该点位的深度历史
// point         — 当前点位配置
// other_point   — 另一个点位配置
// config        — 测试配置（settle_time_ms, max_reacquire 等）
// out_reacquire_count — 输出实际重采次数
// 返回补救后的深度值 (mm)
double remedyDepth(std::shared_ptr<AICamera>& camera,
                   DepthHistory& history,
                   const DepthTestPoint& point,
                   const DepthTestPoint& other_point,
                   const DepthTestConfig& config,
                   int& out_reacquire_count);

// 从深度图中心区域提取深度中位数 (mm)，0=无有效深度
double extractCenterDepth(const cv::Mat& depth, double center_ratio);

#endif // DEPTH_REMEDY_HPP
