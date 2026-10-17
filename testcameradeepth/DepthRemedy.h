#ifndef DEPTH_REMEDY_HPP
#define DEPTH_REMEDY_HPP

#include <vector>
#include <opencv2/core/mat.hpp>

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

// 从深度图中心区域提取深度中位数 (mm)，0=无有效深度
double extractCenterDepth(const cv::Mat& depth, double center_ratio);

#endif // DEPTH_REMEDY_HPP
