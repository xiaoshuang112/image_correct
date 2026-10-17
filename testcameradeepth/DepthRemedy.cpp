#include "DepthRemedy.h"
#include <algorithm>
#include <cmath>
#include <numeric>

void DepthHistory::addValidDepth(double depth_mm, double outlier_sigma) {
    valid_depths.push_back(depth_mm);
    total_count++;

    if ((int)valid_depths.size() < 3) {
        default_depth = depth_mm;
        return;
    }

    // 计算均值和标准差
    double sum = std::accumulate(valid_depths.begin(), valid_depths.end(), 0.0);
    double mean = sum / valid_depths.size();
    double sq_sum = 0.0;
    for (double v : valid_depths)
        sq_sum += (v - mean) * (v - mean);
    double stddev = std::sqrt(sq_sum / valid_depths.size());

    // 剔除离群点
    std::vector<double> filtered;
    for (double v : valid_depths) {
        if (std::fabs(v - mean) <= outlier_sigma * stddev)
            filtered.push_back(v);
    }

    if (!filtered.empty()) {
        double fsum = std::accumulate(filtered.begin(), filtered.end(), 0.0);
        default_depth = fsum / filtered.size();
    }
}

bool DepthHistory::isStaleFrame(double depth_mm,
                                 double depth_min, double depth_max,
                                 double other_depth_min, double other_depth_max) const {
    // 无历史数据 → 正常（初始学习阶段）
    if (valid_depths.empty())
        return false;

    bool in_cur = (depth_mm >= depth_min && depth_mm <= depth_max);
    bool in_other = (depth_mm >= other_depth_min && depth_mm <= other_depth_max);

    if (in_cur && !in_other) return false;   // 明确在当前范围 → 正常
    if (!in_cur && in_other) return true;     // 明确在对方范围 → 陈旧
    if (in_cur && in_other) return true;      // 范围重叠 → 保守判陈旧

    // 都不在范围 → 检查偏离 default_depth 的程度
    double diff = std::fabs(depth_mm - default_depth);
    double other_mid = (other_depth_min + other_depth_max) / 2.0;
    double diff_to_other = std::fabs(depth_mm - other_mid);
    return diff_to_other < diff;
}

double extractCenterDepth(const cv::Mat& depth, double center_ratio) {
    int roi_w = (int)(depth.cols * center_ratio);
    int roi_h = (int)(depth.rows * center_ratio);
    if (roi_w < 1 || roi_h < 1) return 0;

    int x = (depth.cols - roi_w) / 2;
    int y = (depth.rows - roi_h) / 2;
    cv::Mat roi = depth(cv::Rect(x, y, roi_w, roi_h));

    std::vector<ushort> values;
    for (int r = 0; r < roi.rows; r++) {
        const ushort* row = roi.ptr<ushort>(r);
        for (int c = 0; c < roi.cols; c++) {
            if (row[c] > 0)
                values.push_back(row[c]);
        }
    }

    if (values.empty()) return 0;

    size_t n = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + n, values.end());
    return static_cast<double>(values[n]);
}
