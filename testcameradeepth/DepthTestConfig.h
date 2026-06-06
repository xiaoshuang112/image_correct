#ifndef DEPTH_TEST_CONFIG_HPP
#define DEPTH_TEST_CONFIG_HPP

#include <opencv2/opencv.hpp>
#include <string>

struct DepthTestPoint {
    std::string name;
    double flange_pose[6] = {};  // [x,y,z,rx,ry,rz] (mm, deg)
    double depth_min = 0;        // mm
    double depth_max = 0;        // mm
    double default_depth = 0;    // 0 = learn from capture
};

struct DepthTestConfig {
    int captures_per_point = 20000;
    int capture_timeout_ms = 3000;
    int settle_time_ms = 500;
    std::string save_dir = "data/depth_test";
    DepthTestPoint pointA;
    DepthTestPoint pointB;
    double center_ratio = 0.15;
    double stale_threshold_mm = 80;
    double outlier_sigma = 2.0;
    int max_reacquire = 3;
    int history_size = 50;
};

bool loadDepthTestConfig(const std::string& path, DepthTestConfig& cfg);

#endif // DEPTH_TEST_CONFIG_HPP
