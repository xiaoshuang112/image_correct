#include "DepthTestConfig.h"
#include <iostream>

bool loadDepthTestConfig(const std::string& path, DepthTestConfig& cfg) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[DepthTest] 无法打开配置文件: " << path << std::endl;
        return false;
    }

    cv::FileNode root = fs["depth_test"];
    if (root.empty()) {
        std::cerr << "[DepthTest] 配置文件中缺少 depth_test 节点" << std::endl;
        return false;
    }

    root["captures_per_point"] >> cfg.captures_per_point;
    root["capture_timeout_ms"] >> cfg.capture_timeout_ms;
    root["settle_time_ms"] >> cfg.settle_time_ms;
    root["save_dir"] >> cfg.save_dir;

    // Point A
    root["pointA_name"] >> cfg.pointA.name;
    cv::FileNode pa_pose = root["pointA_flange_pose"];
    for (int i = 0; i < 6 && i < (int)pa_pose.size(); i++)
        pa_pose[i] >> cfg.pointA.flange_pose[i];
    root["pointA_depth_min"] >> cfg.pointA.depth_min;
    root["pointA_depth_max"] >> cfg.pointA.depth_max;

    // Point B
    root["pointB_name"] >> cfg.pointB.name;
    cv::FileNode pb_pose = root["pointB_flange_pose"];
    for (int i = 0; i < 6 && i < (int)pb_pose.size(); i++)
        pb_pose[i] >> cfg.pointB.flange_pose[i];
    root["pointB_depth_min"] >> cfg.pointB.depth_min;
    root["pointB_depth_max"] >> cfg.pointB.depth_max;

    // Detection
    cv::FileNode det = root["detection"];
    if (!det.empty()) {
        det["center_ratio"] >> cfg.center_ratio;
        det["outlier_sigma"] >> cfg.outlier_sigma;
    }

    fs.release();

    // Validate
    if (cfg.captures_per_point < 1 || cfg.pointA.name.empty() || cfg.pointB.name.empty()) {
        std::cerr << "[DepthTest] 配置不完整" << std::endl;
        return false;
    }
    if (cfg.pointA.depth_min >= cfg.pointA.depth_max ||
        cfg.pointB.depth_min >= cfg.pointB.depth_max) {
        std::cerr << "[DepthTest] depth_min 必须小于 depth_max" << std::endl;
        return false;
    }

    return true;
}
