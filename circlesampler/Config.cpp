#include "Config.h"
#include <iostream>
#include <cmath>

bool loadScrewSamplingConfig(const std::string& path, ScrewSamplingConfig& cfg) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ScrewConfig] Failed to open: " << path << std::endl;
        return false;
    }

    cv::FileNode sampling = fs["sampling"];
    if (!sampling.empty()) {
        cv::FileNode pos_node = sampling["position"];
        if (pos_node.type() == cv::FileNode::SEQ) {
            cfg.positions.clear();
            auto it = pos_node.begin();
            while (it != pos_node.end()) {
                double x = (double)*it; ++it;
                if (it == pos_node.end()) break;
                double y = (double)*it; ++it;
                if (it == pos_node.end()) break;
                double z = (double)*it; ++it;
                cfg.positions.push_back(cv::Point3d(x, y, z));
            }
        }

        cv::FileNode radii_node = sampling["radii"];
        if (radii_node.type() == cv::FileNode::SEQ) {
            cfg.radii.clear();
            for (auto it = radii_node.begin(); it != radii_node.end(); ++it)
                cfg.radii.push_back((double)*it);
        }

        sampling["background_type"] >> cfg.background_type;
        sampling["num_exposures"] >> cfg.num_exposures;
        sampling["capture_timeout_ms"] >> cfg.capture_timeout_ms;
        sampling["output_dir"] >> cfg.output_dir;

        cv::FileNode yaw_node = sampling["yaw_angles"];
        if (yaw_node.type() == cv::FileNode::SEQ) {
            cfg.yaw_angles.clear();
            for (auto it = yaw_node.begin(); it != yaw_node.end(); ++it)
                cfg.yaw_angles.push_back((int)*it);
        }
    }

    cv::FileNode tfc_node = fs["T_flange_camera"];
    if (!tfc_node.empty() && tfc_node.isSeq()) {
        cfg.T_flange_camera = cv::Mat(4, 4, CV_64F);
        int idx = 0;
        for (auto it = tfc_node.begin(); it != tfc_node.end() && idx < 16; ++it, ++idx)
            cfg.T_flange_camera.at<double>(idx / 4, idx % 4) = (double)*it;
    }

    return true;
}

// Helper: parse a string sequence node into vector<string>
static bool readStrVec(const cv::FileNode& node, std::vector<std::string>& out) {
    if (node.type() != cv::FileNode::SEQ) return false;
    out.clear();
    for (auto it = node.begin(); it != node.end(); ++it)
        out.push_back((std::string)*it);
    return true;
}

// Helper: parse an int sequence node into vector<int>
static bool readIntVec(const cv::FileNode& node, std::vector<int>& out) {
    if (node.type() != cv::FileNode::SEQ) return false;
    out.clear();
    for (auto it = node.begin(); it != node.end(); ++it)
        out.push_back((int)*it);
    return true;
}

bool loadCircleTestConfig(const std::string& path, CircleTestConfig& cfg) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[CircleTestConfig] Failed to open: " << path << std::endl;
        return false;
    }

    cv::FileNode test = fs["circle_test"];
    if (test.empty()) {
        std::cerr << "[CircleTestConfig] Missing 'circle_test' section in " << path << std::endl;
        return false;
    }

    // String vectors
    cv::FileNode bg_node = test["backgrounds"];
    if (!readStrVec(bg_node, cfg.backgrounds) || cfg.backgrounds.empty()) {
        std::cerr << "[CircleTestConfig] 'backgrounds' must be a non-empty string list" << std::endl;
        return false;
    }

    cv::FileNode side_node = test["sides"];
    if (!readStrVec(side_node, cfg.sides) || cfg.sides.empty()) {
        std::cerr << "[CircleTestConfig] 'sides' must be a non-empty string list" << std::endl;
        return false;
    }

    cv::FileNode thd_node = test["thread_types"];
    if (!readStrVec(thd_node, cfg.thread_types) || cfg.thread_types.empty()) {
        std::cerr << "[CircleTestConfig] 'thread_types' must be a non-empty string list" << std::endl;
        return false;
    }

    // Int vectors
    cv::FileNode lay_node = test["layers"];
    if (!readIntVec(lay_node, cfg.layers) || cfg.layers.empty()) {
        std::cerr << "[CircleTestConfig] 'layers' must be a non-empty int list (e.g. [40,45,50])" << std::endl;
        return false;
    }

    cv::FileNode exp_node = test["exposure_indices"];
    if (!readIntVec(exp_node, cfg.exposure_indices) || cfg.exposure_indices.empty()) {
        std::cerr << "[CircleTestConfig] 'exposure_indices' must be a non-empty int list" << std::endl;
        return false;
    }

    // Scalar values
    test["points_per_layer"] >> cfg.points_per_layer;
    if (cfg.points_per_layer < 1) cfg.points_per_layer = 18;

    test["start_angle"] >> cfg.start_angle;
    test["num_exposures"] >> cfg.num_exposures;
    test["capture_timeout_ms"] >> cfg.capture_timeout_ms;
    test["output_dir"] >> cfg.output_dir;
    test["radius"] >> cfg.radius;

    // Position (single triple)
    cv::FileNode pos_node = test["position"];
    if (pos_node.type() == cv::FileNode::SEQ) {
        auto it = pos_node.begin();
        double x = (double)*it; ++it;
        double y = (double)*it; ++it;
        double z = (double)*it;
        cfg.position = cv::Point3d(x, y, z);
    }

    // T_flange_camera (top-level, optional if also in hand_eye_result.yaml)
    cv::FileNode tfc_node = fs["T_flange_camera"];
    if (!tfc_node.empty() && tfc_node.isSeq()) {
        cfg.T_flange_camera = cv::Mat(4, 4, CV_64F);
        int idx = 0;
        for (auto it = tfc_node.begin(); it != tfc_node.end() && idx < 16; ++it, ++idx)
            cfg.T_flange_camera.at<double>(idx / 4, idx % 4) = (double)*it;
    }

    return true;
}

bool validateCircleTestConfig(const CircleTestConfig& cfg) {
    if (cfg.backgrounds.empty()) {
        std::cerr << "[CircleTestConfig] No backgrounds configured" << std::endl;
        return false;
    }
    if (cfg.sides.empty()) {
        std::cerr << "[CircleTestConfig] No sides configured" << std::endl;
        return false;
    }
    if (cfg.thread_types.empty()) {
        std::cerr << "[CircleTestConfig] No thread_types configured" << std::endl;
        return false;
    }
    if (cfg.layers.empty()) {
        std::cerr << "[CircleTestConfig] No layers configured" << std::endl;
        return false;
    }
    if (cfg.points_per_layer < 4) {
        std::cerr << "[CircleTestConfig] points_per_layer must be >= 4" << std::endl;
        return false;
    }
    if (cfg.exposure_indices.empty()) {
        std::cerr << "[CircleTestConfig] No exposure_indices configured" << std::endl;
        return false;
    }
    if (cfg.radius <= 0) {
        std::cerr << "[CircleTestConfig] radius must be positive" << std::endl;
        return false;
    }
    for (int idx : cfg.exposure_indices) {
        if (idx < 0 || idx >= cfg.num_exposures) {
            std::cerr << "[CircleTestConfig] exposure_index " << idx
                      << " out of range [0, " << cfg.num_exposures - 1 << "]" << std::endl;
            return false;
        }
    }
    if (cfg.T_flange_camera.empty()) {
        std::cerr << "[CircleTestConfig] T_flange_camera is required" << std::endl;
        return false;
    }
    return true;
}
