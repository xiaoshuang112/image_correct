#include "Config.h"
#include <iostream>
#include <cmath>

bool loadPieceSamplerConfig(const std::string& path, PieceSamplerConfig& cfg) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[PieceConfig] Failed to open: " << path << std::endl;
        return false;
    }

    cv::FileNode root = fs["piece_sampler"];
    if (root.empty()) {
        std::cerr << "[PieceConfig] Missing 'piece_sampler' section in " << path << std::endl;
        return false;
    }

    // Position (single triple)
    cv::FileNode pos_node = root["position"];
    if (pos_node.type() == cv::FileNode::SEQ) {
        auto it = pos_node.begin();
        double x = (double)*it; ++it;
        double y = (double)*it; ++it;
        double z = (double)*it;
        cfg.position = cv::Point3d(x, y, z);
    }

    // XY offsets
    cv::FileNode xy_node = root["xy_offsets"];
    if (xy_node.type() == cv::FileNode::SEQ) {
        cfg.xy_offsets.clear();
        for (auto it = xy_node.begin(); it != xy_node.end(); ++it)
            cfg.xy_offsets.push_back((double)*it);
    }

    // Z heights
    cv::FileNode zh_node = root["z_heights"];
    if (zh_node.type() == cv::FileNode::SEQ) {
        cfg.z_heights.clear();
        for (auto it = zh_node.begin(); it != zh_node.end(); ++it)
            cfg.z_heights.push_back((double)*it);
    }

    // Backgrounds (string list)
    cv::FileNode bg_node = root["backgrounds"];
    if (bg_node.type() == cv::FileNode::SEQ) {
        cfg.backgrounds.clear();
        for (auto it = bg_node.begin(); it != bg_node.end(); ++it)
            cfg.backgrounds.push_back((std::string)*it);
    }

    // Workpiece configurations (string list)
    cv::FileNode wp_node = root["workpiece_configs"];
    if (wp_node.type() == cv::FileNode::SEQ) {
        cfg.workpiece_configs.clear();
        for (auto it = wp_node.begin(); it != wp_node.end(); ++it)
            cfg.workpiece_configs.push_back((std::string)*it);
    }

    // Exposure indices (int list)
    cv::FileNode exp_node = root["exposure_indices"];
    if (exp_node.type() == cv::FileNode::SEQ) {
        cfg.exposure_indices.clear();
        for (auto it = exp_node.begin(); it != exp_node.end(); ++it)
            cfg.exposure_indices.push_back((int)*it);
    }

    // Scalar values
    root["num_exposures"] >> cfg.num_exposures;
    root["capture_timeout_ms"] >> cfg.capture_timeout_ms;
    root["output_dir"] >> cfg.output_dir;

    // T_flange_camera (top-level, optional)
    cv::FileNode tfc_node = fs["T_flange_camera"];
    if (!tfc_node.empty() && tfc_node.isSeq()) {
        cfg.T_flange_camera = cv::Mat(4, 4, CV_64F);
        int idx = 0;
        for (auto it = tfc_node.begin(); it != tfc_node.end() && idx < 16; ++it, ++idx)
            cfg.T_flange_camera.at<double>(idx / 4, idx % 4) = (double)*it;
    }

    return true;
}

bool validatePieceSamplerConfig(const PieceSamplerConfig& cfg) {
    if (cfg.xy_offsets.size() < 1) {
        std::cerr << "[PieceConfig] xy_offsets must have at least 1 value" << std::endl;
        return false;
    }
    if (cfg.z_heights.empty()) {
        std::cerr << "[PieceConfig] z_heights must not be empty" << std::endl;
        return false;
    }
    if (cfg.backgrounds.empty()) {
        std::cerr << "[PieceConfig] backgrounds must not be empty" << std::endl;
        return false;
    }
    if (cfg.workpiece_configs.empty()) {
        std::cerr << "[PieceConfig] workpiece_configs must not be empty" << std::endl;
        return false;
    }
    if (cfg.exposure_indices.empty()) {
        std::cerr << "[PieceConfig] exposure_indices must not be empty" << std::endl;
        return false;
    }
    if (cfg.num_exposures < 1) {
        std::cerr << "[PieceConfig] num_exposures must be >= 1" << std::endl;
        return false;
    }
    for (int idx : cfg.exposure_indices) {
        if (idx < 0 || idx >= cfg.num_exposures) {
            std::cerr << "[PieceConfig] exposure_index " << idx
                      << " out of range [0, " << cfg.num_exposures - 1 << "]" << std::endl;
            return false;
        }
    }
    if (cfg.T_flange_camera.empty()) {
        std::cerr << "[PieceConfig] T_flange_camera is required" << std::endl;
        return false;
    }
    return true;
}
