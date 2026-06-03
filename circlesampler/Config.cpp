#include "Config.h"
#include <iostream>

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
