#ifndef CIRCLESAMPLER_CONFIG_H
#define CIRCLESAMPLER_CONFIG_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

struct ScrewSamplingConfig {
    std::vector<cv::Point3d> positions;
    std::vector<double> radii;
    std::string background_type = "default";
    int num_exposures = 10;
    int capture_timeout_ms = 3000;
    std::vector<int> yaw_angles = {40, 45, 50};
    std::string output_dir = "data";
    cv::Mat T_flange_camera;
};

bool loadScrewSamplingConfig(const std::string& path, ScrewSamplingConfig& cfg);

#endif
