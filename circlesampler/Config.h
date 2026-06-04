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

struct CircleTestConfig {
    std::vector<std::string> backgrounds;
    std::vector<std::string> sides;
    std::vector<std::string> thread_types;
    std::vector<int> layers;
    int points_per_layer = 18;
    int start_angle = 0;
    std::vector<int> exposure_indices;
    cv::Point3d position;
    double radius = 140.0;
    int num_exposures = 10;
    int capture_timeout_ms = 3000;
    std::string output_dir = "data/test";
    cv::Mat T_flange_camera;
};

bool loadScrewSamplingConfig(const std::string& path, ScrewSamplingConfig& cfg);
bool loadCircleTestConfig(const std::string& path, CircleTestConfig& cfg);
bool validateCircleTestConfig(const CircleTestConfig& cfg);

#endif
