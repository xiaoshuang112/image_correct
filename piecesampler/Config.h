#ifndef PIECESAMPLER_CONFIG_H
#define PIECESAMPLER_CONFIG_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

struct PieceSamplerConfig {
    // Workpiece center position in robot base frame (mm)
    cv::Point3d position;

    // XY grid offsets from center (mm), e.g. [-500, -300, 0, 300, 500]
    std::vector<double> xy_offsets;

    // Z heights above workpiece (mm), e.g. [300, 500, 700]
    std::vector<double> z_heights;

    // Background names (prompted during collection)
    std::vector<std::string> backgrounds;

    // Workpiece arrangement descriptions (e.g. "1piece_A", "2pieces_B")
    std::vector<std::string> workpiece_configs;

    // Exposure indices to use from the log-spaced list, e.g. [4,5,6,7,8]
    std::vector<int> exposure_indices;
    int num_exposures = 10;       // total exposures in log-spaced list
    int capture_timeout_ms = 3000;

    std::string output_dir = "data/piece_sampling";
    cv::Mat T_flange_camera;      // 4x4 transform: flange -> camera
};

bool loadPieceSamplerConfig(const std::string& path, PieceSamplerConfig& cfg);
bool validatePieceSamplerConfig(const PieceSamplerConfig& cfg);

#endif
