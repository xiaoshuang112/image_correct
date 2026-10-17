#ifndef PIECESAMPLER_H
#define PIECESAMPLER_H

#include <memory>
#include <opencv2/opencv.hpp>
#include "cameras/AICamera.hpp"
#include "Config.h"

class PieceSampler {
public:
    PieceSampler(std::shared_ptr<AICamera>& camera, const PieceSamplerConfig& config);

    int run();

private:
    // Fixed downward camera: compute flange pose for camera at (px, py, pz)
    void computeFixedPose(double px, double py, double pz, double flangAA[6], double out_pose[6]);

    // Capture exposures for specific indices
    int captureExposures(const std::string& prefix, const std::vector<int>& indices);

    // Generate log-spaced exposure list
    void genExposures();

    // Checkpoint helpers
    std::string checkpointPath() const;
    bool saveCheckpoint(int bg_idx, int wp_idx, int z_idx, int xy_idx);

    std::shared_ptr<AICamera>& m_camera;
    PieceSamplerConfig m_config;

    cv::Mat m_R_f_c;       // rotation: flange -> camera
    cv::Mat m_t_f_c;       // translation: flange -> camera

    // Precomputed for fixed downward camera
    cv::Mat m_rvec;        // axis-angle of fixed flange orientation (3x1, degrees)
    double m_ox, m_oy, m_oz;  // R_b_f * t_f_c (constant flange offset)

    std::vector<int> _exp_values;
};

#endif
