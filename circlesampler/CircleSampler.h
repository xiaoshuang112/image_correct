#ifndef CIRCLESAMPLER_H
#define CIRCLESAMPLER_H

#include <memory>
#include <opencv2/opencv.hpp>
#include "cameras/AICamera.hpp"
#include "Config.h"
#include "robotapi.h"

class CircleSampler {
public:
    CircleSampler(std::shared_ptr<AICamera>& camera, const ScrewSamplingConfig& config);
    CircleSampler(std::shared_ptr<AICamera>& camera, const CircleTestConfig& config);
    int run();
    int runTestCollection();
    int dryRunTestCollection();

private:
    bool computeLookAtPose(double cx, double cy, double cz,
                           double r, double yaw_deg, double theta_deg, double out_pose[6]);
    int captureExposures(const std::string& prefix);
    int captureExposures(const std::string& prefix, const std::vector<int>& indices);
    CRresult moveToFlangeAxisPos(double axisAnglePos[6]);
    void genExposures();
    void printProgress(int current, int total) const;

    // Checkpoint helpers (test mode)
    std::string checkpointPath() const;
    bool saveCheckpoint(int bg_idx, int combo_idx, int layer_idx, int point_idx);

    std::shared_ptr<AICamera>& m_camera;
    ScrewSamplingConfig m_config;
    CircleTestConfig m_test_config;

    cv::Mat m_R_f_c;
    cv::Mat m_t_f_c;
    std::vector<int> _exp_values;
};

#endif
