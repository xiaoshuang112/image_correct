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
    int run();

private:
    bool computeLookAtPose(double cx, double cy, double cz,
                           double r, double yaw_deg, double theta_deg, double out_pose[6]);
    int captureExposures(const std::string& prefix);
    int captureGains(const std::string& prefix);
    CRresult moveToFlangeAxisPos(double axisAnglePos[6]);

    std::shared_ptr<AICamera>& m_camera;
    ScrewSamplingConfig m_config;

    cv::Mat m_R_f_c;
    cv::Mat m_t_f_c;
    cv::Matx44d m_T_f_c;
};

#endif
