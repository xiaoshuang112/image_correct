#include "CircleSampler.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <ctime>
#include <sys/stat.h>
#include "robotapi.h"
#include "basestruct.h"
#include "utils/robot_util.hpp"

extern RobotHandle robotHandle;

// ============================================================
// CircleSampler 实现
// ============================================================

CircleSampler::CircleSampler(std::shared_ptr<AICamera>& camera, const ScrewSamplingConfig& config)
    : m_camera(camera), m_config(config) {
    m_R_f_c = config.T_flange_camera(cv::Rect(0, 0, 3, 3)).clone();
    m_t_f_c = config.T_flange_camera(cv::Rect(3, 0, 1, 3)).clone();
    m_T_f_c = config.T_flange_camera;
}

CRresult CircleSampler::moveToFlangeAxisPos(double axisAnglePos[6]) {
    double ref_j[ROB_AXIS_NUM], joint[ROB_AXIS_NUM];
    if (cr_get_jointActualPos(robotHandle, ref_j) != sucess)
        return error;
    if (cr_kineInverse(robotHandle, axisAnglePos, ref_j, joint) != sucess)
        return error;
    return moveToJointPos(robotHandle, joint);
}

bool CircleSampler::computeLookAtPose(double cx, double cy, double cz,
                                      double r, double yaw_deg, double theta_deg, double out_pose[6]) {
    double theta = theta_deg * CV_PI / 180.0;

    double cam_x = cx + r * std::cos(theta);
    double cam_y = cy + r * std::sin(theta);
    double cam_z = cz + r * std::tan(yaw_deg * CV_PI / 180.0);

    cv::Mat z_cam = (cv::Mat_<double>(3, 1) << cx - cam_x, cy - cam_y, cz - cam_z);
    double len = cv::norm(z_cam);
    if (len < 1e-10)
        return false;
    z_cam /= len;

    cv::Mat up = (cv::Mat_<double>(3, 1) << 0, 0, 1);
    cv::Mat x_cam = up.cross(z_cam);
    if (cv::norm(x_cam) < 1e-6) {
        up = (cv::Mat_<double>(3, 1) << 0, 1, 0);
        x_cam = up.cross(z_cam);
    }
    x_cam /= cv::norm(x_cam);
    cv::Mat y_cam = z_cam.cross(x_cam);
    y_cam /= cv::norm(y_cam);

    cv::Mat R_b_c(3, 3, CV_64F);
    x_cam.copyTo(R_b_c.col(0));
    y_cam.copyTo(R_b_c.col(1));
    z_cam.copyTo(R_b_c.col(2));

    cv::Mat R_b_f = R_b_c * m_R_f_c.t();

    cv::Mat flange_offset = R_b_f * m_t_f_c;
    double fx = cam_x - flange_offset.at<double>(0);
    double fy = cam_y - flange_offset.at<double>(1);
    double fz = cam_z - flange_offset.at<double>(2);

    cv::Mat rvec;
    cv::Rodrigues(R_b_f, rvec);

    out_pose[0] = fx;
    out_pose[1] = fy;
    out_pose[2] = fz;
    out_pose[3] = rvec.at<double>(0) * 180.0 / CV_PI;
    out_pose[4] = rvec.at<double>(1) * 180.0 / CV_PI;
    out_pose[5] = rvec.at<double>(2) * 180.0 / CV_PI;

    return true;
}

void CircleSampler::genExposures(){
     // Query exposure time range
    int minExp = 0, maxExp = 1000000;
    // if (!m_camera->getExposureRange(minExp, maxExp) || maxExp <= minExp) {
    //     std::cerr << "[Sampler] Failed to get exposure range, using [100, 100000] us" << std::endl;
    //     minExp = 100; maxExp = 100000;
    // }
    minExp = 4662; maxExp = 41592;
    // Disable auto exposure
    // m_camera->setAutoExposure(false);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int n = m_config.num_exposures;
    if (n < 2) n = 2;

    // Generate logarithmically spaced exposure values to cover a wide range
    //std::vector<int> exp_values(n);
    _exp_values = std::vector<int>(n);
    double logMin = std::log10(minExp + 1.0);
    double logMax = std::log10(maxExp);
    for (int i = 0; i < n; i++) {
        double logVal = logMin + (logMax - logMin) * i / (n - 1);
        _exp_values[i] = (int)std::round(std::pow(10.0, logVal));
    }

    std::cout << "  Exposure range: " << minExp << " ~ " << maxExp << " us, using "
              << n << " log-spaced values: " << _exp_values.front() << " ~ " << _exp_values.back() << " us" << std::endl;
}

int CircleSampler::captureExposures(const std::string& prefix) {
    int count = 0;

   
  //  m_camera->setColorExposureTime(524);
    //m_camera->setColorExposureTime(1846);
    for (int i = 0; i < _exp_values.size(); i++) {
        int exp = _exp_values[i];

        if (!m_camera->setColorExposureTime(exp)) {
            std::cerr << "[Sampler] Set exposure " << exp << " us failed" << std::endl;
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        cv::Mat frame = m_camera->captureOneFrame(m_config.capture_timeout_ms);
        if (frame.empty()) {
            std::cerr << "[Sampler] Empty frame at exposure " << exp << " us" << std::endl;
            continue;
        }

        // Build filename: prefix_{exposure_idx}.jpg
        std::ostringstream filename;
        filename << prefix << "_" << i << ".jpg";

        cv::imwrite(filename.str(), frame);
        count++;
    }

    return count;
}

int CircleSampler::captureGains(const std::string& prefix) {
    int count = 0;

    // Query gain range
    int minGain = 0, maxGain = 24;
    if (!m_camera->getGainRange(minGain, maxGain) || maxGain <= minGain) {
        std::cerr << "[Sampler] Failed to get gain range, using [0, 24] dB" << std::endl;
        minGain = 0; maxGain = 24;
    }

    int n = m_config.num_exposures;
    if (n < 2) n = 2;

    // Generate linearly spaced gain values
    std::vector<int> gain_values(n);
    for (int i = 0; i < n; i++) {
        gain_values[i] = minGain + (maxGain - minGain) * i / (n - 1);
    }

    std::cout << "  Gain range: " << minGain << " ~ " << maxGain << " dB, using "
              << n << " linearly spaced values: " << gain_values.front() << " ~ " << gain_values.back() << " dB" << std::endl;

    for (int i = 0; i < n; i++) {
        int gain = gain_values[i];

        if (!m_camera->setColorGain(gain)) {
            std::cerr << "[Sampler] Set gain " << gain << " dB failed" << std::endl;
            continue;
        }

        cv::Mat frame = m_camera->captureOneFrame(m_config.capture_timeout_ms);
        if (frame.empty()) {
            std::cerr << "[Sampler] Empty frame at gain " << gain << " dB" << std::endl;
            continue;
        }

        std::ostringstream filename;
        filename << prefix << "_gain_" << i << ".jpg";

        cv::imwrite(filename.str(), frame);
        count++;
    }

    return count;
}

int CircleSampler::run() {
    if (m_config.radii.empty()) {
        std::cerr << "[CircleSampler] No radii configured" << std::endl;
        return 0;
    }
    if (m_config.num_exposures < 1) {
        std::cerr << "[CircleSampler] num_exposures must be >= 1" << std::endl;
        return 0;
    }
    if (m_config.positions.empty()) {
        std::cerr << "[CircleSampler] No positions configured" << std::endl;
        return 0;
    }
    genExposures();
    int totalImages = 0;

    for (size_t pi = 0; pi < m_config.positions.size(); pi++) {
        double cx = m_config.positions[pi].x;
        double cy = m_config.positions[pi].y;
        double cz = m_config.positions[pi].z;

        std::cout << "\n========== Position " << pi << ": ("
                  << cx << ", " << cy << ", " << cz << ") mm ==========" << std::endl;

        for (double r : m_config.radii) {
            std::time_t now = std::time(nullptr);
            std::tm* tm = std::localtime(&now);
            std::ostringstream dirName;
            dirName << m_config.output_dir << "/screw_"
                    << m_config.background_type << "_"
                    << r << "_p" << pi << "_"
                    << std::put_time(tm, "%Y-%m-%d_%H-%M-%S");
            std::string sessionDir = dirName.str();

            mkdir(sessionDir.c_str(), 0755);

            std::cout << "\n========== Radius r=" << r << " mm ==========" << std::endl;
            std::cout << "Session: " << sessionDir << std::endl;

            for (int yaw : m_config.yaw_angles) {
                for (int theta = 0; theta < 360; theta += 2) {
                    double pose[6];
                    if (!computeLookAtPose(cx, cy, cz,
                                           r, yaw, theta, pose)) {
                        std::cerr << "[CircleSampler] Failed to compute pose at θ=" << theta << std::endl;
                        continue;
                    }

                    std::cout << "[" << (theta + 1) << "/180] Moving to θ=" << theta
                              << "°  pos=(" << std::fixed << std::setprecision(1)
                              << pose[0] << "," << pose[1] << "," << pose[2] << ")" << std::endl;

                    if ( sucess !=moveToFlangeAxisPos(pose)) {
                        std::cerr << "[CircleSampler] Move failed at θ=" << theta << ", skipping" << std::endl;
                        continue;
                    }

                    std::ostringstream prefix;
                    prefix << sessionDir << "/screw_"
                           << m_config.background_type << "_"
                           << r << "_p" << pi << "_y"
                           << std::setw(3) << std::setfill('0') << yaw
                           << "_p"
                           << std::setw(3) << std::setfill('0') << theta;
                    //m_camera->setColorExposureTime(149);
                    int captured = captureExposures(prefix.str());
                    totalImages += captured;
                    std::cout << "  Captured " << captured << "/"
                              << m_config.num_exposures << " exposures" << std::endl;
                    //  totalImages += captureGains(prefix.str());
                }
            }

            std::cout << "\nRadius " << r << " done. Total this radius: "
                      << (180 * (int)m_config.yaw_angles.size() * m_config.num_exposures) << " expected, "
                      << totalImages << " captured" << std::endl;
        }
    }

    std::cout << "\n========== Sampling Complete ==========" << std::endl;
    std::cout << "Total images: " << totalImages << std::endl;
    return totalImages;
}
