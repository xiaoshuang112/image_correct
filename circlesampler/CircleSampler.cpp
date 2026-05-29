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

extern RobotHandle robotHandle;

// ============================================================
// 内部：运动到目标关节角
// ============================================================
static bool moveToJointPos(const double joint[6]) {
    double pose[6] = {0, 0, 0, 0, 0, 0};
    cr_get_jointActualPos(robotHandle, pose);

    PointControlParaSimple simple;
    for (int i = 0; i < ROB_AXIS_NUM; i++) {
        simple.pose[i] = pose[i];
        simple.jointpos[i] = joint[i];
        simple.tcpOffset[i] = 0;
        simple.coordinatePose[i] = 0;
        simple.speed[i] = 30;
        simple.acc[i] = 30;
    }
    simple.coordinateType = jointCoordinate;
    simple.pointTransType = pointTransStop;
    simple.motiontriggerMode = MovetriggerbyOnlyRpc;

    PointControlPara para;
    CRresult ret = cr_move_pointControlPara_transfer(robotHandle, simple, &para);
    if (ret != sucess) {
        std::cerr << "[CircleSampler] cr_move_pointControlPara_transfer failed: "
                  << ret << std::endl;
        return false;
    }
    ret = cr_move_joint(robotHandle, para, TRUE);
    if (ret != sucess) {
        std::cerr << "[CircleSampler] cr_move_joint failed: " << ret << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return true;
}

// ============================================================
// CircleSampler 实现
// ============================================================

CircleSampler::CircleSampler(std::shared_ptr<AICamera>& camera, const ScrewSamplingConfig& config)
    : m_camera(camera), m_config(config) {
    m_R_f_c = config.T_flange_camera(cv::Rect(0, 0, 3, 3)).clone();
    m_t_f_c = config.T_flange_camera(cv::Rect(3, 0, 1, 3)).clone();
    m_T_f_c = config.T_flange_camera;
}

bool CircleSampler::moveToFlangeAxisPos(double axisAnglePos[6]) {
    double ref_j[ROB_AXIS_NUM], joint[ROB_AXIS_NUM];
    if (cr_get_jointActualPos(robotHandle, ref_j) != sucess)
        return false;
    if (cr_kineInverse(robotHandle, axisAnglePos, ref_j, joint) != sucess)
        return false;
    return moveToJointPos(joint);
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

int CircleSampler::captureExposures(const std::string& prefix) {
    int count = 0;

    int minGain = 0, maxGain = 255;
    if (!m_camera->getGainRange(minGain, maxGain) || maxGain <= minGain) {
        std::cerr << "[CircleSampler] Failed to get gain range, using [0, 255]" << std::endl;
        minGain = 0; maxGain = 255;
    }

    int n = m_config.num_exposures;
    if (n < 2) n = 2;

    int gainMax = minGain + (int)(0.45 * (maxGain - minGain));

    std::vector<int> gain_values(n);
    for (int i = 0; i < n; i++) {
        gain_values[i] = minGain + (gainMax - minGain) * i / (n - 1);
    }

    std::cout << "  Gain range: " << minGain << " ~ " << maxGain << ", using "
              << n << " values: " << gain_values.front() << " ~ " << gain_values.back() << std::endl;

    m_camera->setAutoExposure(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (int i = 0; i < n; i++) {
        int gain = gain_values[i];
        if (!m_camera->setColorGain(gain)) {
            std::cerr << "[CircleSampler] Set gain " << gain << " failed" << std::endl;
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        cv::Mat frame = m_camera->captureOneFrame(m_config.capture_timeout_ms);
        if (frame.empty()) {
            std::cerr << "[CircleSampler] Empty frame at gain " << gain << std::endl;
            continue;
        }

        std::ostringstream filename;
        filename << prefix << "_" << i << ".jpg";
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

    int totalImages = 0;

    for (double r : m_config.radii) {
        std::time_t now = std::time(nullptr);
        std::tm* tm = std::localtime(&now);
        std::ostringstream dirName;
        dirName << m_config.output_dir << "/screw_"
                << m_config.background_type << "_"
                << r << "_"
                << m_config.cx << "_"
                << m_config.cy << "_"
                << std::put_time(tm, "%Y-%m-%d_%H-%M-%S");
        std::string sessionDir = dirName.str();

        mkdir(sessionDir.c_str(), 0755);

        std::cout << "\n========== Radius r=" << r << " mm ==========" << std::endl;
        std::cout << "Session: " << sessionDir << std::endl;

        for (int yaw : m_config.yaw_angles) {
            for (int theta = 0; theta < 360; theta++) {
                double pose[6];
                if (!computeLookAtPose(m_config.cx, m_config.cy, m_config.cz,
                                       r, yaw, theta, pose)) {
                    std::cerr << "[CircleSampler] Failed to compute pose at θ=" << theta << std::endl;
                    continue;
                }

                std::cout << "[" << (theta + 1) << "/360] Moving to θ=" << theta
                          << "°  pos=(" << std::fixed << std::setprecision(1)
                          << pose[0] << "," << pose[1] << "," << pose[2] << ")" << std::endl;

                if (!moveToFlangeAxisPos(pose)) {
                    std::cerr << "[CircleSampler] Move failed at θ=" << theta << ", skipping" << std::endl;
                    continue;
                }

                std::ostringstream prefix;
                prefix << sessionDir << "/screw_"
                       << m_config.background_type << "_"
                       << r << "_"
                       << m_config.cx << "_"
                       << m_config.cy << "_y"
                       << std::setw(3) << std::setfill('0') << yaw
                       << "_p"
                       << std::setw(3) << std::setfill('0') << theta;

                int captured = captureExposures(prefix.str());
                totalImages += captured;

                std::cout << "  Captured " << captured << "/"
                          << m_config.num_exposures << " exposures" << std::endl;
            }
        }

        std::cout << "\nRadius " << r << " done. Total this radius: "
                  << (360 * (int)m_config.yaw_angles.size() * m_config.num_exposures) << " expected, "
                  << totalImages << " captured" << std::endl;
    }

    std::cout << "\n========== Sampling Complete ==========" << std::endl;
    std::cout << "Total images: " << totalImages << std::endl;
    return totalImages;
}
