#include "PieceSampler.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include "robotapi.h"
#include "basestruct.h"
#include "utils/robot_util.hpp"

extern RobotHandle robotHandle;

// ============================================================
// PieceSampler 实现
// ============================================================

PieceSampler::PieceSampler(std::shared_ptr<AICamera>& camera, const PieceSamplerConfig& config)
    : m_camera(camera), m_config(config) {
    m_R_f_c = config.T_flange_camera(cv::Rect(0, 0, 3, 3)).clone();
    m_t_f_c = config.T_flange_camera(cv::Rect(3, 0, 1, 3)).clone();
    m_T_f_c = config.T_flange_camera;

    // Precompute fixed flange orientation for downward-pointing camera.
    // Camera in base frame: X→world X, Y→-world Y, Z→-world Z (straight down)
    cv::Mat R_b_c = (cv::Mat_<double>(3, 3) <<
        1,  0,  0,
        0, -1,  0,
        0,  0, -1);

    // R_b_f = R_b_c * R_f_c^T  (constant for all poses)
    cv::Mat R_b_f = R_b_c * m_R_f_c.t();
    cv::Mat rvec;
    cv::Rodrigues(R_b_f, rvec);
    m_rvec = rvec * 180.0 / CV_PI;   // radians → degrees

    // Precompute constant flange offset: offset = R_b_f * t_f_c
    cv::Mat offset = R_b_f * m_t_f_c;
    m_ox = offset.at<double>(0);
    m_oy = offset.at<double>(1);
    m_oz = offset.at<double>(2);
}

void PieceSampler::computeFixedPose(double px, double py, double pz, double flangAA[6], double out_pose[6]) {
    out_pose[0] = px - m_ox;
    out_pose[1] = py - m_oy;
    out_pose[2] = pz - m_oz;
    if(m_config.has_position){
    out_pose[3] = m_rvec.at<double>(0);
    out_pose[4] = m_rvec.at<double>(1);
    out_pose[5] = m_rvec.at<double>(2);
    }
    else{
        out_pose[3] = flangAA[3];
        out_pose[4] = flangAA[4];
        out_pose[5] = flangAA[5];
    }
}

CRresult PieceSampler::moveToFlangeAxisPos(double axisAnglePos[6]) {
    double ref_j[ROB_AXIS_NUM], joint[ROB_AXIS_NUM];
    if (cr_get_jointActualPos(robotHandle, ref_j) != sucess)
        return error;
    if (cr_kineInverse(robotHandle, axisAnglePos, ref_j, joint) != sucess)
        return error;
    return moveToJointPos(robotHandle, joint);
}

void PieceSampler::genExposures() {
    int minExp = 10, maxExp = 1800;
    m_camera->getExposureRange(minExp, maxExp);
    //minExp = 4662; maxExp = 41592;

    int n = m_config.num_exposures;
    if (n < 2) 
        n = 2;

    _exp_values.resize(n);

#ifdef useORB
    for (int i = 0; i < n; i++) {
        _exp_values[i] = minExp + (maxExp - minExp) * i / (n - 1);     
    }
#endif
#ifdef useMVS
     double logMin = std::log10(minExp + 1.0);
    double logMax = std::log10(maxExp);
    for (int i = 0; i < n; i++) {
        double logVal = logMin + (logMax - logMin) * i / (n - 1);     
        _exp_values[i] = (int)std::round(std::pow(10.0, logVal));
    }
 #endif
    

    std::cout << "  Exposure range: " << minExp << " ~ " << maxExp << " us, using "
              << n << " log-spaced values" << std::endl;
}

int PieceSampler::captureExposures(const std::string& prefix, const std::vector<int>& indices) {
    int count = 0;
    for (int exp_idx : indices) {
        if (exp_idx < 0 || exp_idx >= (int)_exp_values.size()) {
            std::cerr << "[PieceSampler] Exposure index " << exp_idx
                      << " out of range [0, " << (_exp_values.size() - 1) << "], skip" << std::endl;
            continue;
        }
        int exp = _exp_values[exp_idx];
        if (!m_camera->setColorExposureTime(exp)) {
            std::cerr << "[PieceSampler] Set exposure " << exp << " us failed" << std::endl;
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        cv::Mat frame;
        for (int retry = 0; retry < 2; retry++) {
            frame = m_camera->captureOneFrame(m_config.capture_timeout_ms);
            if (!frame.empty()) break;
            std::cerr << "[PieceSampler] Retry " << (retry + 1) << "/2 at exposure "
                      << exp << " us" << std::endl;
        }
        if (frame.empty()) {
            std::cerr << "[PieceSampler] Empty frame at exposure " << exp << " us" << std::endl;
            continue;
        }

        std::ostringstream filename;
        filename << prefix << "_E" << exp_idx << ".jpg";
        cv::imwrite(filename.str(), frame);
        count++;
    }
    return count;
}

void PieceSampler::printProgress(int current, int total) const {
    int bar_width = 50;
    int filled = current * bar_width / total;
    int pct = current * 100 / total;
    std::cout << "\r[";
    for (int i = 0; i < bar_width; i++)
        std::cout << (i < filled ? '#' : (i == filled ? '>' : ' '));
    std::cout << "] " << pct << "% (" << current << "/" << total << ")" << std::flush;
}

// ============================================================
// Checkpoint helpers
// ============================================================
std::string PieceSampler::checkpointPath() const {
    return m_config.output_dir + "/.checkpoint.yaml";
}

bool PieceSampler::saveCheckpoint(int bg_idx, int wp_idx, int z_idx, int xy_idx) {
    cv::FileStorage fs(checkpointPath(), cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;
    fs << "checkpoint" << "{";
    fs << "bg_idx" << bg_idx;
    fs << "wp_idx" << wp_idx;
    fs << "z_idx" << z_idx;
    fs << "xy_idx" << xy_idx;
    fs << "}";
    return true;
}

bool PieceSampler::loadCheckpoint(int& bg_idx, int& wp_idx, int& z_idx, int& xy_idx) {
    std::string path = checkpointPath();
    FILE* fp = fopen(path.c_str(), "r");
    if (!fp) return false;
    fclose(fp);

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    cv::FileNode chk = fs["checkpoint"];
    if (chk.empty()) return false;
    bg_idx = (int)chk["bg_idx"];
    wp_idx = (int)chk["wp_idx"];
    z_idx = (int)chk["z_idx"];
    xy_idx = (int)chk["xy_idx"];
    return true;
}

// ============================================================
// Main sampling run
// ============================================================
int PieceSampler::run() {
    const auto& cfg = m_config;

    if (cfg.backgrounds.empty() || cfg.workpiece_configs.empty() ||
        cfg.x_offsets.empty() || cfg.y_offsets.empty() || cfg.z_heights.empty() || cfg.exposure_indices.empty()) {
        std::cerr << "[PieceSampler] Incomplete configuration" << std::endl;
        return 0;
    }

    // Generate exposure list
    genExposures();

    int n_bg  = (int)cfg.backgrounds.size();
    int n_wp  = (int)cfg.workpiece_configs.size();
    int n_x   = (int)cfg.x_offsets.size();
    int n_y   = (int)cfg.y_offsets.size();
    int n_z   = (int)cfg.z_heights.size();
    int n_exp = (int)cfg.exposure_indices.size();
    int total_poses = n_bg * n_wp * n_z * n_x * n_y;
    int total_images = total_poses * n_exp;

    double cx, cy, cz, cur_flange[ROB_AXIS_NUM];
    if (cfg.has_position) {
        cx = cfg.position.x;
        cy = cfg.position.y;
        cz = cfg.position.z;
    } else {
        // Read current flange pose via forward kinematics
        double cur_joints[ROB_AXIS_NUM];
        if (cr_get_jointActualPos(robotHandle, cur_joints) != sucess ||
            cr_kineForward(robotHandle, cur_joints, cur_flange) != sucess) {
            std::cerr << "[PieceSampler] Failed to read current robot pose" << std::endl;
            return 0;
        }
        // Camera position = flange position + constant offset
        cx = cur_flange[0] + m_ox;
        cy = cur_flange[1] + m_oy;
        cz = cur_flange[2] + m_oz;
        std::cout << "\n  [Position not set in config, using current robot pose as center]" << std::endl;
    }

    std::cout << "\n========== Piece Sampler ==========" << std::endl;
    std::cout << "  Workpiece center: (" << cx << ", " << cy << ", " << cz << ") mm" << std::endl;
    std::cout << "  X offsets:";
    for (double d : cfg.x_offsets) std::cout << " " << d;
    std::cout << " (" << n_x << ")" << std::endl;
    std::cout << "  Y offsets:";
    for (double d : cfg.y_offsets) std::cout << " " << d;
    std::cout << " (" << n_y << ")" << std::endl;
    std::cout << "  Grid: " << n_x << " x " << n_y << " = " << (n_x * n_y) << " pts" << std::endl;
    std::cout << "  Z heights:";
    for (double h : cfg.z_heights) std::cout << " " << h;
    std::cout << std::endl;
    std::cout << "  Backgrounds: " << n_bg;
    for (auto& b : cfg.backgrounds) std::cout << " " << b;
    std::cout << std::endl;
    std::cout << "  Workpiece configs: " << n_wp;
    for (auto& w : cfg.workpiece_configs) std::cout << " " << w;
    std::cout << std::endl;
    std::cout << "  Exposures/pose: " << n_exp << " (indices";
    for (int e : cfg.exposure_indices) std::cout << " " << e;
    std::cout << ")" << std::endl;
    std::cout << "  Total poses: " << total_poses << std::endl;
    std::cout << "  Total images: " << total_images << std::endl;
    std::cout << "  Output dir: " << cfg.output_dir << std::endl;
    std::cout << "====================================\n" << std::endl;

    // Ensure output directory
    std::filesystem::create_directories(cfg.output_dir);

    // Check for existing checkpoint
    int start_bg = 0, start_wp = 0, start_z = 0, start_xy = 0;   

    int cumulative = 0;

    for (int bi = start_bg; bi < n_bg; bi++) {
        const std::string& bg = cfg.backgrounds[bi];

        if (bi > 0 || start_bg > 0) {
            std::cout << "\n========================================" << std::endl;
            std::cout << "背景 " << (bi + 1) << "/" << n_bg
                      << " 已完成，请更换背景为: " << bg << std::endl;
            std::cout << "按 Enter 继续..." << std::flush;
            std::cin.get();
        } else {
            std::cout << "\n请放置背景: " << bg << "，按 Enter 继续..." << std::flush;
            std::cin.get();
        }

        int wp_start = (bi == start_bg) ? start_wp : 0;
        for (int wi = wp_start; wi < n_wp; wi++) {
            const std::string& wp = cfg.workpiece_configs[wi];

            std::cout << "\n请摆放工件: " << wp
                      << "，按 Enter 继续..." << std::flush;
            std::cin.get();

            int z_start = (bi == start_bg && wi == wp_start) ? start_z : 0;
            double max_z = *std::max_element(cfg.z_heights.begin(), cfg.z_heights.end());
            for (int zi = z_start; zi < n_z; zi++) {
                double dz = cfg.z_heights[zi], z_shrink = 1;//sqrt(dz / max_z);

                int xy_start = (bi == start_bg && wi == wp_start && zi == z_start) ? start_xy : 0;
                for (int xyi = xy_start; xyi < n_x * n_y; xyi++) {
                    int xi = xyi / n_y;
                    int yi = xyi % n_y;
                    double dx = cfg.x_offsets[xi] * z_shrink;
                    double dy = cfg.y_offsets[yi] * z_shrink;

                    double pose[6];
                    computeFixedPose(cx + dx, cy + dy, cz + dz, cur_flange, pose);

                    if (sucess != moveToFlangeAxisPos(pose)) {
                        std::cerr << "[PieceSampler] Move failed at "
                                  << "Z=" << dz << " X=" << dx << " Y=" << dy << std::endl;
                        saveCheckpoint(bi, wi, zi, xyi + 1);
                        continue;
                    }

                    std::ostringstream prefix;
                    prefix << cfg.output_dir << "/piece_"
                           << bg << "_" << wp << "_"
                           << "Z" << (int)dz << "_"
                           << "X" << (int)dx << "_"
                           << "Y" << (int)dy;

                    captureExposures(prefix.str(), cfg.exposure_indices);

                    saveCheckpoint(bi, wi, zi, xyi + 1);
                    cumulative++;
                    printProgress(cumulative, total_poses);
                } // xy
                start_xy = 0;
            } // z
            start_z = 0;
        } // workpiece
        start_wp = 0;
    } // background

    // Clear checkpoint on success
    std::remove(checkpointPath().c_str());

    std::cout << "\n\n========== Piece Sampling Complete ==========" << std::endl;
    std::cout << "Total poses: " << cumulative << std::endl;
    std::cout << "Total images: " << (cumulative * n_exp) << std::endl;
    std::cout << "Output directory: " << cfg.output_dir << std::endl;

    return cumulative * n_exp;
}
