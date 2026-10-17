#include "CircleSampler.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <sys/stat.h>
#include <sys/types.h>
#include "robotapi.h"
#include "basestruct.h"
#include "utils/sampling_utils.hpp"

extern RobotHandle robotHandle;

// ============================================================
// CircleSampler 实现
// ============================================================

// --- Screw mode constructor (unchanged) ---
CircleSampler::CircleSampler(std::shared_ptr<AICamera>& camera, const ScrewSamplingConfig& config)
    : m_camera(camera), m_config(config) {
    m_R_f_c = config.T_flange_camera(cv::Rect(0, 0, 3, 3)).clone();
    m_t_f_c = config.T_flange_camera(cv::Rect(3, 0, 1, 3)).clone();
}

// --- Test mode constructor ---
CircleSampler::CircleSampler(std::shared_ptr<AICamera>& camera, const CircleTestConfig& config)
    : m_camera(camera), m_test_config(config) {
    // Borrow fields from test config for genExposures / capture timeout
    m_config.num_exposures = config.num_exposures;
    m_config.capture_timeout_ms = config.capture_timeout_ms;

    m_R_f_c = config.T_flange_camera(cv::Rect(0, 0, 3, 3)).clone();
    m_t_f_c = config.T_flange_camera(cv::Rect(3, 0, 1, 3)).clone();
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
    int minExp = 0, maxExp = 1000000;
    minExp = 4662; maxExp = 41592;

    int n = m_config.num_exposures;
    if (n < 2) n = 2;

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

// --- captureExposures: all exposures (screw mode, unchanged) ---
int CircleSampler::captureExposures(const std::string& prefix) {
    int count = 0;
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
        std::ostringstream filename;
        filename << prefix << "_" << i << ".jpg";
        cv::imwrite(filename.str(), frame);
        count++;
    }
    return count;
}

// --- captureExposures: specific exposure indices (test mode) ---
int CircleSampler::captureExposures(const std::string& prefix, const std::vector<int>& indices) {
    int count = 0;
    for (int exp_idx : indices) {
        if (exp_idx < 0 || exp_idx >= (int)_exp_values.size()) {
            std::cerr << "[captureExposures] Index out of range: " << exp_idx
                      << " (valid 0~" << (_exp_values.size() - 1) << "), skip" << std::endl;
            continue;
        }
        int exp = _exp_values[exp_idx];
        if (!m_camera->setColorExposureTime(exp)) {
            std::cerr << "[captureExposures] Set exposure " << exp << " us failed" << std::endl;
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Retry empty frame up to 2 times
        cv::Mat frame;
        for (int retry = 0; retry < 2; retry++) {
            frame = m_camera->captureOneFrame(m_config.capture_timeout_ms);
            if (!frame.empty()) break;
            std::cerr << "[captureExposures] Retry " << (retry + 1) << "/2 at exposure "
                      << exp << " us" << std::endl;
        }
        if (frame.empty()) {
            std::cerr << "[captureExposures] Empty frame at exposure " << exp << " us" << std::endl;
            continue;
        }

        std::ostringstream filename;
        filename << prefix << "_E" << exp_idx << ".jpg";
        cv::imwrite(filename.str(), frame);
        count++;
    }
    return count;
}

// ============================================================
// Checkpoint helpers (test mode)
// ============================================================
std::string CircleSampler::checkpointPath() const {
    return m_test_config.output_dir + "/.checkpoint.yaml";
}

bool CircleSampler::saveCheckpoint(int bg_idx, int combo_idx, int layer_idx, int point_idx) {
    cv::FileStorage fs(checkpointPath(), cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;
    fs << "checkpoint" << "{";
    fs << "bg_idx" << bg_idx;
    fs << "combo_idx" << combo_idx;
    fs << "layer_idx" << layer_idx;
    fs << "point_idx" << point_idx;
    fs << "}";
    return true;
}

// ============================================================
// Dry-run: verify IK reachability only, no capture
// ============================================================
int CircleSampler::dryRunTestCollection() {
    const auto& cfg = m_test_config;
    int total = (int)cfg.backgrounds.size() * (int)cfg.sides.size() * (int)cfg.thread_types.size()
                * (int)cfg.layers.size() * cfg.points_per_layer;
    int ok = 0, skip = 0;

    std::cout << "\n========== DRY RUN ==========" << std::endl;
    std::cout << "Verifying IK reachability for " << total << " poses..." << std::endl;

    for (size_t bi = 0; bi < cfg.backgrounds.size(); bi++) {
        for (size_t si = 0; si < cfg.sides.size(); si++) {
            for (size_t ti = 0; ti < cfg.thread_types.size(); ti++) {
                for (int yaw : cfg.layers) {
                    double theta_step = 360.0 / cfg.points_per_layer;
                    for (int pi = 0; pi < cfg.points_per_layer; pi++) {
                        double theta = cfg.start_angle + pi * theta_step;
                        double pose[6];
                        if (!computeLookAtPose(cfg.position.x, cfg.position.y, cfg.position.z,
                                               cfg.radius, yaw, theta, pose)) {
                            std::cout << "[SKIP] " << cfg.backgrounds[bi] << "/" << cfg.sides[si]
                                      << "/" << cfg.thread_types[ti] << " L" << yaw
                                      << " P" << std::setw(3) << std::setfill('0') << pi
                                      << " computeLookAtPose failed" << std::endl;
                            skip++;
                            continue;
                        }
                        double ref_j[ROB_AXIS_NUM], joint[ROB_AXIS_NUM];
                        if (cr_get_jointActualPos(robotHandle, ref_j) != sucess ||
                            cr_kineInverse(robotHandle, pose, ref_j, joint) != sucess) {
                            std::cout << "[SKIP] " << cfg.backgrounds[bi] << "/" << cfg.sides[si]
                                      << "/" << cfg.thread_types[ti] << " L" << yaw
                                      << " P" << std::setw(3) << std::setfill('0') << pi
                                      << " IK unreachable" << std::endl;
                            skip++;
                            continue;
                        }
                        ok++;
                    }
                }
            }
        }
    }

    std::cout << "\nDry-run complete: " << ok << " reachable, "
              << skip << " failed, " << total << " total" << std::endl;
    return ok;
}

// ============================================================
// Test collection entry
// ============================================================
int CircleSampler::runTestCollection() {
    const auto& cfg = m_test_config;

    if (cfg.backgrounds.empty() || cfg.sides.empty() || cfg.thread_types.empty() ||
        cfg.layers.empty() || cfg.exposure_indices.empty()) {
        std::cerr << "[CircleTest] Incomplete configuration" << std::endl;
        return 0;
    }

    // Generate the full log-spaced exposure list
    genExposures();

    // Compute totals
    int n_bg = (int)cfg.backgrounds.size();
    int n_combo = (int)cfg.sides.size() * (int)cfg.thread_types.size();  // 4 combos per bg
    int n_layer = (int)cfg.layers.size();
    int n_point = cfg.points_per_layer;
    int n_exp = (int)cfg.exposure_indices.size();
    int total_points = n_bg * n_combo * n_layer * n_point;
    int total_images = total_points * n_exp;

    std::cout << "\n========== Circle Test Collection ==========" << std::endl;
    std::cout << "  Backgrounds: " << n_bg;
    for (auto& b : cfg.backgrounds) std::cout << " " << b;
    std::cout << "\n  Combos (side+thread): " << n_combo;
    for (auto& s : cfg.sides)
        for (auto& t : cfg.thread_types)
            std::cout << " (" << s << "+" << t << ")";
    std::cout << "\n  Layers: ";
    for (int l : cfg.layers) std::cout << l << " ";
    std::cout << "\n  Points/layer: " << n_point << " (step=" << (360 / n_point) << "deg)";
    std::cout << "\n  Exposures/point: " << n_exp << " (indices ";
    for (int e : cfg.exposure_indices) std::cout << e << " ";
    std::cout << ")";
    std::cout << "\n  Total images: " << total_images << std::endl;
    std::cout << "  Output dir: " << cfg.output_dir << std::endl;
    std::cout << "=============================================\n" << std::endl;

    // Ensure output directory (creates parent dirs as needed)
    std::filesystem::create_directories(cfg.output_dir);

    // Collection starts from the beginning.
    int start_bg = 0, start_combo = 0, start_layer = 0, start_point = 0;

    int cumulative = 0;   // points completed in this run (for progress)
    int total_expected = n_bg * n_combo * n_layer * n_point;

    for (int bi = start_bg; bi < n_bg; bi++) {
        const std::string& bg = cfg.backgrounds[bi];

        // Prompt: change background board
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

        int combo_start = (bi == start_bg) ? start_combo : 0;
        for (int ci = combo_start; ci < n_combo; ci++) {
            int si = ci / (int)cfg.thread_types.size();
            int ti = ci % (int)cfg.thread_types.size();
            const std::string& side = cfg.sides[si];
            const std::string& thread = cfg.thread_types[ti];

            // Prompt: change workpiece
            if (ci > 0 || bi > 0) {
                std::cout << "\n请更换工件: " << side << " + " << thread
                          << "，按 Enter 继续..." << std::flush;
                std::cin.get();
            } else {
                std::cout << "工件: " << side << " + " << thread << std::endl;
            }

            int layer_start = (bi == start_bg && ci == combo_start) ? start_layer : 0;
            for (int li = layer_start; li < n_layer; li++) {
                int yaw = cfg.layers[li];
                double theta_step = 360.0 / n_point;
                double cx = cfg.position.x;
                double cy = cfg.position.y;
                double cz = cfg.position.z;

                int pt_start = (bi == start_bg && ci == combo_start && li == layer_start) ? start_point : 0;
                for (int pi = pt_start; pi < n_point; pi++) {
                    double theta = cfg.start_angle + pi * theta_step;

                    double pose[6];
                    if (!computeLookAtPose(cx, cy, cz, cfg.radius, yaw, theta, pose)) {
                        std::cerr << "[CircleTest] computeLookAtPose failed at θ="
                                  << theta << ", skip" << std::endl;
                        continue;
                    }

                    if (sucess != sampling::moveToFlangeAxisPos(robotHandle, pose)) {
                        std::cerr << "[CircleTest] Move failed at θ=" << theta << ", skip" << std::endl;
                        saveCheckpoint(bi, ci, li, pi + 1);
                        continue;
                    }

                    // Build filename: test_{bg}_{side}_{thread}_L{layer}_P{point:03d}_E{exp_idx}.jpg
                    std::ostringstream prefix;
                    prefix << cfg.output_dir << "/test_"
                           << bg << "_" << side << "_" << thread << "_"
                           << "L" << yaw << "_"
                           << "P" << std::setw(3) << std::setfill('0') << pi;

                    captureExposures(prefix.str(), cfg.exposure_indices);

                    // Save checkpoint after each point
                    saveCheckpoint(bi, ci, li, pi + 1);

                    cumulative++;
                    sampling::printProgress(cumulative, total_expected);
                } // points
                start_point = 0; // reset for next layer
            } // layers
            start_layer = 0; // reset for next combo
        } // combos
        start_combo = 0; // reset for next bg
    } // backgrounds

    // Clear checkpoint on successful completion
    std::remove(checkpointPath().c_str());

    std::cout << "\n\n========== Circle Test Collection Complete ==========" << std::endl;
    std::cout << "Total images: " << (cumulative * n_exp)
              << " (from " << cumulative << " points)" << std::endl;
    std::cout << "Output directory: " << cfg.output_dir << std::endl;

    return cumulative * n_exp;
}

// ============================================================
// Original screw run (unchanged)
// ============================================================
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

            std::filesystem::create_directories(sessionDir);

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

                    if (sucess != sampling::moveToFlangeAxisPos(robotHandle, pose)) {
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
                    int captured = captureExposures(prefix.str());
                    totalImages += captured;
                    std::cout << "  Captured " << captured << "/"
                              << m_config.num_exposures << " exposures" << std::endl;
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
