#include "eye_to_hand_calibration.hpp"
#include "mechanical_settling_config.hpp"
#include "calibration_utils.hpp"
#include "auto_eye_in_hand_charuco_calibration.hpp"
#include "camera_calibration.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <thread>
#include <chrono>

#include "utils/robot_util.hpp"
#include "utils/calib_transform.hpp"
#include "candidate_planner.hpp"
#include <set>
#include <cctype>
#include <stdexcept>

using namespace calib_transform;

namespace {

struct RigidTransformDistance {
    double translation_mm = std::numeric_limits<double>::quiet_NaN();
    double rotation_deg = std::numeric_limits<double>::quiet_NaN();
};

bool isValidRigidTransform(const cv::Mat& input) {
    if (input.rows != 4 || input.cols != 4 || input.channels() != 1 ||
        !cv::checkRange(input))
        return false;
    cv::Mat transform;
    input.convertTo(transform, CV_64F);
    const cv::Mat rotation = transform(cv::Rect(0, 0, 3, 3));
    const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
    const cv::Mat bottom = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
    return cv::norm(rotation.t() * rotation - identity) < 1e-6 &&
        std::abs(cv::determinant(rotation) - 1.0) < 1e-6 &&
        cv::norm(transform.row(3) - bottom) < 1e-9;
}

RigidTransformDistance rigidTransformDistance(const cv::Mat& left_input,
                                              const cv::Mat& right_input) {
    RigidTransformDistance distance;
    if (!isValidRigidTransform(left_input) || !isValidRigidTransform(right_input))
        return distance;
    cv::Mat left, right;
    left_input.convertTo(left, CV_64F);
    right_input.convertTo(right, CV_64F);
    distance.translation_mm = cv::norm(
        right(cv::Rect(3, 0, 1, 3)) - left(cv::Rect(3, 0, 1, 3))) * 1000.0;
    const cv::Mat relative_rotation =
        left(cv::Rect(0, 0, 3, 3)).t() * right(cv::Rect(0, 0, 3, 3));
    const double cosine = std::clamp(
        (cv::trace(relative_rotation)[0] - 1.0) * 0.5, -1.0, 1.0);
    distance.rotation_deg = std::acos(cosine) * 180.0 / CV_PI;
    return distance;
}

}  // namespace

EyeToHandPublicationCheck checkEyeToHandPublication(
    const EyeToHandResult& candidate,
    const std::string& reference_yaml,
    const PoseGenerationConfig& config) {
    EyeToHandPublicationCheck check;
    auto reject = [&](const std::string& reason) {
        check.passed = false;
        check.message = "眼在手外结果禁止发布: " + reason;
        return check;
    };
    if (!candidate.success || !isValidRigidTransform(candidate.T_base_camera))
        return reject("候选标定结果无效");
    if (candidate.valid_frames < config.minimum_publish_frames)
        return reject("有效帧数 " + std::to_string(candidate.valid_frames) +
                      " 小于门限 " + std::to_string(config.minimum_publish_frames));

    const double translation_rms_mm = candidate.reproj_error * 1000.0;
    const double translation_p95_mm = candidate.quality.translation_p95_mm;
    const double rotation_rms_deg = candidate.quality.rotation_rms_deg;
    if (!std::isfinite(translation_rms_mm) || !std::isfinite(translation_p95_mm) ||
        !std::isfinite(rotation_rms_deg))
        return reject("质量指标缺失或非有限值");
    if (translation_rms_mm > config.maximum_publish_translation_rms_mm)
        return reject("平移 RMS " + std::to_string(translation_rms_mm) +
                      " mm 超过门限 " +
                      std::to_string(config.maximum_publish_translation_rms_mm) + " mm");
    if (translation_p95_mm > config.maximum_publish_translation_p95_mm)
        return reject("平移 P95 " + std::to_string(translation_p95_mm) +
                      " mm 超过门限 " +
                      std::to_string(config.maximum_publish_translation_p95_mm) + " mm");
    if (rotation_rms_deg > config.maximum_publish_rotation_rms_degrees)
        return reject("旋转 RMS " + std::to_string(rotation_rms_deg) +
                      " deg 超过门限 " +
                      std::to_string(config.maximum_publish_rotation_rms_degrees) + " deg");
    if (!candidate.quality.consensus_available || !candidate.quality.consensus_passed)
        return reject("求解器共识未通过");

    namespace fs = std::filesystem;
    if (!fs::exists(reference_yaml)) {
        if (config.require_reference_result)
            return reject("缺少受保护参考结果 " + reference_yaml);
        check.passed = true;
        check.message = "发布质量门通过（未配置参考结果）";
        return check;
    }

    cv::Mat reference;
    try {
        cv::FileStorage storage(reference_yaml, cv::FileStorage::READ);
        if (!storage.isOpened())
            return reject("无法打开参考结果 " + reference_yaml);
        storage["T_base_camera"] >> reference;
        storage.release();
    } catch (const cv::Exception& exception) {
        return reject("参考结果读取失败 " + reference_yaml + ": " + exception.what());
    }
    if (!isValidRigidTransform(reference))
        return reject("参考结果不是有效刚体变换 " + reference_yaml);
    check.reference_loaded = true;
    const RigidTransformDistance drift =
        rigidTransformDistance(reference, candidate.T_base_camera);
    check.translation_drift_mm = drift.translation_mm;
    check.rotation_drift_deg = drift.rotation_deg;
    if (drift.translation_mm > config.maximum_publish_translation_drift_mm)
        return reject("相对参考结果平移漂移 " + std::to_string(drift.translation_mm) +
                      " mm 超过门限 " +
                      std::to_string(config.maximum_publish_translation_drift_mm) + " mm");
    if (drift.rotation_deg > config.maximum_publish_rotation_drift_degrees)
        return reject("相对参考结果旋转漂移 " + std::to_string(drift.rotation_deg) +
                      " deg 超过门限 " +
                      std::to_string(config.maximum_publish_rotation_drift_degrees) + " deg");
    check.passed = true;
    check.message = "发布门通过: 相对参考结果平移漂移 " +
        std::to_string(drift.translation_mm) + " mm, 旋转漂移 " +
        std::to_string(drift.rotation_deg) + " deg";
    return check;
}

bool publishEyeToHandResult(
    const EyeToHandResult& candidate,
    const std::string& reference_yaml,
    const std::string& production_yaml,
    const PoseGenerationConfig& config,
    EyeToHandPublicationCheck& check,
    std::string& error) {
    error.clear();
    check = checkEyeToHandPublication(candidate, reference_yaml, config);
    if (!check.passed) {
        error = check.message;
        return false;
    }
    if (!saveCalibrationResult(production_yaml, "T_base_camera",
                               candidate.T_base_camera, candidate.camera_matrix,
                               candidate.dist_coeffs,
                               candidate.reproj_error * 1000.0,
                               candidate.valid_frames)) {
        error = "眼在手外生产结果保存失败: " + production_yaml;
        return false;
    }
    return true;
}

// ============================================================
// Phase 2: 位姿生成 + IK 过滤
// ============================================================
static bool generateEyeToHandCandidates(
    int robot_handle,
    const double flangeEuleInit[6],
    const PoseGenerationConfig& config,
    std::vector<PoseCandidate>& candidates,
    std::string& error) {

    candidates.clear();
    double initial_joint[6]{};
    if (cr_get_jointActualPos((RobotHandle)robot_handle, initial_joint) != sucess) {
        error = "读取候选规划初始关节角失败";
        return false;
    }
    if (!std::all_of(initial_joint, initial_joint + 6,
                     [](double value) { return std::isfinite(value); })) {
        error = "候选规划初始关节角包含非有限值";
        return false;
    }
    if (!std::all_of(flangeEuleInit, flangeEuleInit + 6,
                     [](double value) { return std::isfinite(value); })) {
        error = "示教原点法兰欧拉位姿包含非有限值";
        return false;
    }
    if (!std::isfinite(config.maximum_joint_step_degrees) ||
        config.maximum_joint_step_degrees <= 0.0 ||
        config.maximum_joint_step_degrees > 180.0) {
        error = "maximum_joint_step_degrees must be in (0, 180]";
        return false;
    }

    // Keep the complete offset product (729 candidates with the default config).
    double euler[6]{}, joint[6]{};
    int source_index = 0;
    for (double dx : config.offset_x) {
    for (double dy : config.offset_y) {
    for (double dz : config.offset_z) {
    for (double drx : config.offset_rx) {
    for (double dry : config.offset_ry) {
    for (double drz : config.offset_rz) {
        if (robotMotionCancellationRequested()) {
            error = "标定已取消";
            return false;
        }
        euler[0] = flangeEuleInit[0] + dx;
        euler[1] = flangeEuleInit[1] + dy;
        euler[2] = flangeEuleInit[2] + dz;
        euler[3] = flangeEuleInit[3] + drx;
        euler[4] = flangeEuleInit[4] + dry;
        euler[5] = flangeEuleInit[5] + drz;
        PoseCandidate candidate{};
        candidate.source_index = source_index++;
        if (convertFlangeOulePos2Joint(robot_handle, euler, joint) &&
            cr_Eule2AxisAngle(euler, candidate.axis_angle) == sucess) {
            std::copy(joint, joint + 6, candidate.joint);
            std::copy(euler, euler + 6, candidate.flange_euler);
            candidates.push_back(candidate);
        }
    }}}}}}

    const auto reachable = candidates.size();
    CandidatePlanningOptions options;
    options.maximum_candidates = reachable;
    options.maximum_joint_step_degrees = config.maximum_joint_step_degrees;
    options.joint_resolver = [robot_handle](
        const PoseCandidate& candidate, const std::array<double, 6>& reference,
        std::array<double, 6>& resolved, int& error_code) {
        double target[6]{}, reference_joint[6]{};
        std::copy(candidate.axis_angle, candidate.axis_angle + 6, target);
        std::copy(reference.begin(), reference.end(), reference_joint);
        const CRresult ret = cr_kineInverse((RobotHandle)robot_handle,
                                            target, reference_joint, resolved.data());
        error_code = static_cast<int>(ret);
        return ret == sucess;
    };
    CandidatePlanningReport report;
    if (!planPoseCandidates(candidates, initial_joint, options, report, error))
        return false;
    std::cout << "[eyeToHand] 候选=" << source_index << ", IK可达=" << reachable
              << ", 选取=" << report.selected_candidates
              << ", 过渡=" << report.transition_candidates
              << ", 不连通=" << report.disconnected_candidates
              << ", 最大关节跳变=" << report.maximum_joint_step_degrees
              << " deg" << std::endl;
    return !candidates.empty();
}

static std::string imagePath(const std::string& directory, std::size_t index) {
    std::ostringstream path;
    path << directory << "/" << std::setw(3) << std::setfill('0') << index << ".jpg";
    return path.str();
}

static bool waitForSettling() {
    const int requested_ms = mechanicalSettlingWaitMs();
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(requested_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (robotMotionCancellationRequested()) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return !robotMotionCancellationRequested();
}

static bool readActualFlangeAxisAngle(int robot_handle, double flange_axis[6]) {
    double tcp_axis[6]{};
    return cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_axis) == sucess &&
        convertTcpAxisPos2FlangPos(robot_handle, tcp_axis, flange_axis) &&
        std::all_of(flange_axis, flange_axis + 6,
                    [](double value) { return std::isfinite(value); });
}

// Return -1 for cancellation, -2 for a fatal motion/read/write failure.
static int executeEyeToHandCollection(
    int robot_handle,
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    const std::vector<PoseCandidate>& candidates,
    const std::string& save_dir,
    std::vector<cv::Mat>& robot_poses,
    int min_corners,
    double maximum_joint_step_degrees,
    double maximum_capture_translation_drift_mm,
    double maximum_capture_rotation_drift_degrees,
    std::string& error,
    AI_det_callback callback = nullptr) {

    robot_poses.clear();
    const int total = static_cast<int>(candidates.size());
    int captured = 0;
    int empty_frames = 0, rejected_frames = 0, unstable_frames = 0;
    int missed_candidates = 0;
    try {
        std::ofstream poses(save_dir + "/rob_pos.csv");
        std::ofstream joints(save_dir + "/rob_joint.csv");
        std::ofstream eulers(save_dir + "/rob_eule.csv");
        for (auto* stream : {&poses, &joints, &eulers}) {
            stream->exceptions(std::ios::failbit | std::ios::badbit);
            *stream << std::fixed << std::setprecision(9);
        }
        for (int i = 0; i < total; ++i) {
            if (robotMotionCancellationRequested()) return -1;
            double current_joint[6]{};
            if (cr_get_jointActualPos((RobotHandle)robot_handle, current_joint) != sucess ||
                !std::all_of(current_joint, current_joint + 6,
                             [](double value) { return std::isfinite(value); }) ||
                // candidateMaximumJointStep uses remainder(delta, 360) for J6,
                // matching moveToJointPos's nearest-equivalent-angle normalization.
                candidateMaximumJointStep(current_joint, candidates[i].joint) >
                    maximum_joint_step_degrees) {
                error = "运动前实际关节读取失败或关节跳变超限";
                return -2;
            }
            const CRresult ret = moveToJointPos(robot_handle, candidates[i].joint);
            if (ret != sucess) {
                if (ret == robot_motion_cancelled)
                    return -1;
                error = "机器人运动失败: " + std::to_string(ret);
                return -2;
            }
            if (robotMotionCancellationRequested()) return -1;
            if (!candidates[i].capture) continue;
            if (!waitForSettling()) return -1;
            // Drain stale frames after the robot has settled.
            for (int discard = 0; discard < 2; ++discard) {
                if (robotMotionCancellationRequested()) return -1;
                camera->captureOneFrame(1500);
            }
            cv::Mat frame;
            bool detected = false;
            double flange_aa[6]{}, actual_joint[6]{}, euler[6]{};
            for (int attempt = 0; attempt < 3; ++attempt) {
                if (robotMotionCancellationRequested()) return -1;
                double before_flange[6]{}, after_flange[6]{}, after_joint[6]{};
                if (!readActualFlangeAxisAngle(robot_handle, before_flange)) {
                    error = "拍照前读取实际法兰位姿失败";
                    return -2;
                }
                frame = camera->captureOneFrame(1500);
                if (robotMotionCancellationRequested()) return -1;
                if (!readActualFlangeAxisAngle(robot_handle, after_flange) ||
                    cr_get_jointActualPos((RobotHandle)robot_handle, after_joint) != sucess ||
                    !std::all_of(after_joint, after_joint + 6,
                                 [](double value) { return std::isfinite(value); })) {
                    error = "拍照后读取实际机器人位姿失败";
                    return -2;
                }
                const RigidTransformDistance capture_drift = rigidTransformDistance(
                    axisAnglePoseToTransform(before_flange),
                    axisAnglePoseToTransform(after_flange));
                if (!std::isfinite(capture_drift.translation_mm) ||
                    !std::isfinite(capture_drift.rotation_deg) ||
                    capture_drift.translation_mm > maximum_capture_translation_drift_mm ||
                    capture_drift.rotation_deg > maximum_capture_rotation_drift_degrees) {
                    ++unstable_frames;
                    std::cout << "[eyeToHand] 拍照期间机器人位姿不稳定，重拍: Δt="
                              << capture_drift.translation_mm << " mm, ΔR="
                              << capture_drift.rotation_deg << " deg" << std::endl;
                    continue;
                }
                if (frame.empty()) {
                    ++empty_frames;
                    continue;
                }
                std::vector<cv::Point2f> corners;
                std::vector<int> ids;
                if (detector.detectCorners(frame, corners, ids) &&
                    ids.size() == corners.size() &&
                    static_cast<int>(ids.size()) >= std::max(12, min_corners)) {
                    std::copy(after_flange, after_flange + 6, flange_aa);
                    std::copy(after_joint, after_joint + 6, actual_joint);
                    if (cr_AxisAngle2Eule(flange_aa, euler) != sucess ||
                        !std::all_of(euler, euler + 6,
                                     [](double value) { return std::isfinite(value); })) {
                        error = "法兰轴角转欧拉位姿失败";
                        return -2;
                    }
                    detected = true;
                    break;
                }
                ++rejected_frames;
            }
            if (!detected) {
                ++missed_candidates;
                continue;
            }
            // Preserve TCP -> flange semantics; use the measured pose immediately
            // after the exposure only when the bracketing poses prove stability.
            for (const double* values : {flange_aa, actual_joint, euler}) {
                if (!std::all_of(values, values + 6,
                                 [](double value) { return std::isfinite(value); })) {
                    error = "实际关节/法兰位姿包含非有限值";
                    return -2;
                }
            }
            if (robotMotionCancellationRequested()) return -1;
            const std::string path = imagePath(save_dir, captured);
            if (!cv::imwrite(path, frame))
                throw std::runtime_error("图像写入失败: " + path);
            for (int axis = 0; axis < 6; ++axis) {
                poses << flange_aa[axis] << (axis < 5 ? "," : "\n");
                joints << actual_joint[axis] << (axis < 5 ? "," : "\n");
                eulers << euler[axis] << (axis < 5 ? "," : "\n");
            }
            poses.flush();
            joints.flush();
            eulers.flush();
            robot_poses.push_back(axisAnglePoseToTransform(flange_aa));
            ++captured;
            std::cout << "[eyeToHand] 采集进度: " << i + 1 << "/" << total
                      << " (有效: " << captured << ")" << std::endl;
            // Only announce frames once their corresponding rows are written.
            if (callback && callback(const_cast<char*>(path.c_str()),
                                     static_cast<int>(path.size()),
                                     (i + 1) * 100 / total) == -1)
                return -1;
        }
        poses.close();
        joints.close();
        eulers.close();
    } catch (const std::exception& exception) {
        error = "采集失败: " + std::string(exception.what());
        return -2;
    }
    error = "采集统计: 有效=" + std::to_string(captured) +
            ", 未检出合格标定板候选=" + std::to_string(missed_candidates) +
            ", 空帧=" + std::to_string(empty_frames) +
            ", 拍照期间位姿不稳定=" + std::to_string(unstable_frames) +
            ", 检测失败或角点不足=" + std::to_string(rejected_frames);
    std::cout << "[eyeToHand] " << error << std::endl;
    return captured;
}

// Strict row parsing prevents a malformed CSV row from shifting every later image.
static bool loadMatchedRobotPoses(const std::string& directory,
                                 std::vector<cv::Mat>& poses,
                                 std::string& error) {
    namespace fs = std::filesystem;
    const bool axis_angle = fs::exists(fs::path(directory) / "rob_pos.csv");
    const auto path = fs::path(directory) / (axis_angle ? "rob_pos.csv" : "rob_eule.csv");
    std::ifstream input(path);
    if (!input) {
        error = "无法读取机器人位姿: " + path.string();
        return false;
    }
    poses.clear();
    std::string line;
    while (std::getline(input, line)) {
        std::istringstream row(line);
        double values[6]{};
        for (int axis = 0; axis < 6; ++axis) {
            char comma = 0;
            if (!(row >> values[axis]) || !std::isfinite(values[axis]) ||
                (axis < 5 && (!(row >> comma) || comma != ','))) {
                error = "机器人位姿 CSV 格式错误，第 " + std::to_string(poses.size() + 1) + " 行";
                return false;
            }
        }
        row >> std::ws;
        if (!row.eof()) {
            error = "机器人位姿 CSV 存在多余字段，第 " + std::to_string(poses.size() + 1) + " 行";
            return false;
        }
        double axis_pose[6]{};
        if (axis_angle) {
            std::copy(values, values + 6, axis_pose);
        } else if (cr_Eule2AxisAngle(values, axis_pose) != sucess) {
            error = "机器人欧拉位姿转换失败";
            return false;
        }
        poses.push_back(axisAnglePoseToTransform(axis_pose));
    }
    if (input.bad() || poses.empty()) {
        error = "机器人位姿为空或读取失败";
        return false;
    }
    return true;
}

static bool validateImagePoseMatching(const std::string& directory,
                                     std::size_t pose_count, std::string& error) {
    namespace fs = std::filesystem;
    if (fs::exists(fs::path(directory) / ".collection_incomplete")) {
        error = "采集未完整写入，拒绝重放: " + directory;
        return false;
    }
    std::set<std::string> expected;
    for (std::size_t i = 0; i < pose_count; ++i)
        expected.insert(fs::path(imagePath(directory, i)).filename().string());
    for (const auto& entry : fs::directory_iterator(directory)) {
        std::string extension = entry.path().extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(),
                       [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
        if (extension != ".jpg" && extension != ".jpeg" && extension != ".png" &&
            extension != ".bmp" && extension != ".tif" && extension != ".tiff")
            continue;
        if (!entry.is_regular_file() || expected.erase(entry.path().filename().string()) != 1) {
            error = "存在多余或非标准编号图像: " + entry.path().string();
            return false;
        }
    }
    if (pose_count == 0 || !expected.empty()) {
        error = expected.empty() ? "没有机器人位姿" : "缺少匹配图像: " + *expected.begin();
        return false;
    }
    return true;
}

static bool computeBoardPosesFromImages(
    const std::string& directory, CharucoDetector& detector,
    const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
    std::vector<cv::Mat>& camera_poses, std::vector<cv::Mat>& robot_poses,
    std::string& error) {
    camera_poses.clear();
    if (!validateImagePoseMatching(directory, robot_poses.size(), error)) return false;
    detector.setCameraParams(camera_matrix, dist_coeffs);
    std::vector<cv::Mat> matched_robot, matched_camera;
    for (std::size_t i = 0; i < robot_poses.size(); ++i) {
        const auto path = imagePath(directory, i);
        const cv::Mat image = cv::imread(path);
        if (image.empty()) {
            error = "无法读取匹配图像: " + path;
            return false;
        }
        cv::Mat rvec, tvec;
        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detector.detectPose(image, rvec, tvec, corners, ids, 12) ||
            ids.size() < 12 || ids.size() != corners.size()) {
            std::cout << "[eyeToHand] PnP失败或角点不足12，跳过配对帧: " << path << std::endl;
            continue;
        }
        matched_robot.push_back(robot_poses[i]);
        matched_camera.push_back(rvecTvecToTransform(rvec, tvec));
    }
    robot_poses.swap(matched_robot);
    camera_poses.swap(matched_camera);
    return true;
}

static bool saveEyeToHandResult(const std::string& directory, EyeToHandResult& result,
                                const std::string& output_yaml = "") {
    const std::string path = output_yaml.empty()
        ? directory + "/eye_to_hand_result.yaml"
        : output_yaml;
    if (!saveCalibrationResult(path, "T_base_camera", result.T_base_camera,
                               result.camera_matrix, result.dist_coeffs,
                               result.reproj_error * 1000.0, result.valid_frames)) {
        result.success = false;
        result.message = "标定结果 YAML 写入失败: " + path;
        return false;
    }
    return true;
}

bool autoEyeToHandCharucoCalibration(
    int robot_handle, char* pchRobotIP, std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector, const PoseGenerationConfig& pose_config,
    EyeToHandResult& result, const std::string& save_dir,
    AI_det_callback callback, const std::string& output_yaml) {
    result = EyeToHandResult{};
    (void)pchRobotIP;
    if (pose_config.min_corners <= 0 || pose_config.min_valid_frames <= 0) {
        result.message = "min_corners and min_valid_frames must be positive";
        return false;
    }
    const int minimum_frames = std::max(12, pose_config.min_valid_frames);
    try {
        if (robotMotionCancellationRequested()) {
            result.message = "标定已取消";
            return false;
        }
        if (!camera || save_dir.empty()) {
            result.message = "相机未初始化或保存目录为空";
            return false;
        }
        for (const auto* offsets : {&pose_config.offset_x, &pose_config.offset_y,
                                   &pose_config.offset_z, &pose_config.offset_rx,
                                   &pose_config.offset_ry, &pose_config.offset_rz}) {
            if (offsets->empty() || !std::all_of(offsets->begin(), offsets->end(),
                                  [](double value) { return std::isfinite(value); })) {
                result.message = "候选偏移必须非空且为有限值";
                return false;
            }
        }
        namespace fs = std::filesystem;
        const auto parent = fs::absolute(save_dir);
        fs::create_directories(parent);
        const auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        // Atomic directory creation also handles simultaneous runs and clock collisions.
        for (unsigned int suffix = 0; ; ++suffix) {
            const auto path = parent / ("run_" + std::to_string(stamp) + "_" + std::to_string(suffix));
            if (fs::create_directory(path)) {
                result.data_directory = path.string();
                break;
            }
        }
        const std::string directory = result.data_directory;
        std::cout << "[eyeToHand] 本次数据目录: " << directory << std::endl;
        const auto marker = fs::path(directory) / ".collection_incomplete";
        {
            std::ofstream incomplete(marker);
            incomplete.exceptions(std::ios::failbit | std::ios::badbit);
            incomplete << "Collection has not completed successfully.\n";
            incomplete.close();
        }
        double tcp_aa[6]{}, flange_euler[6]{};
        if (cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa) != sucess ||
            !convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flange_euler)) {
            result.message = "获取示教原点法兰位姿失败";
            return false;
        }
        std::vector<PoseCandidate> candidates;
        if (!generateEyeToHandCandidates(robot_handle, flange_euler, pose_config,
                                         candidates, result.message)) {
            if (result.message.empty()) result.message = "无 IK 可达位姿";
            return false;
        }
        std::vector<cv::Mat> robot_poses;
        const int captured = executeEyeToHandCollection(
            robot_handle, camera, detector, candidates, directory, robot_poses,
            pose_config.min_corners, pose_config.maximum_joint_step_degrees,
            pose_config.maximum_capture_translation_drift_mm,
            pose_config.maximum_capture_rotation_drift_degrees,
            result.message, callback);
        if (captured == -2) return false; // Preserve the concrete failure, even if cancellation follows.
        if (captured == -1) {
            result.message = "标定已取消";
            return false;
        }
        if (!fs::remove(marker)) {
            result.message = "无法完成采集写入标记";
            return false;
        }
        if (robotMotionCancellationRequested()) {
            result.message = "标定已取消";
            return false;
        }
        if (captured < minimum_frames) {
            result.message = "有效帧数不足: " + std::to_string(captured) +
                " (need >= " + std::to_string(minimum_frames) + ")" +
                (result.message.empty() ? "" : "; " + result.message);
            return false;
        }
        if (!validateImagePoseMatching(directory, robot_poses.size(), result.message))
            return false;
        const bool use_fixed_intrinsics =
            !detector.getCameraMatrix().empty() && !detector.getDistCoeffs().empty();
        if (use_fixed_intrinsics) {
            result.camera_matrix = detector.getCameraMatrix().clone();
            result.dist_coeffs = detector.getDistCoeffs().clone();
            result.reused_intrinsics = true;
            std::cout << "[eyeToHand] 使用身份校验通过的固定相机内参" << std::endl;
        } else {
            result.reused_intrinsics = false;
            if (!calibrateCameraFromDisk(directory, detector,
                                         result.camera_matrix, result.dist_coeffs)) {
                result.message = "相机标定失败";
                return false;
            }
        }
        // Use a checked stream for actual disk writes; FileStorage serializes in memory.
        {
            cv::FileStorage yaml("", cv::FileStorage::WRITE |
                                     cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_YAML);
            yaml << "camera_matrix" << result.camera_matrix;
            yaml << "distortion_coefficients" << result.dist_coeffs;
            std::ofstream output(directory + "/camera_params.yaml");
            output.exceptions(std::ios::failbit | std::ios::badbit);
            output << yaml.releaseAndGetString();
            output.close();
        }
        std::vector<cv::Mat> camera_poses;
        if (!computeBoardPosesFromImages(directory, detector, result.camera_matrix,
                                        result.dist_coeffs, camera_poses, robot_poses,
                                        result.message))
            return false;
        if (robotMotionCancellationRequested()) {
            result.message = "标定已取消";
            return false;
        }
        if (robot_poses.size() < static_cast<std::size_t>(minimum_frames)) {
            result.message = "PnP有效配对帧数不足: " + std::to_string(robot_poses.size()) +
                             " (need >= " + std::to_string(minimum_frames) + ")";
            return false;
        }
        // Wrapper returns T_base_camera directly and populates the shared quality report.
        if (!solveEyeToHandPoses(robot_poses, camera_poses, result)) return false;
        // Publication cutoff: honor cancellation through this pre-save check,
        // including requests received during the solve. Once saving starts,
        // let the checked write finish; a later cancellation does not retract it.
        if (robotMotionCancellationRequested()) {
            result.success = false;
            result.T_base_camera.release();
            result.message = "标定已取消";
            return false;
        }
        if (output_yaml.empty())
            return true;

        fs::path reference_path = pose_config.reference_result_file;
        if (reference_path.is_relative()) {
            reference_path = fs::path(pose_config.source_file).parent_path() /
                             reference_path;
        }
        EyeToHandPublicationCheck publication;
        std::string publication_error;
        if (!publishEyeToHandResult(
                result, reference_path.lexically_normal().string(), output_yaml,
                pose_config, publication, publication_error)) {
            result.success = false;
            result.message = publication_error +
                "; 候选数据保留在 " + result.data_directory;
            return false;
        }
        result.message += "; " + publication.message;
        return true;
    } catch (const std::exception& exception) {
        result.success = false;
        result.T_base_camera.release();
        result.message = "自动眼在手外标定失败: " + std::string(exception.what());
        return false;
    }
}

bool calibrateEyeToHandFromDisk(const std::string& save_dir, EyeToHandResult& result,
                                const std::string& camera_params_file,
                                const std::string& output_yaml) {
    result = EyeToHandResult{};
    try {
        if (save_dir.empty()) {
            result.message = "数据目录为空";
            return false;
        }
        result.data_directory = std::filesystem::absolute(save_dir).string();
        std::vector<cv::Mat> robot_poses;
        if (!loadMatchedRobotPoses(save_dir, robot_poses, result.message) ||
            !validateImagePoseMatching(save_dir, robot_poses.size(), result.message))
            return false;
        if (robot_poses.size() < 12) {
            result.message = "有效配对帧少于12";
            return false;
        }
        CharucoBoardConfig board_config;
        CharucoDetector detector(board_config);
        if (!camera_params_file.empty()) {
            cv::FileStorage storage(camera_params_file, cv::FileStorage::READ);
            if (!storage.isOpened()) {
                result.message = "无法打开相机内参: " + camera_params_file;
                return false;
            }
            storage["camera_matrix"] >> result.camera_matrix;
            storage["distortion_coefficients"] >> result.dist_coeffs;
            storage.release();
            if (!result.camera_matrix.empty() && result.camera_matrix.depth() != CV_64F)
                result.camera_matrix.convertTo(result.camera_matrix, CV_64F);
            if (!result.dist_coeffs.empty() && result.dist_coeffs.depth() != CV_64F)
                result.dist_coeffs.convertTo(result.dist_coeffs, CV_64F);
            if (result.camera_matrix.rows != 3 || result.camera_matrix.cols != 3 ||
                result.dist_coeffs.empty() ||
                !cv::checkRange(result.camera_matrix) || !cv::checkRange(result.dist_coeffs) ||
                result.camera_matrix.at<double>(0, 0) <= 0.0 ||
                result.camera_matrix.at<double>(1, 1) <= 0.0) {
                result.camera_matrix.release();
                result.dist_coeffs.release();
                result.message = "相机内参格式无效: " + camera_params_file;
                return false;
            }
            std::cout << "[eyeToHand] 使用固定相机内参: "
                      << camera_params_file << std::endl;
        } else {
            if (!calibrateCameraFromDisk(save_dir, detector,
                                         result.camera_matrix, result.dist_coeffs)) {
                result.message = "相机标定失败";
                return false;
            }
        }
        std::vector<cv::Mat> camera_poses;
        if (!computeBoardPosesFromImages(save_dir, detector, result.camera_matrix,
                                        result.dist_coeffs, camera_poses, robot_poses,
                                        result.message))
            return false;
        if (!solveEyeToHandPoses(robot_poses, camera_poses, result)) return false;
        return saveEyeToHandResult(save_dir, result, output_yaml);
    } catch (const std::exception& exception) {
        result.success = false;
        result.T_base_camera.release();
        result.message = "离线眼在手外标定失败: " + std::string(exception.what());
        return false;
    }
}
