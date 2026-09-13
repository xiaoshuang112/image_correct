#include "semi_auto_eye_in_hand_calibration.hpp"

#include "camera_calibration.hpp"           // calibrateCameraFromDisk, loadImagesFromDisk
#include "calibration_utils.hpp"            // executeCollection, PoseCandidate
#include "calibration_session.hpp"
#include "hand_eye_gui/DiagnosticLogger.hpp"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <filesystem>
#include <future>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <thread>

#include "utils/robot_util.hpp"
#include "utils/calib_transform.hpp"

// ============================================================
// 内部函数
// ============================================================
namespace {

// Phase 2: 位姿生成 + IK 过滤
// 与 generateEyeToHandCandidates 结构相同，输出 std::vector<PoseCandidate>
static bool generateEyeInHandCandidates(
    int robot_handle,
    const double flangeEuleInit[6],
    const PoseGenerationConfig& config,
    std::vector<PoseCandidate>& candidates,
    CandidatePlanningReport& planning_report,
    std::string& error) {

    candidates.clear();
    error.clear();
    double initial_joint[6]{};
    const CRresult joint_ret = cr_get_jointActualPos(
        (RobotHandle)robot_handle, initial_joint);
    if (joint_ret != sucess) {
        error = "读取候选规划初始关节角失败: " + std::to_string(joint_ret);
        return false;
    }

    double euler[6]{}, axis_angle[6]{}, joint[6]{};
    int source_index = 0;
    int euler_failures = 0;
    int ik_failures = 0;

    for (double dx : config.offset_x) {
    for (double dy : config.offset_y) {
    for (double dz : config.offset_z) {
    for (double drx : config.offset_rx) {
    for (double dry : config.offset_ry) {
    for (double drz : config.offset_rz) {
        euler[0] = flangeEuleInit[0] + dx;
        euler[1] = flangeEuleInit[1] + dy;
        euler[2] = flangeEuleInit[2] + dz;
        euler[3] = flangeEuleInit[3] + drx;
        euler[4] = flangeEuleInit[4] + dry;
        euler[5] = flangeEuleInit[5] + drz;

        const CRresult euler_ret = cr_Eule2AxisAngle(euler, axis_angle);
        if (euler_ret != sucess) {
            ++euler_failures;
            ++source_index;
            continue;
        }
        const CRresult ik_ret = cr_kineInverse((RobotHandle)robot_handle,
                                               axis_angle, initial_joint, joint);
        if (ik_ret == sucess) {
            PoseCandidate cand{};
            std::copy(joint, joint + 6, cand.joint);
            std::copy(axis_angle, axis_angle + 6, cand.axis_angle);
            std::copy(euler, euler + 6, cand.flange_euler);
            cand.source_index = source_index;
            candidates.push_back(cand);
        } else {
            ++ik_failures;
        }
        ++source_index;
    }}}}}}

    const std::size_t reachable = candidates.size();
    CandidatePlanningOptions options;
    options.maximum_candidates = static_cast<std::size_t>(config.maximum_candidates);
    options.maximum_joint_step_degrees = config.maximum_joint_step_degrees;
    options.joint_resolver = [robot_handle](
        const PoseCandidate& candidate,
        const std::array<double, 6>& reference,
        std::array<double, 6>& resolved,
        int& error_code) {
        double target[6]{};
        double reference_joint[6]{};
        std::copy(candidate.axis_angle, candidate.axis_angle + 6, target);
        std::copy(reference.begin(), reference.end(), reference_joint);
        const CRresult result = cr_kineInverse((RobotHandle)robot_handle,
                                               target, reference_joint,
                                               resolved.data());
        error_code = static_cast<int>(result);
        return result == sucess;
    };
    if (!planPoseCandidates(candidates, initial_joint, options,
                            planning_report, error)) {
        return false;
    }

    std::cout << "[semiAutoEyeInHand] 位姿生成: planned=" << source_index
              << ", IK可达=" << reachable
              << ", IK失败=" << ik_failures
              << ", 欧拉转换失败=" << euler_failures
              << ", 选取=" << planning_report.selected_candidates
              << ", 过渡=" << planning_report.transition_candidates
              << ", 安全连通=" << planning_report.connected_candidates
              << ", 不连通=" << planning_report.disconnected_candidates
              << ", 最大关节跳变=" << std::fixed << std::setprecision(3)
              << planning_report.maximum_joint_step_degrees << " deg"
              << std::endl;

    return !candidates.empty();
}

static std::string formatSixValues(const double values[6]) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4) << "[";
    for (int axis = 0; axis < 6; ++axis) {
        if (axis > 0)
            stream << ",";
        stream << values[axis];
    }
    stream << "]";
    return stream.str();
}

// 统计 save_dir 下 .jpg 文件数
static int countCapturedFrames(const std::string& dir) {
    namespace fs = std::filesystem;
    int count = 0;
    if (!fs::exists(dir)) return 0;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() == ".jpg") count++;
    }
    return count;
}

static std::string auditBaseDirectoryForSource(const std::string& source) {
    namespace fs = std::filesystem;
    const fs::path sourcePath(source);
    if (fs::exists(sourcePath / "session.yaml") &&
        sourcePath.parent_path().filename() == "sessions") {
        return sourcePath.parent_path().parent_path().string();
    }
    return source;
}

static void populateSourceSessionLineage(const std::string& source,
                                         CalibrationSessionOptions& options) {
    namespace fs = std::filesystem;
    const fs::path sourcePath(source);
    const fs::path manifest = sourcePath / "manifest.sha256";
    const fs::path session = sourcePath / "session.yaml";
    if (!fs::exists(manifest) || !fs::exists(session))
        return;
    options.source_manifest_sha256 = CalibrationSession::sha256File(manifest.string());
    try {
        cv::FileStorage storage(session.string(), cv::FileStorage::READ);
        if (storage.isOpened())
            storage["session_id"] >> options.source_session_id;
    } catch (const cv::Exception&) {
        options.source_session_id.clear();
    }
    if (options.source_session_id.empty())
        options.source_session_id = sourcePath.filename().string();
}

} // anonymous namespace

// ============================================================
// 在线标定入口
// ============================================================
bool semiAutoEyeInHandCalibration(
    int robot_handle,
    char* pchRobotIP,
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    const PoseGenerationConfig& pose_config,
    const CharucoBoardConfig& board_config,
    cv::InputOutputArray camera_matrix,
    cv::InputOutputArray dist_coeffs,
    CalibrationResult& result,
    const std::string& save_dir,
    const std::string& output_yaml,
    AI_det_callback callback) {

    std::cout << "\n========== 半自动眼在手内 ChArUco 标定 ==========" << std::endl;

    result = CalibrationResult{};
    CalibrationSessionOptions session_options;
    session_options.base_directory = auditBaseDirectoryForSource(save_dir);
    session_options.mode = "online_semi_auto_eye_in_hand";
    session_options.source_data_directory = "live_capture";
    session_options.board = board_config;
    session_options.robot_endpoint = pchRobotIP ? pchRobotIP : "";
    session_options.pose_config_source_file = pose_config.source_file;
    session_options.pose_offset_x = pose_config.offset_x;
    session_options.pose_offset_y = pose_config.offset_y;
    session_options.pose_offset_z = pose_config.offset_z;
    session_options.pose_offset_rx = pose_config.offset_rx;
    session_options.pose_offset_ry = pose_config.offset_ry;
    session_options.pose_offset_rz = pose_config.offset_rz;
    session_options.pose_min_corners = pose_config.min_corners;
    session_options.pose_min_valid_frames = pose_config.min_valid_frames;
    session_options.pose_maximum_candidates = pose_config.maximum_candidates;
    session_options.pose_maximum_joint_step_degrees =
        pose_config.maximum_joint_step_degrees;
    if (camera) {
        const CONFIG& config = camera->getConfig();
        const CameraDeviceAuditInfo deviceInfo = camera->getDeviceAuditInfo();
        session_options.camera_serial = deviceInfo.serial;
        session_options.camera_model = deviceInfo.model;
        session_options.camera_firmware = deviceInfo.firmware;
        session_options.camera_endpoint = config.cam_ip + ":" +
            std::to_string(config.cam_port);
        session_options.image_width = config.cam_width;
        session_options.image_height = config.cam_height;
    }
    const bool provided_intrinsics = !camera_matrix.empty() && !dist_coeffs.empty();
    session_options.intrinsic_source = provided_intrinsics
        ? "provided_runtime" : "calibrated_from_collection";
    if (provided_intrinsics) {
        session_options.camera_matrix = camera_matrix.getMat().clone();
        session_options.dist_coeffs = dist_coeffs.getMat().clone();
    }
    std::string session_error;
    std::unique_ptr<CalibrationSession> session =
        CalibrationSession::create(session_options, session_error);
    if (!session) {
        result.message = "无法创建标定审计会话: " + session_error;
        return false;
    }
    result.session_dir = session->directory();
    auto failSession = [&](const std::string& stage,
                           const std::string& reason) -> bool {
        session->finalizeFailure(stage, reason, result.message);
        return false;
    };

    // ---- Phase 1: 获取示教原点 ----
    session->setStage("prepare");
    double tcp_aa[6], flangeEuleInit[6];
    if (cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa) != sucess) {
        result.message = "获取机器人位姿失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return failSession("prepare", "robot_pose_read_failed");
    }
    if (!convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEuleInit)) {
        result.message = "轴角转法兰欧拉失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return failSession("prepare", "flange_pose_conversion_failed");
    }
    std::cout << "[semiAutoEyeInHand] 示教原点 (法兰欧拉): "
              << flangeEuleInit[0] << ", " << flangeEuleInit[1] << ", " << flangeEuleInit[2] << " mm, "
              << flangeEuleInit[3] << ", " << flangeEuleInit[4] << ", " << flangeEuleInit[5] << " deg"
              << std::endl;

    // ---- Phase 2: 生成候选位姿 ----
    std::vector<PoseCandidate> candidates;
    CandidatePlanningReport planning_report;
    std::string planning_error;
    if (!generateEyeInHandCandidates(robot_handle, flangeEuleInit, pose_config,
                                     candidates, planning_report,
                                     planning_error)) {
        result.message = "候选位姿规划失败: " + planning_error;
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return failSession("pose_generation", "candidate_planning_failed");
    }
    session->appendEvent(
        "candidate_plan", "success",
        "input=" + std::to_string(planning_report.input_candidates) +
        " connected=" + std::to_string(planning_report.connected_candidates) +
        " disconnected=" + std::to_string(
            planning_report.disconnected_candidates) +
        " selected=" + std::to_string(planning_report.selected_candidates) +
        " ordered=" + std::to_string(planning_report.ordered_candidates) +
        " transitions=" + std::to_string(
            planning_report.transition_candidates) +
        " path_deg=" + std::to_string(planning_report.estimated_path_degrees) +
        " max_step_deg=" + std::to_string(
            planning_report.maximum_joint_step_degrees) +
        " max_step_axis=J" + std::to_string(
            planning_report.maximum_step_axis + 1) +
        " max_axis_steps=" + formatSixValues(
            planning_report.maximum_axis_step_degrees.data()) +
        " max_j6_step_deg=" + std::to_string(
            planning_report.maximum_j6_step_degrees));
    for (std::size_t ordered_index = 0; ordered_index < candidates.size();
         ++ordered_index) {
        const PoseCandidate& candidate = candidates[ordered_index];
        session->appendEvent(
            "candidate_plan_item", "ready",
            "ordered=" + std::to_string(ordered_index) +
            " from_source=" + std::to_string(
                candidate.previous_source_index) +
            " source=" + std::to_string(candidate.source_index) +
            " role=" + std::string(
                candidate.capture ? "capture" : "transition") +
            " axis_steps_deg=" + formatSixValues(
                candidate.step_from_previous) +
            " joint=" + formatSixValues(candidate.joint) +
            " flange_euler=" + formatSixValues(candidate.flange_euler));
    }

    // ---- Phase 3: 逐点采集 ----
    session->setStage("collection");
    if (!executeCollection(robot_handle, pchRobotIP, camera, detector,
                            candidates, session->framesDirectory(), callback,
                            session.get(), session->robotPosesDirectory())) {
        result.message = "采集失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return failSession("collection", "collection_failed");
    }

    {
        int captured = countCapturedFrames(session->framesDirectory());
        if (captured < pose_config.min_valid_frames) {
            result.message = "有效帧数不足: " + std::to_string(captured) +
                             " (need >= " + std::to_string(pose_config.min_valid_frames) + ")";
            std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
            return failSession("collection", "insufficient_captured_frames");
        }
        std::cout << "[semiAutoEyeInHand] 有效帧: " << captured << std::endl;
    }

    // ---- Phase 4: 相机内参标定 ----
    cv::Mat cam_mat, dist;
    if (!camera_matrix.empty()) {
        cam_mat = camera_matrix.getMat().clone();
        dist = dist_coeffs.getMat().clone();
        std::cout << "[semiAutoEyeInHand] 使用已知相机内参" << std::endl;
    } else {
        if (!calibrateCameraFromDisk(session->framesDirectory(), detector, cam_mat, dist)) {
            result.message = "相机内参标定失败";
            std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
            return failSession("intrinsic_calibration", "intrinsic_calibration_failed");
        }
        std::cout << "[semiAutoEyeInHand] 相机内参已标定" << std::endl;
    }
    session->setIntrinsics(cam_mat, dist, provided_intrinsics
        ? "provided_runtime" : "calibrated_from_collection");

    // ---- Phase 5: 手眼标定 ----
    if (!calibrateHandEyeFromDisk(session->framesDirectory(), cam_mat, dist,
                                   board_config, result, output_yaml,
                                   session.get())) {
        if (result.message.empty())
            result.message = "手眼标定失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return false;
    }

    // 输出回传
    cam_mat.copyTo(camera_matrix);
    dist.copyTo(dist_coeffs);

    return true;
}

// ============================================================
// 离线标定入口
// ============================================================
bool calibrateEyeInHandFromDisk(
    const std::string& save_dir,
    const CharucoBoardConfig& board_config,
    cv::InputOutputArray camera_matrix,
    cv::InputOutputArray dist_coeffs,
    CalibrationResult& result,
    const std::string& output_yaml) {

    result = CalibrationResult{};
    std::cout << "\n========== 半自动眼在手内离线标定 ==========" << std::endl;
    std::cout << "[semiAutoEyeInHand] 数据目录: " << save_dir << std::endl;

    CharucoDetector detector(board_config);
    CalibrationSessionOptions session_options;
    session_options.base_directory = save_dir;
    session_options.mode = "offline_eye_in_hand_with_intrinsic_calibration";
    session_options.source_data_directory = save_dir;
    session_options.intrinsic_source = (!camera_matrix.empty() && !dist_coeffs.empty())
        ? "provided_offline" : "calibrated_from_collection";
    session_options.board = board_config;
    if (!camera_matrix.empty() && !dist_coeffs.empty()) {
        session_options.camera_matrix = camera_matrix.getMat().clone();
        session_options.dist_coeffs = dist_coeffs.getMat().clone();
    }
    std::string session_error;
    std::unique_ptr<CalibrationSession> session =
        CalibrationSession::create(session_options, session_error);
    if (!session) {
        result.message = "无法创建标定审计会话: " + session_error;
        return false;
    }
    result.session_dir = session->directory();
    if (!session->importLegacyDataset(save_dir, session_error)) {
        result.message = "无法导入离线数据集: " + session_error;
        session->finalizeFailure("dataset_import", "dataset_import_failed", result.message);
        return false;
    }
    auto failSession = [&](const std::string& stage,
                           const std::string& reason) -> bool {
        session->finalizeFailure(stage, reason, result.message);
        return false;
    };

    // ---- Phase 1: 相机内参标定 ----
    cv::Mat cam_mat, dist;
    if (!camera_matrix.empty() && !dist_coeffs.empty()) {
        cam_mat = camera_matrix.getMat().clone();
        dist = dist_coeffs.getMat().clone();
        std::cout << "[semiAutoEyeInHand] 使用已知相机内参" << std::endl;
    } else {
        session->setStage("intrinsic_calibration");
        if (!calibrateCameraFromDisk(session->framesDirectory(), detector, cam_mat, dist)) {
            result.message = "相机内参标定失败";
            std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
            return failSession("intrinsic_calibration", "intrinsic_calibration_failed");
        }
        std::cout << "[semiAutoEyeInHand] 相机内参已标定" << std::endl;
    }
    session->setIntrinsics(cam_mat, dist,
        (!camera_matrix.empty() && !dist_coeffs.empty())
            ? "provided_offline" : "calibrated_from_collection");

    // ---- Phase 2: 手眼标定 ----
    if (!calibrateHandEyeFromDisk(session->framesDirectory(), cam_mat, dist,
                                  board_config, result, output_yaml,
                                  session.get())) {
        if (result.message.empty())
            result.message = "手眼标定失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return false;
    }

    // 输出回传
    cam_mat.copyTo(camera_matrix);
    dist.copyTo(dist_coeffs);

    return true;
}

// ============================================================
// 手眼标定精化：Gauss-Newton 最小化基座标系下板位姿平移方差
// ============================================================
namespace {

static cv::Mat refineHandEyePoseConsistency(
    const cv::Mat& X_init,
    const std::vector<cv::Mat>& robot_poses,
    const std::vector<cv::Mat>& camera_poses,
    bool emit_diagnostics = true) {

    int n = (int)robot_poses.size();
    if (n < 3) return X_init.clone();

    cv::Mat p(6, 1, CV_64F);
    cv::Mat R_init = X_init(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat rvec_init;
    cv::Rodrigues(R_init, rvec_init);
    p.at<double>(0) = X_init.at<double>(0, 3);
    p.at<double>(1) = X_init.at<double>(1, 3);
    p.at<double>(2) = X_init.at<double>(2, 3);
    p.at<double>(3) = rvec_init.at<double>(0);
    p.at<double>(4) = rvec_init.at<double>(1);
    p.at<double>(5) = rvec_init.at<double>(2);

    auto buildX = [](const cv::Mat& params) -> cv::Mat {
        cv::Mat R(3, 3, CV_64F);
        cv::Mat rv(3, 1, CV_64F);
        rv.at<double>(0) = params.at<double>(3);
        rv.at<double>(1) = params.at<double>(4);
        rv.at<double>(2) = params.at<double>(5);
        cv::Rodrigues(rv, R);
        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(T(cv::Rect(0, 0, 3, 3)));
        T.at<double>(0, 3) = params.at<double>(0);
        T.at<double>(1, 3) = params.at<double>(1);
        T.at<double>(2, 3) = params.at<double>(2);
        return T;
    };

    auto eval = [&](const cv::Mat& X,
                    std::vector<cv::Mat>& residuals,
                    cv::Mat& mean_t) -> double {
        std::vector<cv::Mat> tvecs(n);
        mean_t = cv::Mat::zeros(3, 1, CV_64F);
        for (int i = 0; i < n; i++) {
            cv::Mat T = robot_poses[i] * X * camera_poses[i];
            tvecs[i] = T(cv::Rect(3, 0, 1, 3)).clone();
            mean_t += tvecs[i];
        }
        mean_t /= n;
        double cost = 0.0;
        residuals.resize(n);
        for (int i = 0; i < n; i++) {
            residuals[i] = tvecs[i] - mean_t;
            cost += residuals[i].dot(residuals[i]);
        }
        return cost;
    };

    const double eps_step = 1e-8;
    const int max_iters = 100;
    double lambda = 1e-6;
    cv::Mat X = buildX(p);
    std::vector<cv::Mat> res;
    cv::Mat mean_t;
    double prev_cost = eval(X, res, mean_t);
    if (emit_diagnostics)
        std::cout << "[refine] 初始方差: " << (prev_cost * 1e6) << " mm²" << std::endl;

    for (int iter = 0; iter < max_iters; iter++) {
        cv::Mat R_full(3 * n, 1, CV_64F);
        for (int i = 0; i < n; i++) {
            R_full.at<double>(3 * i,     0) = res[i].at<double>(0);
            R_full.at<double>(3 * i + 1, 0) = res[i].at<double>(1);
            R_full.at<double>(3 * i + 2, 0) = res[i].at<double>(2);
        }

        cv::Mat J_full(3 * n, 6, CV_64F);
        for (int j = 0; j < 6; j++) {
            cv::Mat p_pert = p.clone();
            p_pert.at<double>(j) += eps_step;
            cv::Mat X_pert = buildX(p_pert);
            std::vector<cv::Mat> res_pert;
            cv::Mat mean_pert;
            eval(X_pert, res_pert, mean_pert);
            for (int i = 0; i < n; i++) {
                J_full.at<double>(3 * i,     j) = (res_pert[i].at<double>(0) - res[i].at<double>(0)) / eps_step;
                J_full.at<double>(3 * i + 1, j) = (res_pert[i].at<double>(1) - res[i].at<double>(1)) / eps_step;
                J_full.at<double>(3 * i + 2, j) = (res_pert[i].at<double>(2) - res[i].at<double>(2)) / eps_step;
            }
        }

        cv::Mat H = J_full.t() * J_full;
        cv::Mat g = J_full.t() * R_full;
        for (int j = 0; j < 6; j++)
            H.at<double>(j, j) += lambda;

        cv::Mat dp;
        if (!cv::solve(H, -g, dp, cv::DECOMP_SVD))
            break;

        cv::Mat p_new = p + dp;
        cv::Mat X_new = buildX(p_new);
        std::vector<cv::Mat> res_new;
        cv::Mat mean_new;
        double cost = eval(X_new, res_new, mean_new);

        if (cost < prev_cost) {
            p = p_new;
            X = X_new;
            res = res_new;
            mean_t = mean_new;
            lambda *= 0.3;
            if (emit_diagnostics && iter % 10 == 0)
                std::cout << "[refine]   iter " << iter << ": "
                          << (cost * 1e6) << " mm²" << std::endl;
            if (std::abs(prev_cost - cost) < 1e-14 || prev_cost == 0.0)
                break;
            prev_cost = cost;
        } else {
            lambda *= 3.0;
        }
    }

    if (emit_diagnostics)
        std::cout << "[refine] 精化后方差: " << (prev_cost * 1e6) << " mm²" << std::endl;
    return X;
}

constexpr int kMinHandEyeFrames = 12;
constexpr int kMinHandEyeCorners = 12;
constexpr double kMinimumInlierRatio = 0.60;
constexpr double kMadScale = 3.0;
constexpr double kMinimumOutlierThresholdMeters = 0.003;  // 3 mm
constexpr double kMaxTranslationRmsMeters = 0.005;        // 5 mm
constexpr double kMaxTranslationP95Meters = 0.008;        // 8 mm
constexpr double kMaxRotationRmsDegrees = 1.0;
constexpr double kMaxRotationP95Degrees = 2.0;
constexpr int kMinimumConsensusSolvers = 3;
constexpr double kMaxSolverPairTranslationMeters = 0.010;  // 10 mm
constexpr double kMaxSolverPairRotationDegrees = 1.0;

static double percentile(std::vector<double> values, double quantile) {
    if (values.empty())
        return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const double position = std::clamp(quantile, 0.0, 1.0) * (values.size() - 1);
    const size_t lower = static_cast<size_t>(std::floor(position));
    const size_t upper = static_cast<size_t>(std::ceil(position));
    const double weight = position - lower;
    return values[lower] * (1.0 - weight) + values[upper] * weight;
}

struct HandEyeConsistencyStats {
    std::vector<double> translation_errors;
    double translation_rms = std::numeric_limits<double>::quiet_NaN();
    double translation_mean = std::numeric_limits<double>::quiet_NaN();
    double translation_median = std::numeric_limits<double>::quiet_NaN();
    double translation_p95 = std::numeric_limits<double>::quiet_NaN();
    double translation_max = std::numeric_limits<double>::quiet_NaN();
    double rotation_rms_degrees = std::numeric_limits<double>::quiet_NaN();
    double rotation_p95_degrees = std::numeric_limits<double>::quiet_NaN();
    double rotation_max_degrees = std::numeric_limits<double>::quiet_NaN();
    cv::Vec3d translation_span_mm{0.0, 0.0, 0.0};
};

static HandEyeConsistencyStats evaluateHandEyeConsistency(
    const cv::Mat& transform,
    const std::vector<cv::Mat>& robot_poses,
    const std::vector<cv::Mat>& camera_poses) {
    HandEyeConsistencyStats stats;
    const int count = static_cast<int>(robot_poses.size());
    if (transform.empty() || count == 0 || camera_poses.size() != robot_poses.size())
        return stats;

    std::vector<cv::Mat> base_board_poses;
    base_board_poses.reserve(count);
    cv::Mat mean_translation = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat rotation_sum = cv::Mat::zeros(3, 3, CV_64F);
    cv::Vec3d minimum(std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max(),
                      std::numeric_limits<double>::max());
    cv::Vec3d maximum(-std::numeric_limits<double>::max(),
                      -std::numeric_limits<double>::max(),
                      -std::numeric_limits<double>::max());

    for (int index = 0; index < count; ++index) {
        cv::Mat base_board = robot_poses[index] * transform * camera_poses[index];
        if (!cv::checkRange(base_board))
            return stats;
        base_board_poses.push_back(base_board);
        mean_translation += base_board(cv::Rect(3, 0, 1, 3));
        rotation_sum += base_board(cv::Rect(0, 0, 3, 3));
        for (int axis = 0; axis < 3; ++axis) {
            const double value = base_board.at<double>(axis, 3);
            minimum[axis] = std::min(minimum[axis], value);
            maximum[axis] = std::max(maximum[axis], value);
        }
    }
    mean_translation /= count;

    cv::SVD rotation_svd(rotation_sum, cv::SVD::FULL_UV);
    cv::Mat mean_rotation = rotation_svd.u * rotation_svd.vt;
    if (cv::determinant(mean_rotation) < 0.0) {
        cv::Mat corrected_u = rotation_svd.u.clone();
        corrected_u.col(2) *= -1.0;
        mean_rotation = corrected_u * rotation_svd.vt;
    }

    double translation_squared_sum = 0.0;
    double translation_sum = 0.0;
    double rotation_squared_sum = 0.0;
    std::vector<double> rotation_errors;
    stats.translation_errors.reserve(count);
    rotation_errors.reserve(count);
    for (const cv::Mat& base_board : base_board_poses) {
        const cv::Mat difference = base_board(cv::Rect(3, 0, 1, 3)) - mean_translation;
        const double translation_error = std::sqrt(difference.dot(difference));
        stats.translation_errors.push_back(translation_error);
        translation_squared_sum += translation_error * translation_error;
        translation_sum += translation_error;

        const cv::Mat relative_rotation =
            mean_rotation.t() * base_board(cv::Rect(0, 0, 3, 3));
        const double cosine = std::clamp(
            (cv::trace(relative_rotation)[0] - 1.0) * 0.5, -1.0, 1.0);
        const double rotation_error = std::acos(cosine) * 180.0 / CV_PI;
        rotation_errors.push_back(rotation_error);
        rotation_squared_sum += rotation_error * rotation_error;
    }

    stats.translation_rms = std::sqrt(translation_squared_sum / count);
    stats.translation_mean = translation_sum / count;
    stats.translation_median = percentile(stats.translation_errors, 0.5);
    stats.translation_p95 = percentile(stats.translation_errors, 0.95);
    stats.translation_max = *std::max_element(
        stats.translation_errors.begin(), stats.translation_errors.end());
    stats.rotation_rms_degrees = std::sqrt(rotation_squared_sum / count);
    stats.rotation_p95_degrees = percentile(rotation_errors, 0.95);
    stats.rotation_max_degrees = *std::max_element(rotation_errors.begin(), rotation_errors.end());
    for (int axis = 0; axis < 3; ++axis)
        stats.translation_span_mm[axis] = (maximum[axis] - minimum[axis]) * 1000.0;
    return stats;
}

struct HandEyeMethodResult {
    std::string name;
    cv::HandEyeCalibrationMethod method;
    cv::Mat transform;
    HandEyeConsistencyStats stats;
};

struct HandEyeSolveAudit {
    std::vector<double> initial_consistency_errors;
    std::vector<bool> mad_inliers;
    double mad_threshold = std::numeric_limits<double>::quiet_NaN();
    std::string selected_method;
    bool quality_passed = false;
    HandEyePhase2Audit phase2;
};

static std::vector<HandEyeMethodResult> solveHandEyeMethods(
    const std::vector<cv::Mat>& robot_poses,
    const std::vector<cv::Mat>& camera_poses,
    const char* phase,
    HandEyePhase2Audit* audit,
    bool emit_diagnostics = true,
    const std::string& only_method = "") {
    std::vector<cv::Mat> robot_rotations, robot_translations;
    std::vector<cv::Mat> camera_rotations, camera_translations;
    robot_rotations.reserve(robot_poses.size());
    robot_translations.reserve(robot_poses.size());
    camera_rotations.reserve(camera_poses.size());
    camera_translations.reserve(camera_poses.size());
    for (size_t index = 0; index < robot_poses.size(); ++index) {
        robot_rotations.push_back(robot_poses[index](cv::Rect(0, 0, 3, 3)).clone());
        robot_translations.push_back(robot_poses[index](cv::Rect(3, 0, 1, 3)).clone());
        camera_rotations.push_back(camera_poses[index](cv::Rect(0, 0, 3, 3)).clone());
        camera_translations.push_back(camera_poses[index](cv::Rect(3, 0, 1, 3)).clone());
    }

    const std::array<std::pair<const char*, cv::HandEyeCalibrationMethod>, 5> methods{{
        {"TSAI", cv::CALIB_HAND_EYE_TSAI},
        {"PARK", cv::CALIB_HAND_EYE_PARK},
        {"HORAUD", cv::CALIB_HAND_EYE_HORAUD},
        {"ANDREFF", cv::CALIB_HAND_EYE_ANDREFF},
        {"DANIILIDIS", cv::CALIB_HAND_EYE_DANIILIDIS}
    }};

    std::vector<HandEyeMethodResult> results;
    for (const auto& method : methods) {
        if (!only_method.empty() && only_method != method.first)
            continue;
        HandEyeSolverAuditRecord auditRecord;
        auditRecord.phase = phase;
        auditRecord.method = method.first;
        cv::Mat rotation_camera_to_gripper, translation_camera_to_gripper;
        try {
            cv::calibrateHandEye(robot_rotations, robot_translations,
                                 camera_rotations, camera_translations,
                                 rotation_camera_to_gripper,
                                 translation_camera_to_gripper,
                                 method.second);
        } catch (const cv::Exception& exception) {
            if (emit_diagnostics) {
                APP_LOG_EVENT_WARNING("calibration.hand_eye",
                                  "hand_eye_method",
                                  "fail",
                                  {{"phase", phase},
                                   {"method", method.first},
                                   {"frames", QString::number(robot_poses.size())},
                                   {"reason", "opencv_exception"},
                                   {"message", QString::fromUtf8(exception.what())}});
            }
            auditRecord.failure_reason = "opencv_exception";
            if (audit)
                audit->solvers.push_back(std::move(auditRecord));
            continue;
        }
        if (rotation_camera_to_gripper.empty() || translation_camera_to_gripper.empty() ||
            !cv::checkRange(rotation_camera_to_gripper) ||
            !cv::checkRange(translation_camera_to_gripper)) {
            if (emit_diagnostics) {
                APP_LOG_EVENT_WARNING("calibration.hand_eye",
                                  "hand_eye_method",
                                  "fail",
                                  {{"phase", phase},
                                   {"method", method.first},
                                   {"frames", QString::number(robot_poses.size())},
                                   {"reason", "non_finite_output"}});
            }
            auditRecord.failure_reason = "non_finite_output";
            if (audit)
                audit->solvers.push_back(std::move(auditRecord));
            continue;
        }

        cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
        rotation_camera_to_gripper.copyTo(transform(cv::Rect(0, 0, 3, 3)));
        translation_camera_to_gripper.copyTo(transform(cv::Rect(3, 0, 1, 3)));
        HandEyeConsistencyStats stats =
            evaluateHandEyeConsistency(transform, robot_poses, camera_poses);
        if (!std::isfinite(stats.translation_rms) ||
            !std::isfinite(stats.rotation_rms_degrees)) {
            if (emit_diagnostics) {
                APP_LOG_EVENT_WARNING("calibration.hand_eye",
                                  "hand_eye_method",
                                  "fail",
                                  {{"phase", phase},
                                   {"method", method.first},
                                   {"frames", QString::number(robot_poses.size())},
                                   {"reason", "invalid_quality_metrics"}});
            }
            auditRecord.failure_reason = "invalid_quality_metrics";
            if (audit)
                audit->solvers.push_back(std::move(auditRecord));
            continue;
        }

        auditRecord.success = true;
        auditRecord.raw_transform = transform.clone();
        auditRecord.raw_translation_rms_mm = stats.translation_rms * 1000.0;
        auditRecord.raw_translation_p95_mm = stats.translation_p95 * 1000.0;
        auditRecord.raw_rotation_rms_deg = stats.rotation_rms_degrees;
        auditRecord.raw_rotation_p95_deg = stats.rotation_p95_degrees;
        if (audit)
            audit->solvers.push_back(auditRecord);

        if (emit_diagnostics) {
            APP_LOG_EVENT_INFO("calibration.hand_eye",
                           "hand_eye_method",
                           "success",
                           {{"phase", phase},
                            {"method", method.first},
                            {"result_stage", "raw_unoptimized"},
                            {"frames", QString::number(robot_poses.size())},
                            {"translation_rms_mm", QString::number(stats.translation_rms * 1000.0, 'f', 6)},
                            {"translation_p95_mm", QString::number(stats.translation_p95 * 1000.0, 'f', 6)},
                            {"rotation_rms_deg", QString::number(stats.rotation_rms_degrees, 'f', 6)},
                            {"rotation_p95_deg", QString::number(stats.rotation_p95_degrees, 'f', 6)}});
            std::cout << "[autoCalib] " << method.first << " [raw]"
                      << " -> 平移一致性 RMS: " << stats.translation_rms * 1000.0
                      << " mm, P95: " << stats.translation_p95 * 1000.0
                      << " mm, 旋转 RMS: " << stats.rotation_rms_degrees << " deg" << std::endl;
        }
        results.push_back({method.first, method.second, transform, std::move(stats)});
    }
    return results;
}

struct TransformDistance {
    double translation_meters = std::numeric_limits<double>::quiet_NaN();
    double rotation_degrees = std::numeric_limits<double>::quiet_NaN();
};

static TransformDistance measureTransformDistance(const cv::Mat& left,
                                                  const cv::Mat& right) {
    TransformDistance distance;
    if (left.rows != 4 || left.cols != 4 || right.rows != 4 || right.cols != 4)
        return distance;
    const cv::Mat leftRotation = left(cv::Rect(0, 0, 3, 3));
    const cv::Mat rightRotation = right(cv::Rect(0, 0, 3, 3));
    const cv::Mat relativeRotation = leftRotation.t() * rightRotation;
    const cv::Mat translationDifference =
        right(cv::Rect(3, 0, 1, 3)) - left(cv::Rect(3, 0, 1, 3));
    distance.translation_meters = cv::norm(leftRotation.t() * translationDifference);
    const double cosine = std::clamp(
        (cv::trace(relativeRotation)[0] - 1.0) * 0.5, -1.0, 1.0);
    distance.rotation_degrees = std::acos(cosine) * 180.0 / CV_PI;
    return distance;
}

struct SolverConsensusResult {
    size_t medoid_index = 0;
    double max_pair_translation_meters = 0.0;
    double max_pair_rotation_degrees = 0.0;
    bool passed = false;
    std::string failure_reason;
};

static SolverConsensusResult evaluateSolverConsensus(
    const std::vector<HandEyeMethodResult>& results,
    const char* phase,
    HandEyePhase2Audit* audit,
    bool emit_diagnostics = true) {
    SolverConsensusResult consensus;
    std::vector<double> normalizedDistanceSums(results.size(), 0.0);
    bool invalidDistance = false;
    for (size_t left = 0; left < results.size(); ++left) {
        for (size_t right = left + 1; right < results.size(); ++right) {
            const TransformDistance distance = measureTransformDistance(
                results[left].transform, results[right].transform);
            if (!std::isfinite(distance.translation_meters) ||
                !std::isfinite(distance.rotation_degrees)) {
                invalidDistance = true;
                continue;
            }
            consensus.max_pair_translation_meters = std::max(
                consensus.max_pair_translation_meters, distance.translation_meters);
            consensus.max_pair_rotation_degrees = std::max(
                consensus.max_pair_rotation_degrees, distance.rotation_degrees);
            const double normalized =
                distance.translation_meters / kMaxSolverPairTranslationMeters +
                distance.rotation_degrees / kMaxSolverPairRotationDegrees;
            normalizedDistanceSums[left] += normalized;
            normalizedDistanceSums[right] += normalized;

            if (audit) {
                HandEyeSolverPairAuditRecord pair;
                pair.phase = phase;
                pair.left_method = results[left].name;
                pair.right_method = results[right].name;
                pair.delta_translation_mm = distance.translation_meters * 1000.0;
                pair.delta_rotation_deg = distance.rotation_degrees;
                audit->pairs.push_back(std::move(pair));
            }
            if (emit_diagnostics) {
                APP_LOG_EVENT_INFO(
                    "calibration.hand_eye", "solver_consensus_pair", "measured",
                    {{"phase", phase},
                     {"left_method", QString::fromStdString(results[left].name)},
                     {"right_method", QString::fromStdString(results[right].name)},
                     {"delta_translation_mm", QString::number(
                          distance.translation_meters * 1000.0, 'f', 6)},
                     {"delta_rotation_deg", QString::number(
                          distance.rotation_degrees, 'f', 6)}});
            }
        }
    }

    if (!results.empty()) {
        consensus.medoid_index = static_cast<size_t>(std::distance(
            normalizedDistanceSums.begin(),
            std::min_element(normalizedDistanceSums.begin(),
                             normalizedDistanceSums.end())));
    }
    if (results.size() < static_cast<size_t>(kMinimumConsensusSolvers))
        consensus.failure_reason = "insufficient_successful_solvers";
    else if (invalidDistance)
        consensus.failure_reason = "invalid_solver_distance";
    else if (consensus.max_pair_translation_meters >
             kMaxSolverPairTranslationMeters)
        consensus.failure_reason = "solver_translation_disagreement";
    else if (consensus.max_pair_rotation_degrees >
             kMaxSolverPairRotationDegrees)
        consensus.failure_reason = "solver_rotation_disagreement";
    consensus.passed = consensus.failure_reason.empty();

    if (audit) {
        for (HandEyeSolverAuditRecord& record : audit->solvers) {
            if (record.phase != phase || !record.success)
                continue;
            for (size_t index = 0; index < results.size(); ++index) {
                if (record.method == results[index].name) {
                    record.medoid_normalized_distance_sum = normalizedDistanceSums[index];
                    record.selected_medoid = index == consensus.medoid_index;
                    break;
                }
            }
        }
        HandEyeConsensusAuditRecord record;
        record.phase = phase;
        record.successful_solvers = static_cast<int>(results.size());
        if (!results.empty())
            record.medoid_method = results[consensus.medoid_index].name;
        record.max_pair_translation_mm =
            consensus.max_pair_translation_meters * 1000.0;
        record.max_pair_rotation_deg = consensus.max_pair_rotation_degrees;
        record.translation_limit_mm = kMaxSolverPairTranslationMeters * 1000.0;
        record.rotation_limit_deg = kMaxSolverPairRotationDegrees;
        record.passed = consensus.passed;
        record.failure_reason = consensus.failure_reason;
        audit->consensus.push_back(std::move(record));
    }

    const std::string medoid = results.empty()
        ? "none" : results[consensus.medoid_index].name;
    if (emit_diagnostics) {
        APP_LOG_EVENT(
            Info, "calibration.hand_eye", "solver_consensus",
            consensus.passed ? "success" : "fail",
            {"phase", phase},
            {"successful_solvers", QString::number(results.size())},
            {"medoid_method", QString::fromStdString(medoid)},
            {"max_pair_translation_mm", QString::number(
                 consensus.max_pair_translation_meters * 1000.0, 'f', 6)},
            {"max_pair_rotation_deg", QString::number(
                 consensus.max_pair_rotation_degrees, 'f', 6)},
            {"translation_limit_mm", QString::number(
                 kMaxSolverPairTranslationMeters * 1000.0, 'f', 3)},
            {"rotation_limit_deg", QString::number(
                 kMaxSolverPairRotationDegrees, 'f', 3)},
            {"failure_reason", QString::fromStdString(consensus.failure_reason)});
        std::cout << "[autoCalib] Solver consensus [" << phase << "]: medoid="
                  << medoid << ", max Δt="
                  << consensus.max_pair_translation_meters * 1000.0
                  << " mm, max ΔR=" << consensus.max_pair_rotation_degrees
                  << " deg, gate=" << (consensus.passed ? "PASS" : "FAIL")
                  << std::endl;
    }
    return consensus;
}

struct OptimizedMedoidResult {
    cv::Mat transform;
    HandEyeConsistencyStats stats;
};

static OptimizedMedoidResult optimizeMedoidOnce(
    const HandEyeMethodResult& medoid,
    const std::vector<cv::Mat>& robot_poses,
    const std::vector<cv::Mat>& camera_poses,
    const char* phase,
    HandEyePhase2Audit* audit,
    bool emit_diagnostics = true) {
    OptimizedMedoidResult optimized;
    optimized.transform = refineHandEyePoseConsistency(
        medoid.transform, robot_poses, camera_poses, emit_diagnostics);
    optimized.stats = evaluateHandEyeConsistency(
        optimized.transform, robot_poses, camera_poses);
    const TransformDistance delta = measureTransformDistance(
        medoid.transform, optimized.transform);

    HandEyeOptimizationAuditRecord record;
    record.performed = true;
    record.phase = phase;
    record.method = medoid.name;
    record.initial_transform = medoid.transform.clone();
    record.optimized_transform = optimized.transform.clone();
    record.delta_translation_mm = delta.translation_meters * 1000.0;
    record.delta_rotation_deg = delta.rotation_degrees;
    record.before_translation_rms_mm = medoid.stats.translation_rms * 1000.0;
    record.after_translation_rms_mm = optimized.stats.translation_rms * 1000.0;
    record.before_rotation_rms_deg = medoid.stats.rotation_rms_degrees;
    record.after_rotation_rms_deg = optimized.stats.rotation_rms_degrees;
    if (audit)
        audit->optimizations.push_back(record);

    if (emit_diagnostics) {
        APP_LOG_EVENT_INFO(
            "calibration.hand_eye", "medoid_refinement", "complete",
            {{"phase", phase},
             {"method", QString::fromStdString(medoid.name)},
             {"delta_translation_mm", QString::number(
                  delta.translation_meters * 1000.0, 'f', 6)},
             {"delta_rotation_deg", QString::number(delta.rotation_degrees, 'f', 6)},
             {"before_translation_rms_mm", QString::number(
                  medoid.stats.translation_rms * 1000.0, 'f', 6)},
             {"after_translation_rms_mm", QString::number(
                  optimized.stats.translation_rms * 1000.0, 'f', 6)},
             {"before_rotation_rms_deg", QString::number(
                  medoid.stats.rotation_rms_degrees, 'f', 6)},
             {"after_rotation_rms_deg", QString::number(
                  optimized.stats.rotation_rms_degrees, 'f', 6)}});
        std::cout << "[autoCalib] Medoid refinement [" << phase << "] "
                  << medoid.name << ": Δt=" << delta.translation_meters * 1000.0
                  << " mm, ΔR=" << delta.rotation_degrees << " deg" << std::endl;
    }
    return optimized;
}

struct LooConsensusStageResult {
    bool calculable = false;
    std::string failure_reason;
    std::string selected_method;
    cv::Mat transform;
    HandEyeConsistencyStats quality;
    SolverConsensusResult consensus;
};

struct FullPipelineLooResult {
    bool success = false;
    bool filter_set_evaluated = false;
    std::string failure_reason;
    std::string selected_method;
    cv::Mat transform;
    std::vector<size_t> inlier_positions;
};

static LooConsensusStageResult solveLooConsensusStage(
    const std::vector<cv::Mat>& robot_poses,
    const std::vector<cv::Mat>& camera_poses,
    const char* phase) {
    LooConsensusStageResult stage;
    if (robot_poses.size() != camera_poses.size()) {
        stage.failure_reason = "pose_count_mismatch";
        return stage;
    }
    if (robot_poses.size() < static_cast<size_t>(kMinHandEyeFrames)) {
        stage.failure_reason = "insufficient_frames";
        return stage;
    }
    std::vector<HandEyeMethodResult> methods = solveHandEyeMethods(
        robot_poses, camera_poses, phase, nullptr, false);
    if (methods.empty()) {
        stage.failure_reason = "all_methods_failed";
        return stage;
    }
    stage.consensus = evaluateSolverConsensus(methods, phase, nullptr, false);
    if (stage.consensus.medoid_index >= methods.size()) {
        stage.failure_reason = "invalid_medoid_index";
        return stage;
    }
    const HandEyeMethodResult& medoid = methods[stage.consensus.medoid_index];
    const OptimizedMedoidResult optimized = optimizeMedoidOnce(
        medoid, robot_poses, camera_poses, phase, nullptr, false);
    if (optimized.transform.empty() ||
        !cv::checkRange(optimized.transform) ||
        !std::isfinite(optimized.stats.translation_rms) ||
        !std::isfinite(optimized.stats.rotation_rms_degrees)) {
        stage.failure_reason = "invalid_optimized_quality_metrics";
        return stage;
    }
    stage.calculable = true;
    stage.selected_method = medoid.name;
    stage.transform = optimized.transform.clone();
    stage.quality = optimized.stats;
    return stage;
}

static LooConsensusStageResult solveFixedInlierLooStage(
    const std::vector<cv::Mat>& robot_poses,
    const std::vector<cv::Mat>& camera_poses,
    const std::string& selected_method) {
    LooConsensusStageResult stage;
    if (robot_poses.size() != camera_poses.size()) {
        stage.failure_reason = "pose_count_mismatch";
        return stage;
    }
    if (robot_poses.size() < static_cast<size_t>(kMinHandEyeFrames)) {
        stage.failure_reason = "insufficient_frames";
        return stage;
    }
    std::vector<HandEyeMethodResult> methods = solveHandEyeMethods(
        robot_poses, camera_poses, "loo_fixed", nullptr, false,
        selected_method);
    if (methods.empty()) {
        stage.failure_reason = "selected_method_failed";
        return stage;
    }
    const OptimizedMedoidResult optimized = optimizeMedoidOnce(
        methods.front(), robot_poses, camera_poses, "loo_fixed",
        nullptr, false);
    if (optimized.transform.empty() || !cv::checkRange(optimized.transform) ||
        !std::isfinite(optimized.stats.translation_rms) ||
        !std::isfinite(optimized.stats.rotation_rms_degrees)) {
        stage.failure_reason = "invalid_optimized_quality_metrics";
        return stage;
    }
    stage.calculable = true;
    stage.consensus.passed = true;
    stage.selected_method = methods.front().name;
    stage.transform = optimized.transform.clone();
    stage.quality = optimized.stats;
    return stage;
}

static std::string looQualityFailure(
    const LooConsensusStageResult& stage) {
    if (!stage.calculable)
        return stage.failure_reason;
    if (!stage.consensus.passed)
        return stage.consensus.failure_reason;
    if (stage.quality.translation_rms > kMaxTranslationRmsMeters)
        return "translation_rms_exceeded";
    if (stage.quality.translation_p95 > kMaxTranslationP95Meters)
        return "translation_p95_exceeded";
    if (stage.quality.rotation_rms_degrees > kMaxRotationRmsDegrees)
        return "rotation_rms_exceeded";
    if (stage.quality.rotation_p95_degrees > kMaxRotationP95Degrees)
        return "rotation_p95_exceeded";
    return "";
}

static FullPipelineLooResult runFullPipelineLooTrial(
    const std::vector<cv::Mat>& robot_poses,
    const std::vector<cv::Mat>& camera_poses) {
    FullPipelineLooResult result;
    LooConsensusStageResult final_stage = solveLooConsensusStage(
        robot_poses, camera_poses, "loo_full_initial");
    if (!final_stage.calculable) {
        result.failure_reason = final_stage.failure_reason;
        return result;
    }
    if (final_stage.quality.translation_errors.size() != robot_poses.size()) {
        result.failure_reason = "invalid_consistency_error_count";
        return result;
    }

    const double median_error = percentile(
        final_stage.quality.translation_errors, 0.5);
    std::vector<double> deviations;
    deviations.reserve(final_stage.quality.translation_errors.size());
    for (const double value : final_stage.quality.translation_errors)
        deviations.push_back(std::abs(value - median_error));
    const double mad = percentile(deviations, 0.5);
    const double threshold = std::max(
        kMinimumOutlierThresholdMeters,
        median_error + kMadScale * 1.4826 * mad);

    std::vector<cv::Mat> filtered_robot;
    std::vector<cv::Mat> filtered_camera;
    for (size_t index = 0;
         index < final_stage.quality.translation_errors.size(); ++index) {
        if (final_stage.quality.translation_errors[index] <= threshold) {
            result.inlier_positions.push_back(index);
            filtered_robot.push_back(robot_poses[index]);
            filtered_camera.push_back(camera_poses[index]);
        }
    }
    result.filter_set_evaluated = true;
    const int minimum_inliers = std::max(
        kMinHandEyeFrames,
        static_cast<int>(std::ceil(
            robot_poses.size() * kMinimumInlierRatio)));
    if (filtered_robot.size() < static_cast<size_t>(minimum_inliers)) {
        result.failure_reason = "insufficient_inliers";
        return result;
    }
    if (filtered_robot.size() != robot_poses.size()) {
        final_stage = solveLooConsensusStage(
            filtered_robot, filtered_camera, "loo_full_refit");
        if (!final_stage.calculable) {
            result.failure_reason = final_stage.failure_reason;
            return result;
        }
    }
    result.failure_reason = looQualityFailure(final_stage);
    if (!result.failure_reason.empty())
        return result;
    result.success = true;
    result.selected_method = final_stage.selected_method;
    result.transform = final_stage.transform.clone();
    return result;
}

static HandEyeLooSummaryAuditRecord summarizeLooTrials(
    const std::string& mode,
    const std::vector<HandEyeLooTrialAuditRecord>& trials) {
    HandEyeLooSummaryAuditRecord summary;
    summary.mode = mode;
    std::vector<double> translations;
    std::vector<double> rotations;
    for (const HandEyeLooTrialAuditRecord& trial : trials) {
        if (trial.mode != mode)
            continue;
        ++summary.attempted;
        if (trial.filter_set_evaluated &&
            (!trial.filter_set_added_source_indices.empty() ||
             !trial.filter_set_removed_source_indices.empty())) {
            ++summary.trials_with_filter_set_changes;
        }
        if (!trial.success) {
            ++summary.failed;
            continue;
        }
        ++summary.successful;
        translations.push_back(trial.delta_translation_mm);
        rotations.push_back(trial.delta_rotation_deg);
        if (summary.translation_max_source_index < 0 ||
            trial.delta_translation_mm > summary.translation_max_mm) {
            summary.translation_max_mm = trial.delta_translation_mm;
            summary.translation_max_frame_id = trial.removed_frame_id;
            summary.translation_max_source_index = trial.removed_source_index;
        }
        if (summary.rotation_max_source_index < 0 ||
            trial.delta_rotation_deg > summary.rotation_max_deg) {
            summary.rotation_max_deg = trial.delta_rotation_deg;
            summary.rotation_max_frame_id = trial.removed_frame_id;
            summary.rotation_max_source_index = trial.removed_source_index;
        }
    }
    if (!translations.empty()) {
        summary.translation_mean_mm = std::accumulate(
            translations.begin(), translations.end(), 0.0) /
            translations.size();
        summary.translation_p95_mm = percentile(translations, 0.95);
        summary.rotation_mean_deg = std::accumulate(
            rotations.begin(), rotations.end(), 0.0) / rotations.size();
        summary.rotation_p95_deg = percentile(rotations, 0.95);
    }
    return summary;
}

static bool persistLooProgress(CalibrationSession* session,
                               const HandEyePhase3Audit& audit,
                               std::string& error) {
    return session && session->setHandEyePhase3Audit(audit, error);
}

static bool runPhase3LooDiagnostics(
    const std::vector<cv::Mat>& all_robot_poses,
    const std::vector<cv::Mat>& all_camera_poses,
    const std::vector<int>& source_indices,
    const std::vector<bool>& full_mad_inliers,
    const cv::Mat& full_transform,
    const std::string& full_selected_method,
    CalibrationSession* session,
    HandEyePhase3Audit& audit,
    std::string& error) {
    audit.enabled = true;
    audit.status = "running";
    audit.full_transform = full_transform.clone();
    audit.full_selected_method = full_selected_method;
    const size_t worker_count = std::max<size_t>(1, std::min<size_t>(
        4, static_cast<size_t>(std::thread::hardware_concurrency())));
    audit.worker_count = static_cast<int>(worker_count);
    if (all_robot_poses.size() != all_camera_poses.size() ||
        all_robot_poses.size() != source_indices.size() ||
        all_robot_poses.size() != full_mad_inliers.size()) {
        audit.status = "fail";
        audit.failure_reason = "phase3_input_size_mismatch";
        persistLooProgress(session, audit, error);
        if (error.empty())
            error = audit.failure_reason;
        return false;
    }

    std::vector<size_t> baseline_inlier_positions;
    std::set<int> baseline_inlier_sources;
    for (size_t index = 0; index < full_mad_inliers.size(); ++index) {
        if (!full_mad_inliers[index])
            continue;
        baseline_inlier_positions.push_back(index);
        baseline_inlier_sources.insert(source_indices[index]);
        audit.full_inlier_source_indices.push_back(source_indices[index]);
    }
    if (!persistLooProgress(session, audit, error))
        return false;

    constexpr size_t kPersistEveryTrials = 25;
    const auto run_fixed_trial = [&](size_t removed) {
        HandEyeLooTrialAuditRecord trial;
        trial.mode = "fixed_final_inliers";
        const size_t removed_position = baseline_inlier_positions[removed];
        trial.removed_frame_id = source_indices[removed_position];
        trial.removed_source_index = source_indices[removed_position];
        std::vector<cv::Mat> robot_subset;
        std::vector<cv::Mat> camera_subset;
        for (const size_t position : baseline_inlier_positions) {
            if (position == removed_position)
                continue;
            robot_subset.push_back(all_robot_poses[position]);
            camera_subset.push_back(all_camera_poses[position]);
        }
        trial.final_inliers = static_cast<int>(robot_subset.size());
        const LooConsensusStageResult stage = solveFixedInlierLooStage(
            robot_subset, camera_subset, full_selected_method);
        trial.failure_reason = looQualityFailure(stage);
        if (trial.failure_reason.empty()) {
            trial.success = true;
            trial.selected_method = stage.selected_method;
            const TransformDistance delta = measureTransformDistance(
                full_transform, stage.transform);
            if (!std::isfinite(delta.translation_meters) ||
                !std::isfinite(delta.rotation_degrees)) {
                trial.success = false;
                trial.failure_reason = "invalid_transform_distance";
            } else {
                trial.delta_translation_mm = delta.translation_meters * 1000.0;
                trial.delta_rotation_deg = delta.rotation_degrees;
            }
        }
        return trial;
    };
    size_t next_fixed_persist = kPersistEveryTrials;
    for (size_t batch = 0; batch < baseline_inlier_positions.size();
         batch += worker_count) {
        const size_t batch_end = std::min(
            batch + worker_count, baseline_inlier_positions.size());
        std::vector<std::future<HandEyeLooTrialAuditRecord>> futures;
        for (size_t removed = batch; removed < batch_end; ++removed) {
            futures.push_back(std::async(
                std::launch::async, run_fixed_trial, removed));
        }
        for (size_t index = 0; index < futures.size(); ++index) {
            try {
                audit.trials.push_back(futures[index].get());
            } catch (const std::exception& exception) {
                const size_t removed = batch + index;
                const size_t position = baseline_inlier_positions[removed];
                HandEyeLooTrialAuditRecord trial;
                trial.mode = "fixed_final_inliers";
                trial.removed_frame_id = source_indices[position];
                trial.removed_source_index = source_indices[position];
                trial.failure_reason = std::string("worker_exception: ") +
                    exception.what();
                audit.trials.push_back(std::move(trial));
            }
        }
        if (batch_end >= next_fixed_persist) {
            APP_LOG_EVENT_INFO(
                "calibration.hand_eye", "loo_progress", "running",
                {{"mode", "fixed_final_inliers"},
                 {"completed", QString::number(batch_end)},
                 {"total", QString::number(baseline_inlier_positions.size())}});
            if (!persistLooProgress(session, audit, error))
                return false;
            while (next_fixed_persist <= batch_end)
                next_fixed_persist += kPersistEveryTrials;
        }
    }
    audit.summaries.push_back(summarizeLooTrials(
        "fixed_final_inliers", audit.trials));
    if (!persistLooProgress(session, audit, error))
        return false;

    const auto run_full_trial = [&](size_t removed) {
        HandEyeLooTrialAuditRecord trial;
        trial.mode = "full_pipeline";
        trial.removed_frame_id = source_indices[removed];
        trial.removed_source_index = source_indices[removed];
        std::vector<cv::Mat> robot_subset;
        std::vector<cv::Mat> camera_subset;
        std::vector<int> subset_sources;
        for (size_t index = 0; index < all_robot_poses.size(); ++index) {
            if (index == removed)
                continue;
            robot_subset.push_back(all_robot_poses[index]);
            camera_subset.push_back(all_camera_poses[index]);
            subset_sources.push_back(source_indices[index]);
        }
        const FullPipelineLooResult run = runFullPipelineLooTrial(
            robot_subset, camera_subset);
        trial.failure_reason = run.failure_reason;
        trial.final_inliers = static_cast<int>(run.inlier_positions.size());
        trial.filter_set_evaluated = run.filter_set_evaluated;
        if (run.success) {
            trial.success = true;
            trial.selected_method = run.selected_method;
            const TransformDistance delta = measureTransformDistance(
                full_transform, run.transform);
            if (!std::isfinite(delta.translation_meters) ||
                !std::isfinite(delta.rotation_degrees)) {
                trial.success = false;
                trial.failure_reason = "invalid_transform_distance";
            } else {
                trial.delta_translation_mm = delta.translation_meters * 1000.0;
                trial.delta_rotation_deg = delta.rotation_degrees;
            }
        }
        if (run.filter_set_evaluated) {
            std::set<int> trial_sources;
            for (const size_t position : run.inlier_positions) {
                if (position < subset_sources.size())
                    trial_sources.insert(subset_sources[position]);
            }
            std::set<int> expected_sources = baseline_inlier_sources;
            expected_sources.erase(source_indices[removed]);
            std::set_difference(
                trial_sources.begin(), trial_sources.end(),
                expected_sources.begin(), expected_sources.end(),
                std::back_inserter(trial.filter_set_added_source_indices));
            std::set_difference(
                expected_sources.begin(), expected_sources.end(),
                trial_sources.begin(), trial_sources.end(),
                std::back_inserter(trial.filter_set_removed_source_indices));
        }
        return trial;
    };
    size_t next_full_persist = kPersistEveryTrials;
    for (size_t batch = 0; batch < all_robot_poses.size();
         batch += worker_count) {
        const size_t batch_end = std::min(
            batch + worker_count, all_robot_poses.size());
        std::vector<std::future<HandEyeLooTrialAuditRecord>> futures;
        for (size_t removed = batch; removed < batch_end; ++removed) {
            futures.push_back(std::async(
                std::launch::async, run_full_trial, removed));
        }
        for (size_t index = 0; index < futures.size(); ++index) {
            try {
                audit.trials.push_back(futures[index].get());
            } catch (const std::exception& exception) {
                const size_t removed = batch + index;
                HandEyeLooTrialAuditRecord trial;
                trial.mode = "full_pipeline";
                trial.removed_frame_id = source_indices[removed];
                trial.removed_source_index = source_indices[removed];
                trial.failure_reason = std::string("worker_exception: ") +
                    exception.what();
                audit.trials.push_back(std::move(trial));
            }
        }
        if (batch_end >= next_full_persist) {
            APP_LOG_EVENT_INFO(
                "calibration.hand_eye", "loo_progress", "running",
                {{"mode", "full_pipeline"},
                 {"completed", QString::number(batch_end)},
                 {"total", QString::number(all_robot_poses.size())}});
            if (!persistLooProgress(session, audit, error))
                return false;
            while (next_full_persist <= batch_end)
                next_full_persist += kPersistEveryTrials;
        }
    }
    audit.summaries.push_back(summarizeLooTrials(
        "full_pipeline", audit.trials));
    audit.completed = true;
    audit.status = "pass";
    for (const HandEyeLooSummaryAuditRecord& summary : audit.summaries) {
        if (summary.failed > 0) {
            audit.status = "complete_with_failures";
            audit.failure_reason = "one_or_more_loo_trials_failed";
            break;
        }
    }
    if (!persistLooProgress(session, audit, error))
        return false;

    for (const HandEyeLooSummaryAuditRecord& summary : audit.summaries) {
        APP_LOG_EVENT_INFO(
            "calibration.hand_eye", "loo_summary", audit.status.c_str(),
            {{"mode", QString::fromStdString(summary.mode)},
             {"attempted", QString::number(summary.attempted)},
             {"successful", QString::number(summary.successful)},
             {"failed", QString::number(summary.failed)},
             {"translation_mean_mm", QString::number(
                  summary.translation_mean_mm, 'f', 6)},
             {"translation_p95_mm", QString::number(
                  summary.translation_p95_mm, 'f', 6)},
             {"translation_max_mm", QString::number(
                  summary.translation_max_mm, 'f', 6)},
             {"rotation_mean_deg", QString::number(
                  summary.rotation_mean_deg, 'f', 6)},
             {"rotation_p95_deg", QString::number(
                  summary.rotation_p95_deg, 'f', 6)},
             {"rotation_max_deg", QString::number(
                  summary.rotation_max_deg, 'f', 6)},
             {"filter_set_change_trials", QString::number(
                  summary.trials_with_filter_set_changes)}});
    }
    return true;
}

static bool estimateHandEyeTrans(
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    std::vector<cv::Mat>& robot_poses,
    std::vector<cv::Mat>& camera_poses,
    CalibrationResult& result,
    HandEyeSolveAudit* audit,
    CalibrationRuntimeStatus* runtime_status = nullptr,
    const CalibrationRuntimeCallback& runtime_callback = {},
    const char* transform_name = "T_flange_camera") {
    result.success = false;
    result.valid_frames = 0;
    result.reproj_error = 0.0;
    result.T_flange_camera.release();

    auto publishSolve = [&](CalibrationRuntimeStage stage,
                            const std::string& detail = {}) {
        if (!runtime_status)
            return;
        publishCalibrationRuntime(runtime_callback, *runtime_status, stage, detail);
    };

    if (robot_poses.size() != camera_poses.size()) {
        result.message = "机器人位姿与标定板位姿数量不匹配";
        APP_LOG_EVENT_ERROR("calibration.hand_eye",
                            "hand_eye_quality",
                            "fail",
                            {{"reason", "pose_count_mismatch"},
                             {"robot_poses", QString::number(robot_poses.size())},
                             {"camera_poses", QString::number(camera_poses.size())}});
        return false;
    }

    const int initial_count = static_cast<int>(robot_poses.size());
    if (initial_count < kMinHandEyeFrames) {
        result.message = "有效帧数不足: " + std::to_string(initial_count) +
                         " (need >=" + std::to_string(kMinHandEyeFrames) + ")";
        APP_LOG_EVENT_ERROR("calibration.hand_eye",
                            "hand_eye_quality",
                            "fail",
                            {{"reason", "insufficient_frames"},
                             {"frames", QString::number(initial_count)},
                             {"minimum", QString::number(kMinHandEyeFrames)}});
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    publishSolve(CalibrationRuntimeStage::HandEyeInitial, "initial_solve_start");

    std::vector<HandEyeMethodResult> method_results =
        solveHandEyeMethods(robot_poses, camera_poses, "initial",
                            audit ? &audit->phase2 : nullptr);
    if (method_results.empty()) {
        result.message = "所有手眼标定方法均失败";
        APP_LOG_EVENT_ERROR("calibration.hand_eye",
                            "hand_eye_quality",
                            "fail",
                            {{"reason", "all_methods_failed"},
                             {"frames", QString::number(initial_count)}});
        std::cerr << "[autoCalib] " << result.message << std::endl;
        publishSolve(CalibrationRuntimeStage::Failed, "all_methods_failed");
        return false;
    }

    SolverConsensusResult final_consensus = evaluateSolverConsensus(
        method_results, "initial", audit ? &audit->phase2 : nullptr);
    size_t medoid_index = final_consensus.medoid_index;
    OptimizedMedoidResult final_optimized = optimizeMedoidOnce(
        method_results[medoid_index], robot_poses, camera_poses, "initial",
        audit ? &audit->phase2 : nullptr);
    if (final_optimized.stats.translation_errors.size() != robot_poses.size() ||
        !std::isfinite(final_optimized.stats.translation_rms)) {
        result.message = "初始 medoid 精化结果无效";
        APP_LOG_EVENT_ERROR("calibration.hand_eye", "medoid_refinement", "fail",
                            {{"phase", "initial"},
                             {"reason", "invalid_optimized_quality_metrics"}});
        publishSolve(CalibrationRuntimeStage::Failed,
                     "invalid_optimized_quality_metrics");
        return false;
    }
    const std::vector<double>& initial_errors =
        final_optimized.stats.translation_errors;
    const double median_error = percentile(initial_errors, 0.5);
    std::vector<double> deviations;
    deviations.reserve(initial_errors.size());
    for (const double error : initial_errors)
        deviations.push_back(std::abs(error - median_error));
    const double mad = percentile(deviations, 0.5);
    const double outlier_threshold = std::max(
        kMinimumOutlierThresholdMeters,
        median_error + kMadScale * 1.4826 * mad);
    if (audit) {
        audit->initial_consistency_errors = initial_errors;
        audit->mad_inliers.assign(initial_errors.size(), false);
        audit->mad_threshold = outlier_threshold;
    }

    publishSolve(CalibrationRuntimeStage::MadFiltering, "mad_filtering");

    std::vector<cv::Mat> filtered_robot_poses;
    std::vector<cv::Mat> filtered_camera_poses;
    filtered_robot_poses.reserve(robot_poses.size());
    filtered_camera_poses.reserve(camera_poses.size());
    for (size_t index = 0; index < initial_errors.size(); ++index) {
        if (initial_errors[index] <= outlier_threshold) {
            filtered_robot_poses.push_back(robot_poses[index]);
            filtered_camera_poses.push_back(camera_poses[index]);
            if (audit)
                audit->mad_inliers[index] = true;
        }
    }

    const int filtered_count = static_cast<int>(filtered_robot_poses.size());
    const int mad_rejected = initial_count - filtered_count;
    if (runtime_status) {
        runtime_status->mad_rejected_frames = mad_rejected;
        runtime_status->final_inliers = filtered_count;
    }
    const int minimum_inliers = std::max(
        kMinHandEyeFrames,
        static_cast<int>(std::ceil(initial_count * kMinimumInlierRatio)));
    APP_LOG_EVENT_INFO("calibration.hand_eye",
                       "hand_eye_outlier_filter",
                       filtered_count >= minimum_inliers ? "success" : "fail",
                       {{"input_frames", QString::number(initial_count)},
                        {"kept_frames", QString::number(filtered_count)},
                        {"removed_frames", QString::number(mad_rejected)},
                        {"minimum_inliers", QString::number(minimum_inliers)},
                        {"median_mm", QString::number(median_error * 1000.0, 'f', 6)},
                        {"mad_mm", QString::number(mad * 1000.0, 'f', 6)},
                        {"threshold_mm", QString::number(outlier_threshold * 1000.0, 'f', 6)},
                        {"mad_scale", QString::number(kMadScale, 'f', 2)}});
    std::cout << "[autoCalib] 鲁棒离群过滤: " << mad_rejected
              << " 帧, 阈值 " << outlier_threshold * 1000.0 << " mm, 剩余 "
              << filtered_count << "/" << initial_count << " 帧" << std::endl;
    publishSolve(CalibrationRuntimeStage::MadFiltering, "mad_complete");
    if (filtered_count < minimum_inliers) {
        result.message = "离群过滤后有效帧数不足: " +
                         std::to_string(filtered_count) + " (need >=" +
                         std::to_string(minimum_inliers) + ")";
        APP_LOG_EVENT_ERROR("calibration.hand_eye",
                            "hand_eye_outlier_filter",
                            "fail",
                            {{"reason", "insufficient_inliers"},
                             {"input_frames", QString::number(initial_count)},
                             {"kept_frames", QString::number(filtered_count)},
                             {"minimum_inliers", QString::number(minimum_inliers)},
                             {"threshold_mm", QString::number(outlier_threshold * 1000.0, 'f', 6)}});
        std::cerr << "[autoCalib] " << result.message << std::endl;
        publishSolve(CalibrationRuntimeStage::Failed, "insufficient_inliers");
        return false;
    }

    if (filtered_count != initial_count) {
        publishSolve(CalibrationRuntimeStage::HandEyeRefit, "refit_start");
        method_results = solveHandEyeMethods(
            filtered_robot_poses, filtered_camera_poses, "refit",
            audit ? &audit->phase2 : nullptr);
        if (method_results.empty()) {
            result.message = "离群过滤后所有手眼标定方法均失败";
            APP_LOG_EVENT_ERROR("calibration.hand_eye",
                                "hand_eye_quality",
                                "fail",
                                {{"reason", "refit_failed"},
                                 {"frames", QString::number(filtered_count)}});
            publishSolve(CalibrationRuntimeStage::Failed, "refit_failed");
            return false;
        }
        final_consensus = evaluateSolverConsensus(
            method_results, "refit", audit ? &audit->phase2 : nullptr);
        medoid_index = final_consensus.medoid_index;
        final_optimized = optimizeMedoidOnce(
            method_results[medoid_index], filtered_robot_poses,
            filtered_camera_poses, "refit",
            audit ? &audit->phase2 : nullptr);
        publishSolve(CalibrationRuntimeStage::HandEyeRefit, "refit_complete");
    }

    const HandEyeMethodResult& medoid = method_results[medoid_index];
    const cv::Mat& optimized_transform = final_optimized.transform;
    const HandEyeConsistencyStats& quality = final_optimized.stats;

    std::string failure_reason;
    if (!std::isfinite(quality.translation_rms) ||
        !std::isfinite(quality.rotation_rms_degrees))
        failure_reason = "invalid_optimized_quality_metrics";
    else if (!final_consensus.passed)
        failure_reason = final_consensus.failure_reason;
    else if (quality.translation_rms > kMaxTranslationRmsMeters)
        failure_reason = "translation_rms_exceeded";
    else if (quality.translation_p95 > kMaxTranslationP95Meters)
        failure_reason = "translation_p95_exceeded";
    else if (quality.rotation_rms_degrees > kMaxRotationRmsDegrees)
        failure_reason = "rotation_rms_exceeded";
    else if (quality.rotation_p95_degrees > kMaxRotationP95Degrees)
        failure_reason = "rotation_p95_exceeded";

    const bool quality_passed = failure_reason.empty();
    if (audit) {
        audit->selected_method = medoid.name;
        audit->quality_passed = quality_passed;
    }

    if (runtime_status) {
        runtime_status->selected_solver = medoid.name;
        runtime_status->consensus_available = true;
        runtime_status->consensus_passed = final_consensus.passed;
        runtime_status->consensus_max_translation_mm =
            final_consensus.max_pair_translation_meters * 1000.0;
        runtime_status->consensus_max_rotation_deg =
            final_consensus.max_pair_rotation_degrees;
        runtime_status->final_inliers = filtered_count;
        runtime_status->mad_rejected_frames = mad_rejected;
        if (std::isfinite(quality.translation_rms)) {
            runtime_status->translation_rms_mm = quality.translation_rms * 1000.0;
            runtime_status->translation_p95_mm = quality.translation_p95 * 1000.0;
            runtime_status->translation_max_mm = quality.translation_max * 1000.0;
            runtime_status->rotation_rms_deg = quality.rotation_rms_degrees;
            runtime_status->rotation_p95_deg = quality.rotation_p95_degrees;
            runtime_status->rotation_max_deg = quality.rotation_max_degrees;
        }
    }
    publishSolve(CalibrationRuntimeStage::QualityValidation,
                 quality_passed ? "quality_pass" : failure_reason);
    APP_LOG_EVENT(Info,
                  "calibration.hand_eye",
                  "hand_eye_quality",
                  quality_passed ? "success" : "fail",
                  {"method", QString::fromStdString(medoid.name)},
                  {"selection", "se3_medoid_then_single_refinement"},
                  {"input_frames", QString::number(initial_count)},
                  {"valid_frames", QString::number(filtered_count)},
                  {"translation_rms_mm", QString::number(quality.translation_rms * 1000.0, 'f', 6)},
                  {"translation_mean_mm", QString::number(quality.translation_mean * 1000.0, 'f', 6)},
                  {"translation_median_mm", QString::number(quality.translation_median * 1000.0, 'f', 6)},
                  {"translation_p95_mm", QString::number(quality.translation_p95 * 1000.0, 'f', 6)},
                  {"translation_max_mm", QString::number(quality.translation_max * 1000.0, 'f', 6)},
                  {"rotation_rms_deg", QString::number(quality.rotation_rms_degrees, 'f', 6)},
                  {"rotation_p95_deg", QString::number(quality.rotation_p95_degrees, 'f', 6)},
                  {"rotation_max_deg", QString::number(quality.rotation_max_degrees, 'f', 6)},
                  {"max_translation_rms_mm", QString::number(kMaxTranslationRmsMeters * 1000.0, 'f', 3)},
                  {"max_translation_p95_mm", QString::number(kMaxTranslationP95Meters * 1000.0, 'f', 3)},
                  {"max_rotation_rms_deg", QString::number(kMaxRotationRmsDegrees, 'f', 3)},
                  {"max_rotation_p95_deg", QString::number(kMaxRotationP95Degrees, 'f', 3)},
                  {"consensus_passed", QString::number(final_consensus.passed)},
                  {"consensus_max_translation_mm", QString::number(
                       final_consensus.max_pair_translation_meters * 1000.0, 'f', 6)},
                  {"consensus_max_rotation_deg", QString::number(
                       final_consensus.max_pair_rotation_degrees, 'f', 6)},
                  {"failure_reason", QString::fromStdString(failure_reason)},
                  {"span_x_mm", QString::number(quality.translation_span_mm[0], 'f', 3)},
                  {"span_y_mm", QString::number(quality.translation_span_mm[1], 'f', 3)},
                  {"span_z_mm", QString::number(quality.translation_span_mm[2], 'f', 3)});
    if (!quality_passed) {
        APP_LOG_EVENT_ERROR("calibration.hand_eye",
                            "hand_eye_quality_gate",
                            "fail",
                            {{"reason", QString::fromStdString(failure_reason)},
                             {"method", QString::fromStdString(medoid.name)},
                             {"valid_frames", QString::number(filtered_count)},
                             {"translation_rms_mm", QString::number(quality.translation_rms * 1000.0, 'f', 6)},
                             {"translation_p95_mm", QString::number(quality.translation_p95 * 1000.0, 'f', 6)},
                             {"rotation_rms_deg", QString::number(quality.rotation_rms_degrees, 'f', 6)},
                             {"rotation_p95_deg", QString::number(quality.rotation_p95_degrees, 'f', 6)},
                             {"consensus_max_translation_mm", QString::number(
                                  final_consensus.max_pair_translation_meters * 1000.0,
                                  'f', 6)},
                             {"consensus_max_rotation_deg", QString::number(
                                  final_consensus.max_pair_rotation_degrees, 'f', 6)},
                             {"consensus_translation_limit_mm", QString::number(
                                  kMaxSolverPairTranslationMeters * 1000.0, 'f', 3)},
                             {"consensus_rotation_limit_deg", QString::number(
                                  kMaxSolverPairRotationDegrees, 'f', 3)}});
        std::ostringstream message;
        message << "手眼标定质量未通过: " << failure_reason
                << ", RMS=" << quality.translation_rms * 1000.0
                << "mm, P95=" << quality.translation_p95 * 1000.0
                << "mm, rotation_RMS=" << quality.rotation_rms_degrees << "deg";
        result.message = message.str();
        std::cerr << "[autoCalib] " << result.message << std::endl;
        publishSolve(CalibrationRuntimeStage::Failed, failure_reason);
        return false;
    }

    robot_poses.swap(filtered_robot_poses);
    camera_poses.swap(filtered_camera_poses);
    result.T_flange_camera = optimized_transform.clone();
    result.camera_matrix = camera_matrix.clone();
    result.dist_coeffs = dist_coeffs.clone();
    result.reproj_error = quality.translation_rms;
    result.valid_frames = filtered_count;
    result.success = true;
    std::ostringstream message;
    message << "标定成功, 有效帧: " << filtered_count
            << ", 平移 RMS: " << quality.translation_rms * 1000.0
            << " mm, P95: " << quality.translation_p95 * 1000.0
            << " mm, 旋转 RMS: " << quality.rotation_rms_degrees
            << " deg (medoid: " << medoid.name << ")";
    result.message = message.str();
    std::cout << "[autoCalib] " << result.message << std::endl;
    std::cout << "[autoCalib] " << transform_name << ":\n"
              << result.T_flange_camera << std::endl;
    return true;
}}

// ============================================================
// 手眼标定主入口（从磁盘读取数据）
// ============================================================
bool solveEyeToHandPoses(const std::vector<cv::Mat>& robot_poses,
                        const std::vector<cv::Mat>& camera_poses,
                        EyeToHandResult& result) {
    result.success = false;
    result.T_base_camera.release();
    result.valid_frames = 0;
    result.reproj_error = 0.0;
    result.message.clear();
    result.quality = CalibrationRuntimeStatus{};
    auto validTransform = [](const cv::Mat& t) {
        if (t.rows != 4 || t.cols != 4 || t.type() != CV_64FC1 || !cv::checkRange(t))
            return false;
        const cv::Mat r = t(cv::Rect(0, 0, 3, 3));
        const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
        const cv::Mat bottom = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
        return cv::norm(r.t() * r - identity) < 1e-6 &&
            std::abs(cv::determinant(r) - 1.0) < 1e-6 &&
            cv::norm(t.row(3) - bottom) < 1e-9;
    };
    if (robot_poses.size() != camera_poses.size() ||
        robot_poses.size() < static_cast<size_t>(kMinHandEyeFrames) ||
        !std::all_of(robot_poses.begin(), robot_poses.end(), validTransform) ||
        !std::all_of(camera_poses.begin(), camera_poses.end(), validTransform)) {
        result.message = "位姿无效、数量不匹配或有效帧少于12";
        return false;
    }
    // At least two independent rotation axes are needed. Reject static,
    // translation-only and single-axis datasets before invoking the solvers.
    cv::Mat rotations(3, static_cast<int>(robot_poses.size()), CV_64F);
    const cv::Mat r0 = robot_poses.front()(cv::Rect(0, 0, 3, 3));
    for (size_t i = 0; i < robot_poses.size(); ++i) {
        cv::Mat relative = r0.t() * robot_poses[i](cv::Rect(0, 0, 3, 3));
        cv::Mat rv;
        cv::Rodrigues(relative, rv);
        rv.copyTo(rotations.col(static_cast<int>(i)));
    }
    cv::Mat singular;
    cv::SVD::compute(rotations, singular);
    if (singular.at<double>(1) < 1e-3) {
        result.message = "旋转激励不足：需要至少两个独立旋转轴";
        return false;
    }
    // The shared equation is robot_i * X * camera_i = constant.
    // Substituting inv(G_i) yields X=T_base_camera, with NO output inversion.
    std::vector<cv::Mat> inverse_robot, cameras = camera_poses;
    for (const auto& pose : robot_poses)
        inverse_robot.push_back(calib_transform::invertTransform(pose));
    CalibrationResult solved;
    HandEyeSolveAudit audit;
    result.quality.pnp_valid_frames = static_cast<int>(robot_poses.size());
    const bool ok = estimateHandEyeTrans(result.camera_matrix, result.dist_coeffs,
        inverse_robot, cameras, solved, &audit, &result.quality, {}, "T_base_camera");
    result.message = solved.message;
    if (!ok) return false;
    if (!validTransform(solved.T_flange_camera)) {
        result.message = "眼在手外求解输出不是有效刚体变换";
        return false;
    }
    result.T_base_camera = solved.T_flange_camera.clone();
    result.valid_frames = solved.valid_frames;
    result.reproj_error = solved.reproj_error;
    result.success = true;
    return true;
}

bool calibrateHandEyeFromDisk(
    const std::string& save_dir,
    cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const CharucoBoardConfig& board_config,
    CalibrationResult& result,
    const std::string& output_yaml,
    CalibrationSession* session,
    const std::string& intrinsic_source_file,
    bool enable_loo_diagnostics,
    CalibrationRuntimeStatus* runtime_status,
    const CalibrationRuntimeCallback& runtime_callback) {

    result.success = false;
    result.valid_frames = 0;
    result.reproj_error = 0.0;
    result.message.clear();
    result.session_dir.clear();
    result.T_flange_camera.release();

    std::unique_ptr<CalibrationSession> owned_session;
    std::string data_directory = save_dir;
    if (!session) {
        CalibrationSessionOptions options;
        options.base_directory = auditBaseDirectoryForSource(save_dir);
        options.mode = "offline_eye_in_hand";
        options.source_data_directory = save_dir;
        options.intrinsic_source = "provided_offline";
        options.intrinsic_source_file = intrinsic_source_file;
        options.board = board_config;
        options.camera_matrix = camera_matrix.clone();
        options.dist_coeffs = dist_coeffs.clone();
        populateSourceSessionLineage(save_dir, options);
        std::string session_error;
        owned_session = CalibrationSession::create(options, session_error);
        if (!owned_session) {
            result.message = "无法创建标定审计会话: " + session_error;
            return false;
        }
        session = owned_session.get();
        result.session_dir = session->directory();
        if (!session->importLegacyDataset(save_dir, session_error)) {
            result.message = "无法导入离线数据集: " + session_error;
            session->finalizeFailure("dataset_import", "dataset_import_failed",
                                     result.message);
            return false;
        }
        data_directory = session->framesDirectory();
    } else {
        result.session_dir = session->directory();
    }

    auto failSession = [&](const std::string& stage,
                           const std::string& reason) -> bool {
        if (session) {
            session->setStage(stage);
            session->finalizeFailure(stage, reason, result.message);
        }
        if (runtime_status) {
            const CalibrationRuntimeStage terminal =
                (reason == "cancelled")
                    ? CalibrationRuntimeStage::Cancelled
                    : CalibrationRuntimeStage::Failed;
            if (runtime_status->publication_state == "publishing")
                runtime_status->publication_state = "failed";
            publishCalibrationRuntime(runtime_callback, *runtime_status,
                                      terminal, reason);
        }
        return false;
    };
    session->setStage("hand_eye_input");
    session->setIntrinsics(camera_matrix, dist_coeffs,
                           owned_session ? "provided_offline" : "effective_runtime");
    if (runtime_status) {
        runtime_status->calibration_session_id = session->id();
        runtime_status->calibration_session_dir = session->directory();
        if (runtime_status->publication_state.empty())
            runtime_status->publication_state = "not_started";
    }

    if (camera_matrix.rows != 3 || camera_matrix.cols != 3 ||
        dist_coeffs.empty() || !cv::checkRange(camera_matrix) ||
        !cv::checkRange(dist_coeffs) || camera_matrix.at<double>(0, 0) <= 0.0 ||
        camera_matrix.at<double>(1, 1) <= 0.0) {
        result.message = "相机内参无效，拒绝进行手眼标定";
        APP_LOG_EVENT_ERROR("calibration.hand_eye",
                            "hand_eye_input",
                            "fail",
                            {{"reason", "invalid_intrinsics"},
                             {"matrix_rows", QString::number(camera_matrix.rows)},
                             {"matrix_cols", QString::number(camera_matrix.cols)},
                             {"distortion_count", QString::number(dist_coeffs.total())}});
        return failSession("hand_eye_input", "invalid_intrinsics");
    }

    CharucoDetector detector(board_config);
    detector.setCameraParams(camera_matrix, dist_coeffs);

    // 读取法兰位姿
    std::vector<cv::Mat> flange_poses;
    std::string robot_pose_file = data_directory + "/rob_pos.csv";
    if (session && std::filesystem::exists(
            session->robotPosesDirectory() + "/rob_pos.csv")) {
        robot_pose_file = session->robotPosesDirectory() + "/rob_pos.csv";
    }
    if (!loadRobotPosesFromCSV(robot_pose_file, flange_poses)) {
        result.message = "无法加载 " + robot_pose_file;
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return failSession("hand_eye_input", "robot_pose_load_failed");
    }

    // 扫描图像文件
    std::vector<std::string> img_files;
    cv::Size image_size;
    if (!loadImagesFromDisk(data_directory, img_files, image_size)) {
        result.message = "无法加载标定图像: " + data_directory;
        return failSession("hand_eye_input", "image_load_failed");
    }
    session->setImageSize(image_size.width, image_size.height);

    if (img_files.size() != flange_poses.size()) {
        result.message = "图像数量 (" + std::to_string(img_files.size()) +
                         ") 与机器人位姿数量 (" +
                         std::to_string(flange_poses.size()) + ") 不匹配";
        APP_LOG_EVENT_ERROR("calibration.hand_eye",
                            "hand_eye_input",
                            "fail",
                            {{"reason", "image_pose_count_mismatch"},
                             {"images", QString::number(img_files.size())},
                             {"robot_poses", QString::number(flange_poses.size())}});
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return failSession("hand_eye_input", "image_pose_count_mismatch");
    }

    const size_t n = img_files.size();
    const auto& board_points = detector.getChessboardCorners3D();

    struct FrameRecord {
        size_t source_index;
        cv::Mat robot_pose;
        cv::Mat camera_pose;
        double error;
    };
    std::vector<FrameRecord> frames;
    frames.reserve(n);
    std::vector<CalibrationFrameAudit> frame_audits(n);
    size_t unreadable_images = 0;
    size_t pose_detection_failures = 0;
    size_t invalid_correspondences = 0;
    double reprojection_error_sum = 0.0;

    if (runtime_status) {
        publishCalibrationRuntime(runtime_callback, *runtime_status,
                                  CalibrationRuntimeStage::PnP, "pnp_start");
    }

    for (size_t i = 0; i < n; i++) {
        CalibrationFrameAudit& audit = frame_audits[i];
        audit.frame_id = static_cast<int>(i);
        audit.source_index = static_cast<int>(i);
        audit.image_file = "frames/" +
            std::filesystem::path(img_files[i]).filename().generic_string();
        audit.image_sha256 = CalibrationSession::sha256File(img_files[i]);
        audit.T_base_flange = flange_poses[i].clone();

        std::ostringstream raw_name;
        raw_name << "frame_" << std::setw(4) << std::setfill('0') << i << ".json";
        const std::string raw_pose_path =
            session->robotPosesDirectory() + "/" + raw_name.str();
        if (std::filesystem::exists(raw_pose_path)) {
            cv::FileStorage raw_pose(raw_pose_path, cv::FileStorage::READ);
            std::string image_timestamp;
            std::string robot_timestamp;
            raw_pose["timing"]["image_host_realtime_ns"] >> image_timestamp;
            raw_pose["timing"]["robot_host_realtime_ns"] >> robot_timestamp;
            try {
                if (!image_timestamp.empty())
                    audit.image_timestamp_ns = std::stoull(image_timestamp);
                if (!robot_timestamp.empty())
                    audit.robot_timestamp_ns = std::stoull(robot_timestamp);
            } catch (const std::exception&) {
                audit.reject_reasons.push_back("invalid_audit_timestamp");
            }
        }

        cv::Mat img = cv::imread(img_files[i]);
        if (img.empty()) {
            ++unreadable_images;
            audit.reject_stage = "image_read";
            audit.reject_reasons.push_back("image_unreadable");
            continue;
        }
        audit.image_read_pass = true;
        audit.image_width = img.cols;
        audit.image_height = img.rows;
        cv::Mat rvec, tvec;
        std::vector<cv::Point2f> corners;
        std::vector<int> ids;

        // 与采集阶段使用同一个 12 角点门限，避免“采集成功但求解大量丢帧”。
        if (!detector.detectPose(img, rvec, tvec, corners, ids, kMinHandEyeCorners)) {
            ++pose_detection_failures;
            std::vector<cv::Point2f> diagnostic_corners;
            std::vector<int> diagnostic_ids;
            const bool charuco_detected = detector.detectCorners(
                img, diagnostic_corners, diagnostic_ids);
            audit.corner_count = static_cast<int>(diagnostic_ids.size());
            audit.charuco_pass = charuco_detected &&
                diagnostic_ids.size() >= static_cast<size_t>(kMinHandEyeCorners);
            if (audit.charuco_pass) {
                audit.reject_stage = "pnp";
                audit.reject_reasons.push_back("pose_estimation_or_reprojection_filter_failed");
            } else {
                audit.reject_stage = "charuco";
                audit.reject_reasons.push_back(charuco_detected
                    ? "insufficient_corners" : "detection_failed");
            }
            continue;
        }
        audit.charuco_pass = true;
        audit.corner_count = static_cast<int>(ids.size());
        if (corners.size() >= 3 && img.total() > 0) {
            std::vector<cv::Point2f> hull;
            cv::convexHull(corners, hull);
            audit.coverage = std::abs(cv::contourArea(hull)) /
                static_cast<double>(img.total());
        }

        std::vector<cv::Point3f> pts3;
        std::vector<cv::Point2f> pts2;
        for (size_t j = 0; j < ids.size(); j++) {
            int id = ids[j];
            if (id >= 0 && (size_t)id < board_points.size()) {
                pts3.push_back(board_points[id]);
                pts2.push_back(corners[j]);
            }
        }
        if (pts3.size() < static_cast<size_t>(kMinHandEyeCorners)) {
            ++invalid_correspondences;
            audit.reject_stage = "pnp";
            audit.reject_reasons.push_back("insufficient_valid_correspondences");
            continue;
        }

        std::vector<cv::Point2f> projected;
        cv::projectPoints(pts3, rvec, tvec, camera_matrix, dist_coeffs, projected);
        double squared_error_sum = 0.0;
        std::vector<double> pixel_errors;
        pixel_errors.reserve(projected.size());
        for (size_t j = 0; j < projected.size(); j++) {
            const double pixel_error = cv::norm(projected[j] - pts2[j]);
            pixel_errors.push_back(pixel_error);
            squared_error_sum += pixel_error * pixel_error;
        }

        const double err = std::sqrt(squared_error_sum / projected.size());
        audit.pnp_pass = true;
        audit.reprojection_rms_px = err;
        audit.reprojection_p95_px = percentile(pixel_errors, 0.95);
        audit.T_camera_board = calib_transform::rvecTvecToTransform(rvec, tvec);
        reprojection_error_sum += err;
        frames.push_back({i, flange_poses[i], audit.T_camera_board, err});
    }

    session->setFrames(frame_audits);

    const size_t detected_frames = frames.size();
    const double mean_reprojection_error = detected_frames > 0
        ? reprojection_error_sum / detected_frames
        : std::numeric_limits<double>::quiet_NaN();
    if (runtime_status) {
        runtime_status->pnp_valid_frames = static_cast<int>(detected_frames);
        publishCalibrationRuntime(runtime_callback, *runtime_status,
                                  CalibrationRuntimeStage::PnP, "pnp_complete");
    }
    APP_LOG_EVENT_INFO("calibration.hand_eye",
                       "hand_eye_input",
                       detected_frames >= static_cast<size_t>(kMinHandEyeFrames) ? "success" : "fail",
                       {{"images", QString::number(img_files.size())},
                        {"robot_poses", QString::number(flange_poses.size())},
                        {"valid_frames", QString::number(detected_frames)},
                        {"unreadable_images", QString::number(unreadable_images)},
                        {"pose_detection_failures", QString::number(pose_detection_failures)},
                        {"invalid_correspondences", QString::number(invalid_correspondences)},
                        {"minimum_corners", QString::number(kMinHandEyeCorners)},
                        {"mean_reprojection_px", QString::number(mean_reprojection_error, 'f', 6)},
                        {"fx", QString::number(camera_matrix.at<double>(0, 0), 'f', 6)},
                        {"fy", QString::number(camera_matrix.at<double>(1, 1), 'f', 6)},
                        {"cx", QString::number(camera_matrix.at<double>(0, 2), 'f', 6)},
                        {"cy", QString::number(camera_matrix.at<double>(1, 2), 'f', 6)}});
    std::cout << "[autoCalib] 手眼位姿检测: " << detected_frames << "/" << n
              << " 帧, 平均重投影误差 " << mean_reprojection_error << " px" << std::endl;

    std::vector<cv::Mat> robot_poses, camera_poses;
    robot_poses.reserve(detected_frames);
    camera_poses.reserve(detected_frames);
    for (const FrameRecord& frame : frames) {
        robot_poses.push_back(frame.robot_pose);
        camera_poses.push_back(frame.camera_pose);
    }
    const std::vector<cv::Mat> all_detected_robot_poses = robot_poses;
    const std::vector<cv::Mat> all_detected_camera_poses = camera_poses;
    std::vector<int> detected_source_indices;
    detected_source_indices.reserve(frames.size());
    for (const FrameRecord& frame : frames)
        detected_source_indices.push_back(static_cast<int>(frame.source_index));

    session->setStage("solve_and_validate");
    HandEyeSolveAudit solve_audit;
    const bool solved = estimateHandEyeTrans(
        camera_matrix, dist_coeffs, robot_poses, camera_poses, result,
        &solve_audit, runtime_status, runtime_callback);
    std::string audit_error;
    if (!session->setHandEyePhase2Audit(solve_audit.phase2, audit_error)) {
        result.success = false;
        result.message = "Phase 2 审计结果持久化失败: " + audit_error;
        return failSession("solver_diagnostics",
                           "phase2_audit_persist_failed");
    }
    if (!solved) {
        for (size_t index = 0; index < frames.size(); ++index) {
            CalibrationFrameAudit& audit = frame_audits[frames[index].source_index];
            if (index < solve_audit.initial_consistency_errors.size())
                audit.consistency_error_mm =
                    solve_audit.initial_consistency_errors[index] * 1000.0;
            if (index < solve_audit.mad_inliers.size()) {
                audit.mad_filter_pass = solve_audit.mad_inliers[index];
                audit.quality_gate_pass = solve_audit.quality_passed &&
                                          audit.mad_filter_pass;
                if (!audit.mad_filter_pass) {
                    audit.reject_stage = "mad_filter";
                    audit.reject_reasons.push_back("translation_consistency_outlier");
                }
            }
        }
        session->setFrames(frame_audits);
        APP_LOG_EVENT_ERROR("calibration.hand_eye",
                            "hand_eye_pipeline",
                            "fail",
                            {{"stage", "solve_and_validate"},
                             {"valid_frames", QString::number(robot_poses.size())},
                             {"message", QString::fromStdString(result.message)}});
        return failSession("solve_and_validate", "hand_eye_quality_failed");
    }

    for (size_t index = 0; index < frames.size(); ++index) {
        CalibrationFrameAudit& audit = frame_audits[frames[index].source_index];
        if (index < solve_audit.initial_consistency_errors.size())
            audit.consistency_error_mm =
                solve_audit.initial_consistency_errors[index] * 1000.0;
        audit.mad_filter_pass = index < solve_audit.mad_inliers.size() &&
                                solve_audit.mad_inliers[index];
        audit.quality_gate_pass = solve_audit.quality_passed &&
                                  audit.mad_filter_pass;
        audit.final_accepted = audit.mad_filter_pass && solve_audit.quality_passed;
        if (!audit.mad_filter_pass) {
            audit.reject_stage = "mad_filter";
            audit.reject_reasons.push_back("translation_consistency_outlier");
        }
    }
    session->setFrames(frame_audits);

    if (enable_loo_diagnostics) {
        session->setStage("loo_stability");
        HandEyePhase3Audit phase3_audit;
        std::string phase3_error;
        if (!runPhase3LooDiagnostics(
                all_detected_robot_poses, all_detected_camera_poses,
                detected_source_indices, solve_audit.mad_inliers,
                result.T_flange_camera, solve_audit.selected_method,
                session, phase3_audit,
                phase3_error)) {
            result.success = false;
            result.message = "Phase 3 LOO 诊断失败: " + phase3_error;
            return failSession("loo_stability",
                               "phase3_loo_diagnostics_failed");
        }
    }

    std::string finalize_error;
    if (runtime_status) {
        runtime_status->publication_state =
            output_yaml.empty() ? "not_published" : "publishing";
        publishCalibrationRuntime(runtime_callback, *runtime_status,
                                  CalibrationRuntimeStage::Publishing,
                                  output_yaml.empty()
                                      ? "skip_production_publish"
                                      : "publishing_start");
    }
    if (!session->finalizeSuccess(result.T_flange_camera, result.camera_matrix,
                                  result.dist_coeffs, result.reproj_error * 1000.0,
                                  result.valid_frames, result.message, output_yaml,
                                  finalize_error)) {
        result.success = false;
        result.message = "标定结果发布或审计会话封存失败: " + finalize_error;
        if (runtime_status)
            runtime_status->publication_state = "failed";
        return failSession("output", "session_seal_or_atomic_publish_failed");
    }

    if (runtime_status) {
        runtime_status->publication_state =
            output_yaml.empty() ? "not_published" : "published";
        publishCalibrationRuntime(runtime_callback, *runtime_status,
                                  CalibrationRuntimeStage::Completed,
                                  "calibration_complete");
    }

    std::cout << "\n========== 标定结果 ==========" << std::endl;
    std::cout << "手眼矩阵 T_flange_camera:\n" << result.T_flange_camera << std::endl;
    std::cout << "有效帧数: " << result.valid_frames << std::endl;
    std::cout << "平移一致性 RMS: " << (result.reproj_error * 1000.0) << " mm" << std::endl;
    std::cout << result.message << std::endl;
    if (!output_yaml.empty())
        std::cout << "结果已保存到: " << output_yaml << std::endl;

    return true;
}
