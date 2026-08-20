#include "semi_auto_eye_in_hand_calibration.hpp"

#include "camera_calibration.hpp"           // calibrateCameraFromDisk, loadImagesFromDisk
#include "calibration_utils.hpp"            // executeCollection, PoseCandidate
#include "hand_eye_gui/DiagnosticLogger.hpp"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numeric>
#include <sstream>

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
    std::vector<PoseCandidate>& candidates) {

    candidates.clear();
    double euler[6], joint[6];

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

        if (convertFlangeOulePos2Joint(robot_handle, euler, joint)) {
            PoseCandidate cand;
            std::copy(joint, joint + 6, cand.joint);
            std::fill(cand.axis_angle, cand.axis_angle + 6, 0.0);
            candidates.push_back(cand);
        }
    }}}}}}

    std::cout << "[semiAutoEyeInHand] 位姿生成: " << candidates.size() << "/"
              << (config.offset_x.size() * config.offset_y.size() *
                  config.offset_z.size() * config.offset_rx.size() *
                  config.offset_ry.size() * config.offset_rz.size())
              << " IK 可达" << std::endl;

    return !candidates.empty();
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

    // ---- Phase 1: 获取示教原点 ----
    double tcp_aa[6], flangeEuleInit[6];
    if (cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa) != sucess) {
        result.message = "获取机器人位姿失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return false;
    }
    if (!convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEuleInit)) {
        result.message = "轴角转法兰欧拉失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return false;
    }
    std::cout << "[semiAutoEyeInHand] 示教原点 (法兰欧拉): "
              << flangeEuleInit[0] << ", " << flangeEuleInit[1] << ", " << flangeEuleInit[2] << " mm, "
              << flangeEuleInit[3] << ", " << flangeEuleInit[4] << ", " << flangeEuleInit[5] << " deg"
              << std::endl;

    // ---- Phase 2: 生成候选位姿 ----
    std::vector<PoseCandidate> candidates;
    if (!generateEyeInHandCandidates(robot_handle, flangeEuleInit, pose_config, candidates)) {
        result.message = "无 IK 可达位姿";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return false;
    }

    // ---- Phase 3: 逐点采集 ----
    if (!executeCollection(robot_handle, pchRobotIP, camera, detector,
                            candidates, save_dir, callback)) {
        result.message = "采集失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return false;
    }

    {
        int captured = countCapturedFrames(save_dir);
        if (captured < pose_config.min_valid_frames) {
            result.message = "有效帧数不足: " + std::to_string(captured) +
                             " (need >= " + std::to_string(pose_config.min_valid_frames) + ")";
            std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
            return false;
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
        if (!calibrateCameraFromDisk(save_dir, detector, cam_mat, dist)) {
            result.message = "相机内参标定失败";
            std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
            return false;
        }
        std::cout << "[semiAutoEyeInHand] 相机内参已标定" << std::endl;
    }

    // ---- Phase 5: 手眼标定 ----
    if (!calibrateHandEyeFromDisk(save_dir, cam_mat, dist, board_config,
                                   result, output_yaml)) {
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

    std::cout << "\n========== 半自动眼在手内离线标定 ==========" << std::endl;
    std::cout << "[semiAutoEyeInHand] 数据目录: " << save_dir << std::endl;

    CharucoDetector detector(board_config);

    // ---- Phase 1: 相机内参标定 ----
    cv::Mat cam_mat, dist;
    if (!camera_matrix.empty() && !dist_coeffs.empty()) {
        cam_mat = camera_matrix.getMat().clone();
        dist = dist_coeffs.getMat().clone();
        std::cout << "[semiAutoEyeInHand] 使用已知相机内参" << std::endl;
    } else {
        if (!calibrateCameraFromDisk(save_dir, detector, cam_mat, dist)) {
            result.message = "相机内参标定失败";
            std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
            return false;
        }
        std::cout << "[semiAutoEyeInHand] 相机内参已标定" << std::endl;
    }

    // ---- Phase 2: 手眼标定 ----
    if (!calibrateHandEyeFromDisk(save_dir, cam_mat, dist, board_config, result, output_yaml)) {
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
    const std::vector<cv::Mat>& camera_poses) {

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
            if (iter % 10 == 0)
                std::cout << "[refine]   iter " << iter << ": "
                          << (cost * 1e6) << " mm²" << std::endl;
            if (std::abs(prev_cost - cost) < 1e-14 || prev_cost == 0.0)
                break;
            prev_cost = cost;
        } else {
            lambda *= 3.0;
        }
    }

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

static std::vector<HandEyeMethodResult> solveHandEyeMethods(
    const std::vector<cv::Mat>& robot_poses,
    const std::vector<cv::Mat>& camera_poses,
    const char* phase) {
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
        cv::Mat rotation_camera_to_gripper, translation_camera_to_gripper;
        try {
            cv::calibrateHandEye(robot_rotations, robot_translations,
                                 camera_rotations, camera_translations,
                                 rotation_camera_to_gripper,
                                 translation_camera_to_gripper,
                                 method.second);
        } catch (const cv::Exception& exception) {
            APP_LOG_EVENT_WARNING("calibration.hand_eye",
                                  "hand_eye_method",
                                  "fail",
                                  {{"phase", phase},
                                   {"method", method.first},
                                   {"frames", QString::number(robot_poses.size())},
                                   {"reason", "opencv_exception"},
                                   {"message", QString::fromUtf8(exception.what())}});
            continue;
        }
        if (rotation_camera_to_gripper.empty() || translation_camera_to_gripper.empty() ||
            !cv::checkRange(rotation_camera_to_gripper) ||
            !cv::checkRange(translation_camera_to_gripper)) {
            APP_LOG_EVENT_WARNING("calibration.hand_eye",
                                  "hand_eye_method",
                                  "fail",
                                  {{"phase", phase},
                                   {"method", method.first},
                                   {"frames", QString::number(robot_poses.size())},
                                   {"reason", "non_finite_output"}});
            continue;
        }

        cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
        rotation_camera_to_gripper.copyTo(transform(cv::Rect(0, 0, 3, 3)));
        translation_camera_to_gripper.copyTo(transform(cv::Rect(3, 0, 1, 3)));
        transform = refineHandEyePoseConsistency(transform, robot_poses, camera_poses);
        HandEyeConsistencyStats stats =
            evaluateHandEyeConsistency(transform, robot_poses, camera_poses);
        if (!std::isfinite(stats.translation_rms) ||
            !std::isfinite(stats.rotation_rms_degrees)) {
            APP_LOG_EVENT_WARNING("calibration.hand_eye",
                                  "hand_eye_method",
                                  "fail",
                                  {{"phase", phase},
                                   {"method", method.first},
                                   {"frames", QString::number(robot_poses.size())},
                                   {"reason", "invalid_quality_metrics"}});
            continue;
        }

        APP_LOG_EVENT_INFO("calibration.hand_eye",
                           "hand_eye_method",
                           "success",
                           {{"phase", phase},
                            {"method", method.first},
                            {"frames", QString::number(robot_poses.size())},
                            {"translation_rms_mm", QString::number(stats.translation_rms * 1000.0, 'f', 6)},
                            {"translation_p95_mm", QString::number(stats.translation_p95 * 1000.0, 'f', 6)},
                            {"rotation_rms_deg", QString::number(stats.rotation_rms_degrees, 'f', 6)},
                            {"rotation_p95_deg", QString::number(stats.rotation_p95_degrees, 'f', 6)}});
        std::cout << "[autoCalib] " << method.first
                  << " -> 平移一致性 RMS: " << stats.translation_rms * 1000.0
                  << " mm, P95: " << stats.translation_p95 * 1000.0
                  << " mm, 旋转 RMS: " << stats.rotation_rms_degrees << " deg" << std::endl;
        results.push_back({method.first, method.second, transform, std::move(stats)});
    }
    return results;
}

static size_t bestMethodIndex(const std::vector<HandEyeMethodResult>& results) {
    return static_cast<size_t>(std::distance(
        results.begin(), std::min_element(
            results.begin(), results.end(),
            [](const HandEyeMethodResult& lhs, const HandEyeMethodResult& rhs) {
                return lhs.stats.translation_rms < rhs.stats.translation_rms;
            })));
}

static bool estimateHandEyeTrans(
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    std::vector<cv::Mat>& robot_poses,
    std::vector<cv::Mat>& camera_poses,
    CalibrationResult& result) {
    result.success = false;
    result.valid_frames = 0;
    result.reproj_error = 0.0;
    result.T_flange_camera.release();

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

    std::vector<HandEyeMethodResult> method_results =
        solveHandEyeMethods(robot_poses, camera_poses, "initial");
    if (method_results.empty()) {
        result.message = "所有手眼标定方法均失败";
        APP_LOG_EVENT_ERROR("calibration.hand_eye",
                            "hand_eye_quality",
                            "fail",
                            {{"reason", "all_methods_failed"},
                             {"frames", QString::number(initial_count)}});
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    size_t best_index = bestMethodIndex(method_results);
    const std::vector<double>& initial_errors =
        method_results[best_index].stats.translation_errors;
    const double median_error = percentile(initial_errors, 0.5);
    std::vector<double> deviations;
    deviations.reserve(initial_errors.size());
    for (const double error : initial_errors)
        deviations.push_back(std::abs(error - median_error));
    const double mad = percentile(deviations, 0.5);
    const double outlier_threshold = std::max(
        kMinimumOutlierThresholdMeters,
        median_error + kMadScale * 1.4826 * mad);

    std::vector<cv::Mat> filtered_robot_poses;
    std::vector<cv::Mat> filtered_camera_poses;
    filtered_robot_poses.reserve(robot_poses.size());
    filtered_camera_poses.reserve(camera_poses.size());
    for (size_t index = 0; index < initial_errors.size(); ++index) {
        if (initial_errors[index] <= outlier_threshold) {
            filtered_robot_poses.push_back(robot_poses[index]);
            filtered_camera_poses.push_back(camera_poses[index]);
        }
    }

    const int filtered_count = static_cast<int>(filtered_robot_poses.size());
    const int minimum_inliers = std::max(
        kMinHandEyeFrames,
        static_cast<int>(std::ceil(initial_count * kMinimumInlierRatio)));
    APP_LOG_EVENT_INFO("calibration.hand_eye",
                       "hand_eye_outlier_filter",
                       filtered_count >= minimum_inliers ? "success" : "fail",
                       {{"input_frames", QString::number(initial_count)},
                        {"kept_frames", QString::number(filtered_count)},
                        {"removed_frames", QString::number(initial_count - filtered_count)},
                        {"minimum_inliers", QString::number(minimum_inliers)},
                        {"median_mm", QString::number(median_error * 1000.0, 'f', 6)},
                        {"mad_mm", QString::number(mad * 1000.0, 'f', 6)},
                        {"threshold_mm", QString::number(outlier_threshold * 1000.0, 'f', 6)},
                        {"mad_scale", QString::number(kMadScale, 'f', 2)}});
    std::cout << "[autoCalib] 鲁棒离群过滤: " << initial_count - filtered_count
              << " 帧, 阈值 " << outlier_threshold * 1000.0 << " mm, 剩余 "
              << filtered_count << "/" << initial_count << " 帧" << std::endl;
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
        return false;
    }

    if (filtered_count != initial_count) {
        method_results = solveHandEyeMethods(
            filtered_robot_poses, filtered_camera_poses, "refit");
        if (method_results.empty()) {
            result.message = "离群过滤后所有手眼标定方法均失败";
            APP_LOG_EVENT_ERROR("calibration.hand_eye",
                                "hand_eye_quality",
                                "fail",
                                {{"reason", "refit_failed"},
                                 {"frames", QString::number(filtered_count)}});
            return false;
        }
        best_index = bestMethodIndex(method_results);
    }

    const HandEyeMethodResult& best = method_results[best_index];
    const HandEyeConsistencyStats& quality = best.stats;
    std::string failure_reason;
    if (quality.translation_rms > kMaxTranslationRmsMeters)
        failure_reason = "translation_rms_exceeded";
    else if (quality.translation_p95 > kMaxTranslationP95Meters)
        failure_reason = "translation_p95_exceeded";
    else if (quality.rotation_rms_degrees > kMaxRotationRmsDegrees)
        failure_reason = "rotation_rms_exceeded";
    else if (quality.rotation_p95_degrees > kMaxRotationP95Degrees)
        failure_reason = "rotation_p95_exceeded";

    const bool quality_passed = failure_reason.empty();
    APP_LOG_EVENT(Info,
                  "calibration.hand_eye",
                  "hand_eye_quality",
                  quality_passed ? "success" : "fail",
                  {"method", QString::fromStdString(best.name)},
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
                  {"failure_reason", QString::fromStdString(failure_reason)},
                  {"span_x_mm", QString::number(quality.translation_span_mm[0], 'f', 3)},
                  {"span_y_mm", QString::number(quality.translation_span_mm[1], 'f', 3)},
                  {"span_z_mm", QString::number(quality.translation_span_mm[2], 'f', 3)});
    if (!quality_passed) {
        APP_LOG_EVENT_ERROR("calibration.hand_eye",
                            "hand_eye_quality_gate",
                            "fail",
                            {{"reason", QString::fromStdString(failure_reason)},
                             {"method", QString::fromStdString(best.name)},
                             {"valid_frames", QString::number(filtered_count)},
                             {"translation_rms_mm", QString::number(quality.translation_rms * 1000.0, 'f', 6)},
                             {"translation_p95_mm", QString::number(quality.translation_p95 * 1000.0, 'f', 6)},
                             {"rotation_rms_deg", QString::number(quality.rotation_rms_degrees, 'f', 6)},
                             {"rotation_p95_deg", QString::number(quality.rotation_p95_degrees, 'f', 6)}});
        std::ostringstream message;
        message << "手眼标定质量未通过: " << failure_reason
                << ", RMS=" << quality.translation_rms * 1000.0
                << "mm, P95=" << quality.translation_p95 * 1000.0
                << "mm, rotation_RMS=" << quality.rotation_rms_degrees << "deg";
        result.message = message.str();
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    robot_poses.swap(filtered_robot_poses);
    camera_poses.swap(filtered_camera_poses);
    result.T_flange_camera = best.transform.clone();
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
            << " deg (最优: " << best.name << ")";
    result.message = message.str();
    std::cout << "[autoCalib] " << result.message << std::endl;
    std::cout << "[autoCalib] T_flange_camera:\n" << result.T_flange_camera << std::endl;
    return true;
}}

// ============================================================
// 手眼标定主入口（从磁盘读取数据）
// ============================================================
bool calibrateHandEyeFromDisk(
    const std::string& save_dir,
    cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const CharucoBoardConfig& board_config,
    CalibrationResult& result,
    const std::string& output_yaml) {

    result.success = false;
    result.valid_frames = 0;
    result.reproj_error = 0.0;
    result.message.clear();
    result.T_flange_camera.release();

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
        return false;
    }

    CharucoDetector detector(board_config);
    detector.setCameraParams(camera_matrix, dist_coeffs);

    // 读取法兰位姿
    std::vector<cv::Mat> flange_poses;
    if (!loadRobotPosesFromCSV(save_dir + "/rob_pos.csv", flange_poses)) {
        result.message = "无法加载 " + save_dir + "/rob_pos.csv";
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    // 扫描图像文件
    std::vector<std::string> img_files;
    cv::Size image_size;
    if (!loadImagesFromDisk(save_dir, img_files, image_size))
        return false;

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
        return false;
    }

    const size_t n = img_files.size();
    const auto& board_points = detector.getChessboardCorners3D();

    struct FrameRecord {
        cv::Mat robot_pose;
        cv::Mat camera_pose;
        double error;
    };
    std::vector<FrameRecord> frames;
    frames.reserve(n);
    size_t unreadable_images = 0;
    size_t pose_detection_failures = 0;
    size_t invalid_correspondences = 0;
    double reprojection_error_sum = 0.0;

    for (size_t i = 0; i < n; i++) {
        cv::Mat img = cv::imread(img_files[i]);
        if (img.empty()) {
            ++unreadable_images;
            continue;
        }
        cv::Mat rvec, tvec;
        std::vector<cv::Point2f> corners;
        std::vector<int> ids;

        // 与采集阶段使用同一个 12 角点门限，避免“采集成功但求解大量丢帧”。
        if (!detector.detectPose(img, rvec, tvec, corners, ids, kMinHandEyeCorners)) {
            ++pose_detection_failures;
            continue;
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
            continue;
        }

        std::vector<cv::Point2f> projected;
        cv::projectPoints(pts3, rvec, tvec, camera_matrix, dist_coeffs, projected);
        double sum_err = 0.0;
        for (size_t j = 0; j < projected.size(); j++)
            sum_err += cv::norm(projected[j] - pts2[j]);

        double err = sum_err / projected.size();
        reprojection_error_sum += err;
        frames.push_back({flange_poses[i], calib_transform::rvecTvecToTransform(rvec, tvec), err});
    }

    const size_t detected_frames = frames.size();
    const double mean_reprojection_error = detected_frames > 0
        ? reprojection_error_sum / detected_frames
        : std::numeric_limits<double>::quiet_NaN();
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

    if (!estimateHandEyeTrans(
            camera_matrix, dist_coeffs, robot_poses, camera_poses, result)) {
        APP_LOG_EVENT_ERROR("calibration.hand_eye",
                            "hand_eye_pipeline",
                            "fail",
                            {{"stage", "solve_and_validate"},
                             {"valid_frames", QString::number(robot_poses.size())},
                             {"message", QString::fromStdString(result.message)}});
        return false;
    }

    if (!output_yaml.empty()) {
        if (!saveCalibrationResult(output_yaml, "T_flange_camera",
                                   result.T_flange_camera, result.camera_matrix, result.dist_coeffs,
                                   result.reproj_error * 1000.0, result.valid_frames)) {
            result.success = false;
            result.message = "标定结果 YAML 写入失败: " + output_yaml;
            std::cerr << "[autoCalib] " << result.message << std::endl;
            return false;
        }
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
