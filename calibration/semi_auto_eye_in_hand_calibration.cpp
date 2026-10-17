#include "semi_auto_eye_in_hand_calibration.hpp"
#include "eye_to_hand_calibration.hpp"

#include "camera_calibration.hpp"           // calibrateCameraFromDisk, loadImagesFromDisk
#include "calibration_utils.hpp"            // loadRobotPosesFromCSV
#include "calibration_session.hpp"
#include "solver_consensus.hpp"
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

#include "utils/calib_transform.hpp"

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
constexpr int kMinimumConsensusSolvers = 3;
constexpr double kMaxSolverPairTranslationMeters = 0.010;  // 10 mm
constexpr double kMaxSolverPairRotationDegrees = 1.0;
constexpr double kLargeRefinementTranslationMeters = 0.010;  // 10 mm

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
    bool quality_passed = false;
    HandEyePhase2Audit phase2;
};

static std::vector<HandEyeMethodResult> solveHandEyeMethods(
    const std::vector<cv::Mat>& robot_poses,
    const std::vector<cv::Mat>& camera_poses,
    const char* phase,
    HandEyePhase2Audit* audit) {
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
            APP_LOG_EVENT_WARNING("calibration.hand_eye",
                              "hand_eye_method",
                              "fail",
                              {{"phase", phase},
                               {"method", method.first},
                               {"frames", QString::number(robot_poses.size())},
                               {"reason", "opencv_exception"},
                               {"message", QString::fromUtf8(exception.what())}});
            auditRecord.failure_reason = "opencv_exception";
            if (audit)
                audit->solvers.push_back(std::move(auditRecord));
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
            APP_LOG_EVENT_WARNING("calibration.hand_eye",
                              "hand_eye_method",
                              "fail",
                              {{"phase", phase},
                               {"method", method.first},
                               {"frames", QString::number(robot_poses.size())},
                               {"reason", "invalid_quality_metrics"}});
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
    std::vector<size_t> cluster_indices;
    std::vector<size_t> excluded_indices;
    double max_pair_translation_meters = 0.0;
    double max_pair_rotation_degrees = 0.0;
    double global_max_pair_translation_meters = 0.0;
    double global_max_pair_rotation_degrees = 0.0;
    double cluster_p95_translation_meters = 0.0;
    double cluster_p95_rotation_degrees = 0.0;
    bool ambiguous = false;
    bool tight_core_available = false;
    std::vector<size_t> tight_core_indices;
    bool passed = false;
    std::string failure_reason;
};

static std::string joinMethodNames(
    const std::vector<HandEyeMethodResult>& results,
    const std::vector<size_t>& indices) {
    std::ostringstream stream;
    for (size_t position = 0; position < indices.size(); ++position) {
        if (position > 0)
            stream << "+";
        stream << results[indices[position]].name;
    }
    return stream.str();
}

static SolverConsensusResult evaluateSolverConsensus(
    const std::vector<HandEyeMethodResult>& results,
    const char* phase,
    HandEyePhase2Audit* audit) {
    SolverConsensusResult consensus;
    std::vector<std::vector<solver_consensus::PairDistance>> distances(
        results.size(),
        std::vector<solver_consensus::PairDistance>(results.size()));
    bool invalid_distance = false;
    for (size_t left = 0; left < results.size(); ++left) {
        for (size_t right = left + 1; right < results.size(); ++right) {
            const TransformDistance measured = measureTransformDistance(
                results[left].transform, results[right].transform);
            if (!std::isfinite(measured.translation_meters) ||
                !std::isfinite(measured.rotation_degrees)) {
                invalid_distance = true;
                continue;
            }
            distances[left][right] = {
                measured.translation_meters, measured.rotation_degrees};
            distances[right][left] = distances[left][right];
            if (audit) {
                HandEyeSolverPairAuditRecord pair;
                pair.phase = phase;
                pair.left_method = results[left].name;
                pair.right_method = results[right].name;
                pair.delta_translation_mm = measured.translation_meters * 1000.0;
                pair.delta_rotation_deg = measured.rotation_degrees;
                audit->pairs.push_back(std::move(pair));
            }
            APP_LOG_EVENT_INFO(
                "calibration.hand_eye", "solver_consensus_pair", "measured",
                {{"phase", phase},
                 {"left_method", QString::fromStdString(results[left].name)},
                 {"right_method", QString::fromStdString(results[right].name)},
                 {"delta_translation_mm", QString::number(
                      measured.translation_meters * 1000.0, 'f', 6)},
                 {"delta_rotation_deg", QString::number(
                      measured.rotation_degrees, 'f', 6)}});
        }
    }

    solver_consensus::Selection selected;
    solver_consensus::Selection tight_core;
    if (invalid_distance) {
        selected.failure_reason = "invalid_solver_distance";
    } else {
        selected = solver_consensus::selectCompleteLinkCluster(
            distances, kMaxSolverPairTranslationMeters,
            kMaxSolverPairRotationDegrees, kMinimumConsensusSolvers);
        tight_core = solver_consensus::selectCompleteLinkCluster(
            distances, 0.003, 0.5, kMinimumConsensusSolvers);
    }
    consensus.medoid_index = selected.medoid_index;
    consensus.cluster_indices = selected.members;
    consensus.excluded_indices = selected.excluded;
    consensus.max_pair_translation_meters = selected.cluster_max_translation;
    consensus.max_pair_rotation_degrees = selected.cluster_max_rotation;
    consensus.global_max_pair_translation_meters = selected.global_max_translation;
    consensus.global_max_pair_rotation_degrees = selected.global_max_rotation;
    consensus.cluster_p95_translation_meters = selected.cluster_p95_translation;
    consensus.cluster_p95_rotation_degrees = selected.cluster_p95_rotation;
    consensus.ambiguous = selected.ambiguous;
    consensus.tight_core_available = tight_core.passed;
    consensus.tight_core_indices = tight_core.members;
    consensus.passed = selected.passed;
    consensus.failure_reason = selected.failure_reason;

    std::vector<double> cluster_distance_sums(results.size(), 0.0);
    for (size_t left_pos = 0; left_pos < selected.members.size(); ++left_pos) {
        for (size_t right_pos = left_pos + 1;
             right_pos < selected.members.size(); ++right_pos) {
            const size_t left = selected.members[left_pos];
            const size_t right = selected.members[right_pos];
            const double normalized =
                distances[left][right].translation /
                    kMaxSolverPairTranslationMeters +
                distances[left][right].rotation /
                    kMaxSolverPairRotationDegrees;
            cluster_distance_sums[left] += normalized;
            cluster_distance_sums[right] += normalized;
        }
    }

    if (audit) {
        for (HandEyeSolverAuditRecord& record : audit->solvers) {
            if (record.phase != phase || !record.success)
                continue;
            for (size_t index = 0; index < results.size(); ++index) {
                if (record.method != results[index].name)
                    continue;
                record.in_consensus_cluster = std::find(
                    selected.members.begin(), selected.members.end(), index) !=
                    selected.members.end();
                record.selected_medoid = !selected.members.empty() &&
                    index == selected.medoid_index;
                record.medoid_normalized_distance_sum =
                    cluster_distance_sums[index];
                if (!record.in_consensus_cluster) {
                    std::ostringstream reason;
                    reason << "outside_complete_link_cluster";
                    bool first = true;
                    for (const size_t member : selected.members) {
                        const auto& distance = distances[index][member];
                        if (distance.translation <= kMaxSolverPairTranslationMeters &&
                            distance.rotation <= kMaxSolverPairRotationDegrees)
                            continue;
                        reason << (first ? ":conflicts_with=" : ",")
                               << results[member].name;
                        first = false;
                    }
                    record.consensus_exclusion_reason = reason.str();
                }
                break;
            }
        }
        HandEyeConsensusAuditRecord record;
        record.phase = phase;
        record.successful_solvers = static_cast<int>(results.size());
        record.cluster_size = static_cast<int>(selected.members.size());
        for (const size_t index : selected.members)
            record.cluster_members.push_back(results[index].name);
        for (const size_t index : selected.excluded) {
            record.excluded_solvers.push_back(results[index].name);
            for (const HandEyeSolverAuditRecord& solver : audit->solvers) {
                if (solver.phase == phase && solver.method == results[index].name) {
                    record.exclusion_reasons.push_back(
                        solver.consensus_exclusion_reason);
                    break;
                }
            }
        }
        for (const auto& candidate : selected.largest_clusters)
            record.largest_cluster_candidates.push_back(
                joinMethodNames(results, candidate));
        if (!selected.members.empty())
            record.medoid_method = results[selected.medoid_index].name;
        record.cluster_max_translation_mm =
            selected.cluster_max_translation * 1000.0;
        record.cluster_p95_translation_mm =
            selected.cluster_p95_translation * 1000.0;
        record.cluster_max_rotation_deg = selected.cluster_max_rotation;
        record.cluster_p95_rotation_deg = selected.cluster_p95_rotation;
        record.max_pair_translation_mm = selected.global_max_translation * 1000.0;
        record.max_pair_rotation_deg = selected.global_max_rotation;
        record.max_normalized_pair_distance =
            selected.max_normalized_pair_distance;
        record.normalized_pair_distance_sum =
            selected.normalized_pair_distance_sum;
        record.translation_limit_mm = kMaxSolverPairTranslationMeters * 1000.0;
        record.rotation_limit_deg = kMaxSolverPairRotationDegrees;
        record.ambiguous = selected.ambiguous;
        record.tight_core_available = tight_core.passed;
        for (const size_t index : tight_core.members)
            record.tight_core_members.push_back(results[index].name);
        record.passed = selected.passed;
        record.failure_reason = selected.failure_reason;
        audit->consensus.push_back(std::move(record));
    }

    const std::string medoid = selected.members.empty()
        ? "none" : results[selected.medoid_index].name;
    const std::string members = joinMethodNames(results, selected.members);
    const std::string excluded = joinMethodNames(results, selected.excluded);
    APP_LOG_EVENT(
        Info, "calibration.hand_eye", "solver_consensus_cluster",
        selected.passed ? "success" : "fail",
        {"phase", phase},
        {"successful_solvers", QString::number(results.size())},
        {"cluster_members", QString::fromStdString(members)},
        {"excluded_solvers", QString::fromStdString(excluded)},
        {"medoid_method", QString::fromStdString(medoid)},
        {"cluster_max_translation_mm", QString::number(
             selected.cluster_max_translation * 1000.0, 'f', 6)},
        {"cluster_p95_translation_mm", QString::number(
             selected.cluster_p95_translation * 1000.0, 'f', 6)},
        {"cluster_max_rotation_deg", QString::number(
             selected.cluster_max_rotation, 'f', 6)},
        {"cluster_p95_rotation_deg", QString::number(
             selected.cluster_p95_rotation, 'f', 6)},
        {"raw_global_max_translation_mm", QString::number(
             selected.global_max_translation * 1000.0, 'f', 6)},
        {"raw_global_max_rotation_deg", QString::number(
             selected.global_max_rotation, 'f', 6)},
        {"tight_core_members", QString::fromStdString(
             joinMethodNames(results, tight_core.members))},
        {"failure_reason", QString::fromStdString(selected.failure_reason)});
    std::cout << "[autoCalib] Solver consensus [" << phase
              << "]: cluster=" << members << ", excluded=" << excluded
              << ", medoid=" << medoid << ", cluster max Δt="
              << selected.cluster_max_translation * 1000.0
              << " mm, cluster max ΔR=" << selected.cluster_max_rotation
              << " deg, raw global max Δt="
              << selected.global_max_translation * 1000.0
              << " mm, gate=" << (selected.passed ? "PASS" : "FAIL")
              << std::endl;
    return consensus;
}

struct OptimizedMedoidResult {
    cv::Mat transform;
    HandEyeConsistencyStats stats;
    bool large_refinement_delta = false;
};

static OptimizedMedoidResult optimizeMedoidOnce(
    const HandEyeMethodResult& medoid,
    const std::vector<cv::Mat>& robot_poses,
    const std::vector<cv::Mat>& camera_poses,
    const char* phase,
    HandEyePhase2Audit* audit) {
    OptimizedMedoidResult optimized;
    optimized.transform = refineHandEyePoseConsistency(
        medoid.transform, robot_poses, camera_poses);
    optimized.stats = evaluateHandEyeConsistency(
        optimized.transform, robot_poses, camera_poses);
    const TransformDistance delta = measureTransformDistance(
        medoid.transform, optimized.transform);
    optimized.large_refinement_delta =
        std::isfinite(delta.translation_meters) &&
        delta.translation_meters > kLargeRefinementTranslationMeters;

    HandEyeOptimizationAuditRecord record;
    record.performed = true;
    record.phase = phase;
    record.method = medoid.name;
    record.initial_transform = medoid.transform.clone();
    record.optimized_transform = optimized.transform.clone();
    record.delta_translation_mm = delta.translation_meters * 1000.0;
    record.delta_rotation_deg = delta.rotation_degrees;
    record.large_refinement_delta = optimized.large_refinement_delta;
    record.requires_manual_confirmation = optimized.large_refinement_delta;
    record.before_translation_rms_mm = medoid.stats.translation_rms * 1000.0;
    record.after_translation_rms_mm = optimized.stats.translation_rms * 1000.0;
    record.before_rotation_rms_deg = medoid.stats.rotation_rms_degrees;
    record.after_rotation_rms_deg = optimized.stats.rotation_rms_degrees;
    if (audit)
        audit->optimizations.push_back(record);

    if (optimized.large_refinement_delta) {
        APP_LOG_EVENT_WARNING(
            "calibration.hand_eye", "medoid_refinement", "complete",
            {"phase", phase},
            {"method", QString::fromStdString(medoid.name)},
            {"delta_translation_mm", QString::number(
                 delta.translation_meters * 1000.0, 'f', 6)},
            {"delta_rotation_deg", QString::number(
                 delta.rotation_degrees, 'f', 6)},
            {"large_refinement_delta", "1"},
            {"publication_policy", "manual_confirmation_required"});
    } else {
        APP_LOG_EVENT_INFO(
            "calibration.hand_eye", "medoid_refinement", "complete",
            {"phase", phase},
            {"method", QString::fromStdString(medoid.name)},
            {"delta_translation_mm", QString::number(
                 delta.translation_meters * 1000.0, 'f', 6)},
            {"delta_rotation_deg", QString::number(
                 delta.rotation_degrees, 'f', 6)},
            {"large_refinement_delta", "0"},
            {"publication_policy", "automatic_allowed"});
    }
    std::cout << "[autoCalib] Medoid refinement [" << phase << "] "
              << medoid.name << ": Δt=" << delta.translation_meters * 1000.0
              << " mm, ΔR=" << delta.rotation_degrees << " deg" << std::endl;
    return optimized;
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
    if (!final_consensus.passed) {
        result.message = "原始 solver 多数共识未通过: " +
                         final_consensus.failure_reason;
        publishSolve(CalibrationRuntimeStage::Failed,
                     final_consensus.failure_reason);
        return false;
    }
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
        if (!final_consensus.passed) {
            result.message = "MAD 后原始 solver 多数共识未通过: " +
                             final_consensus.failure_reason;
            publishSolve(CalibrationRuntimeStage::Failed,
                         final_consensus.failure_reason);
            return false;
        }
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
    else if (final_optimized.large_refinement_delta)
        failure_reason = "large_refinement_delta_requires_manual_confirmation";
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
    // The shared solver equation is robot_i * X * camera_i = constant.
    // Substituting inv(G_i) yields the internal eye-to-hand transform.  The
    // production eye_to_hand_result.yaml convention (validated against the
    // 2026-06-27 field result and used by existing consumers) stores its
    // inverse under the historical T_base_camera key.
    std::vector<cv::Mat> inverse_robot, cameras = camera_poses;
    for (const auto& pose : robot_poses)
        inverse_robot.push_back(calib_transform::invertTransform(pose));
    CalibrationResult solved;
    HandEyeSolveAudit audit;
    result.quality.pnp_valid_frames = static_cast<int>(robot_poses.size());
    const bool ok = estimateHandEyeTrans(result.camera_matrix, result.dist_coeffs,
        inverse_robot, cameras, solved, &audit, &result.quality, {},
        "T_eye_to_hand_internal");
    result.message = solved.message;
    if (!ok) return false;
    if (!validTransform(solved.T_flange_camera)) {
        result.message = "眼在手外求解输出不是有效刚体变换";
        return false;
    }
    result.T_base_camera = calib_transform::invertTransform(
        solved.T_flange_camera);
    if (!validTransform(result.T_base_camera)) {
        result.T_base_camera.release();
        result.message = "眼在手外生产方向转换失败";
        return false;
    }
    std::cout << "[eyeToHand] 生产约定 T_base_camera:\n"
              << result.T_base_camera << std::endl;
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
    CalibrationSession& session,
    CalibrationRuntimeStatus* runtime_status,
    const CalibrationRuntimeCallback& runtime_callback) {

    result.success = false;
    result.valid_frames = 0;
    result.reproj_error = 0.0;
    result.message.clear();
    result.session_dir.clear();
    result.T_flange_camera.release();

    result.session_dir = session.directory();

    auto failSession = [&](const std::string& stage,
                           const std::string& reason) -> bool {
        session.setStage(stage);
        session.finalizeFailure(stage, reason, result.message);
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
    session.setStage("hand_eye_input");
    session.setIntrinsics(camera_matrix, dist_coeffs,
                           "effective_runtime");
    if (runtime_status) {
        runtime_status->calibration_session_id = session.id();
        runtime_status->calibration_session_dir = session.directory();
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
    std::string robot_pose_file = save_dir + "/rob_pos.csv";
    if (std::filesystem::exists(
            session.robotPosesDirectory() + "/rob_pos.csv")) {
        robot_pose_file = session.robotPosesDirectory() + "/rob_pos.csv";
    }
    if (!loadRobotPosesFromCSV(robot_pose_file, flange_poses)) {
        result.message = "无法加载 " + robot_pose_file;
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return failSession("hand_eye_input", "robot_pose_load_failed");
    }

    // 扫描图像文件
    std::vector<std::string> img_files;
    cv::Size image_size;
    if (!loadImagesFromDisk(save_dir, img_files, image_size)) {
        result.message = "无法加载标定图像: " + save_dir;
        return failSession("hand_eye_input", "image_load_failed");
    }
    session.setImageSize(image_size.width, image_size.height);

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
            session.robotPosesDirectory() + "/" + raw_name.str();
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

    session.setFrames(frame_audits);

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
    session.setStage("solve_and_validate");
    HandEyeSolveAudit solve_audit;
    const bool solved = estimateHandEyeTrans(
        camera_matrix, dist_coeffs, robot_poses, camera_poses, result,
        &solve_audit, runtime_status, runtime_callback);
    std::string audit_error;
    if (!session.setHandEyePhase2Audit(solve_audit.phase2, audit_error)) {
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
        session.setFrames(frame_audits);
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
    session.setFrames(frame_audits);

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
    if (!session.finalizeSuccess(result.T_flange_camera, result.camera_matrix,
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
