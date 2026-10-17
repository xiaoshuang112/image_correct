#include "camera_calibration.hpp"
#include "hand_eye_gui/DiagnosticLogger.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <filesystem>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

namespace {

constexpr size_t kMaxInitialOptimizationFrames = 60;
constexpr size_t kMaxRefineOptimizationFrames = 48;
constexpr size_t kMinCalibrationFrames = 5;
constexpr int kFastCalibrationSolverFlag = cv::CALIB_USE_LU;
constexpr double kMaxMedianRegressionRatio = 1.05;
constexpr double kMaxMeanRegressionRatio = 1.08;
constexpr double kMaxP95RegressionRatio = 1.10;
constexpr double kMedianRegressionTolerancePx = 0.02;
constexpr double kMeanRegressionTolerancePx = 0.03;
constexpr double kP95RegressionTolerancePx = 0.05;

using Clock = std::chrono::steady_clock;

static long long elapsedMilliseconds(const Clock::time_point& startedAt)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               Clock::now() - startedAt)
        .count();
}

static double maximumAcceptedError(double baseline,
                                   double ratio,
                                   double absoluteTolerance)
{
    return std::max(baseline * ratio, baseline + absoluteTolerance);
}

struct ViewDescriptor {
    size_t sourceIndex = 0;
    std::array<double, 6> feature{};
    size_t cornerCount = 0;
};

static ViewDescriptor describeView(size_t sourceIndex,
                                   const std::vector<cv::Point2f>& points,
                                   const cv::Size& imageSize)
{
    ViewDescriptor descriptor;
    descriptor.sourceIndex = sourceIndex;
    descriptor.cornerCount = points.size();
    if (points.empty() || imageSize.width <= 0 || imageSize.height <= 0)
        return descriptor;

    cv::Point2d center(0.0, 0.0);
    for (const auto& point : points) {
        center.x += point.x;
        center.y += point.y;
    }
    center.x /= static_cast<double>(points.size());
    center.y /= static_cast<double>(points.size());

    std::vector<cv::Point2f> hull;
    cv::convexHull(points, hull);
    const double imageArea = static_cast<double>(imageSize.width) *
                             static_cast<double>(imageSize.height);
    const double hullArea = hull.size() >= 3 ? std::abs(cv::contourArea(hull)) : 0.0;
    const double scale = imageArea > 0.0
        ? std::sqrt(std::max(0.0, hullArea / imageArea))
        : 0.0;

    const cv::RotatedRect rectangle = cv::minAreaRect(points);
    double angleDegrees = rectangle.angle;
    if (rectangle.size.width < rectangle.size.height)
        angleDegrees += 90.0;
    const double angle = angleDegrees * CV_PI / 180.0;
    const double major = std::max<double>(rectangle.size.width, rectangle.size.height);
    const double minor = std::min<double>(rectangle.size.width, rectangle.size.height);
    const double aspect = major > 1e-6 ? minor / major : 0.0;

    descriptor.feature = {
        center.x / static_cast<double>(imageSize.width),
        center.y / static_cast<double>(imageSize.height),
        scale,
        aspect,
        std::cos(2.0 * angle),
        std::sin(2.0 * angle)
    };
    return descriptor;
}

static double viewDistanceSquared(const ViewDescriptor& lhs,
                                  const ViewDescriptor& rhs)
{
    const double dx = lhs.feature[0] - rhs.feature[0];
    const double dy = lhs.feature[1] - rhs.feature[1];
    const double ds = lhs.feature[2] - rhs.feature[2];
    const double da = lhs.feature[3] - rhs.feature[3];
    const double dc = lhs.feature[4] - rhs.feature[4];
    const double dn = lhs.feature[5] - rhs.feature[5];
    return 4.0 * dx * dx + 4.0 * dy * dy + 9.0 * ds * ds +
           da * da + 0.25 * dc * dc + 0.25 * dn * dn;
}

// 用最远点采样保留中心位置、成像尺度、透视形变和画面旋转均有差异的帧。
// 这一步只做几何统计，不进入非线性优化，适合先处理数百帧输入。
static std::vector<size_t> selectRepresentativeViews(
    const std::vector<std::vector<cv::Point2f>>& imagePoints,
    const cv::Size& imageSize,
    const std::vector<size_t>& candidateIndices,
    size_t maxFrames)
{
    if (candidateIndices.size() <= maxFrames)
        return candidateIndices;

    std::vector<ViewDescriptor> descriptors;
    descriptors.reserve(candidateIndices.size());
    size_t maxCornerCount = 1;
    for (const size_t index : candidateIndices) {
        descriptors.push_back(describeView(index, imagePoints[index], imageSize));
        maxCornerCount = std::max(maxCornerCount, imagePoints[index].size());
    }

    // 从角点最完整的帧开始；相同角点数时保持磁盘顺序，结果可复现。
    size_t seed = 0;
    for (size_t i = 1; i < descriptors.size(); ++i) {
        if (descriptors[i].cornerCount > descriptors[seed].cornerCount)
            seed = i;
    }

    std::vector<bool> used(descriptors.size(), false);
    std::vector<size_t> selectedDescriptorIndices;
    selectedDescriptorIndices.reserve(maxFrames);
    selectedDescriptorIndices.push_back(seed);
    used[seed] = true;

    while (selectedDescriptorIndices.size() < maxFrames) {
        size_t best = descriptors.size();
        double bestScore = -1.0;
        for (size_t candidate = 0; candidate < descriptors.size(); ++candidate) {
            if (used[candidate])
                continue;

            double minDistance = std::numeric_limits<double>::max();
            for (const size_t selected : selectedDescriptorIndices) {
                minDistance = std::min(
                    minDistance,
                    viewDistanceSquared(descriptors[candidate], descriptors[selected]));
            }
            const double cornerQuality = static_cast<double>(descriptors[candidate].cornerCount) /
                                         static_cast<double>(maxCornerCount);
            const double score = minDistance * (0.85 + 0.15 * cornerQuality) +
                                 0.001 * cornerQuality;
            if (score > bestScore) {
                bestScore = score;
                best = candidate;
            }
        }
        if (best == descriptors.size())
            break;
        used[best] = true;
        selectedDescriptorIndices.push_back(best);
    }

    std::vector<size_t> selected;
    selected.reserve(selectedDescriptorIndices.size());
    for (const size_t descriptorIndex : selectedDescriptorIndices)
        selected.push_back(descriptors[descriptorIndex].sourceIndex);
    return selected;
}

static bool calibrationResultIsValid(double error,
                                     const cv::Mat& cameraMatrix,
                                     const cv::Mat& distortion)
{
    return std::isfinite(error) && error >= 0.0 &&
           cameraMatrix.rows == 3 && cameraMatrix.cols == 3 &&
           cv::checkRange(cameraMatrix) && cv::checkRange(distortion);
}

// 初始优化只使用最多 60 个有代表性的视角，避免输入帧数直接放大
// calibrateCamera 的非线性参数规模。
static bool calibrateCameraFiltered(
    const std::vector<std::vector<cv::Point3f>>& object_points,
    const std::vector<std::vector<cv::Point2f>>& image_points,
    const cv::Size& image_size,
    cv::Mat& camera_matrix,
    cv::Mat& dist_coeffs) {
    if (object_points.size() != image_points.size() ||
        object_points.size() < kMinCalibrationFrames) {
        APP_LOG_EVENT_ERROR("camera.calibration",
                           "camera_intrinsics_initial",
                           "fail",
                           {{"reason", "invalid_input"},
                            {"object_frames", QString::number(object_points.size())},
                            {"image_frames", QString::number(image_points.size())},
                            {"min_required", QString::number(kMinCalibrationFrames)}});
        return false;
    }

    std::vector<size_t> candidates(object_points.size());
    std::iota(candidates.begin(), candidates.end(), size_t{0});
    const std::vector<size_t> selected = selectRepresentativeViews(
        image_points, image_size, candidates, kMaxInitialOptimizationFrames);

    std::vector<std::vector<cv::Point3f>> selectedObjectPoints;
    std::vector<std::vector<cv::Point2f>> selectedImagePoints;
    selectedObjectPoints.reserve(selected.size());
    selectedImagePoints.reserve(selected.size());
    for (const size_t index : selected) {
        selectedObjectPoints.push_back(object_points[index]);
        selectedImagePoints.push_back(image_points[index]);
    }

    const auto startedAt = Clock::now();
    std::cout << "[cameraCalib] 初始非线性优化: " << selected.size()
              << "/" << object_points.size() << " 帧" << std::endl;
    APP_LOG_EVENT_INFO("camera.calibration",
                       "camera_intrinsics_initial",
                       "start",
                       {{"phase", "initial_optimizer"},
                        {"input_frames", QString::number(object_points.size())},
                        {"selected_frames", QString::number(selected.size())},
                        {"solver", "LU"},
                        {"image_w", QString::number(image_size.width)},
                        {"image_h", QString::number(image_size.height)}});

    cv::Mat candidateMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat candidateDistortion = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    double final_err = std::numeric_limits<double>::quiet_NaN();
    try {
        final_err = cv::calibrateCamera(selectedObjectPoints, selectedImagePoints, image_size,
                                        candidateMatrix, candidateDistortion,
                                        rvecs, tvecs, kFastCalibrationSolverFlag);
    } catch (const cv::Exception& e) {
        APP_LOG_EVENT_ERROR("camera.calibration",
                           "camera_intrinsics_initial",
                           "fail",
                           {{"phase", "initial_optimizer"},
                            {"selected_frames", QString::number(selected.size())},
                            {"elapsed_ms", QString::number(elapsedMilliseconds(startedAt))},
                            {"exception", QString::fromUtf8(e.what())}});
        std::cerr << "[autoCalib] 最终相机标定异常: " << e.what() << std::endl;
        return false;
    }

    if (!calibrationResultIsValid(final_err, candidateMatrix, candidateDistortion)) {
        APP_LOG_EVENT_ERROR("camera.calibration",
                           "camera_intrinsics_initial",
                           "fail",
                           {{"phase", "initial_optimizer"},
                            {"error_rms_px", QString::number(final_err, 'f', 6)},
                            {"selected_frames", QString::number(selected.size())},
                            {"camera_valid", candidateMatrix.empty() ? "false" : "true"},
                            {"dist_valid", candidateDistortion.empty() ? "false" : "true"}});
        return false;
    }

    camera_matrix = candidateMatrix;
    dist_coeffs = candidateDistortion;
    const long long elapsedMs = elapsedMilliseconds(startedAt);
    std::cout << "[cameraCalib] 初始优化完成: " << selected.size()
              << " 帧, " << elapsedMs << " ms, 重投影误差 "
              << final_err << " px" << std::endl;
    APP_LOG_EVENT_INFO("camera.calibration",
                       "camera_intrinsics_initial",
                       "success",
                       {{"phase", "initial_optimizer"},
                        {"selected_frames", QString::number(selected.size())},
                        {"elapsed_ms", QString::number(elapsedMs)},
                        {"rms_px", QString::number(final_err, 'f', 6)}});
    return true;
}

// 全部帧仅执行快速 solvePnP 误差评估；真正的第二次非线性优化最多 48 帧。
// K, D 入/出: 初始值来自代表帧优化，精化失败时保留初始结果。
static bool refineCameraIntrinsics(
    cv::Mat& K, cv::Mat& D,
    const std::vector<std::vector<cv::Point3f>>& all_obj,
    const std::vector<std::vector<cv::Point2f>>& all_img,
    const cv::Size& image_size) {

    const auto stageStartedAt = Clock::now();
    const int n = static_cast<int>(all_obj.size());
    if (n < static_cast<int>(kMinCalibrationFrames) || all_img.size() != all_obj.size())
        return false;

    APP_LOG_EVENT_INFO("camera.calibration",
                       "camera_intrinsics_refine",
                       "start",
                       {{"phase", "evaluation"},
                        {"input_frames", QString::number(n)},
                        {"optimization_cap", QString::number(kMaxRefineOptimizationFrames)}});

    // 逐帧 solvePnP + 重投影误差 (快: ~0.2ms/帧)
    struct Rec { int idx; double err; };
    std::vector<Rec> recs;
    recs.reserve(n);
    for (int i = 0; i < n; i++) {
        cv::Mat rvec, tvec;
        if (!cv::solvePnP(all_obj[i], all_img[i], K, D, rvec, tvec))
            continue;
        std::vector<cv::Point2f> proj;
        cv::projectPoints(all_obj[i], rvec, tvec, K, D, proj);
        double sum = 0.0;
        for (size_t j = 0; j < proj.size(); j++)
            sum += cv::norm(proj[j] - all_img[i][j]);
        if (proj.empty())
            continue;
        const double frameError = sum / static_cast<double>(proj.size());
        if (!std::isfinite(frameError))
            continue;
        recs.push_back({i, frameError});
        if ((i + 1) % 50 == 0 || i + 1 == n) {
            APP_LOG_EVENT_DEBUG("camera.calibration",
                               "camera_intrinsics_refine",
                               "progress",
                               {{"phase", "evaluation"},
                                {"processed", QString::number(i + 1)},
                                {"total", QString::number(n)},
                                {"valid", QString::number(recs.size())},
                                {"elapsed_ms", QString::number(elapsedMilliseconds(stageStartedAt))}});
        }
    }
    if (recs.size() < kMinCalibrationFrames) {
        APP_LOG_EVENT_WARNING("camera.calibration",
                              "camera_intrinsics_refine",
                              "skip",
                              {{"phase", "evaluation"},
                               {"reason", "insufficient_pnp_frames"},
                               {"valid_frames", QString::number(recs.size())},
                               {"total_frames", QString::number(n)}});
        return false;
    }

    std::sort(recs.begin(), recs.end(),
              [](const Rec& a, const Rec& b) { return a.err < b.err; });

    // solvePnP 可能会跳过退化帧，因此中位数索引必须基于 recs，
    // 不能继续使用输入帧数量，否则会发生越界访问。
    const double median = recs[recs.size() / 2].err;
    const size_t initialP95Index = static_cast<size_t>(
        0.95 * static_cast<double>(recs.size() - 1));
    const double initialP95 = recs[initialP95Index].err;
    const double initialMean = std::accumulate(
        recs.begin(), recs.end(), 0.0,
        [](double total, const Rec& record) { return total + record.err; }) /
        static_cast<double>(recs.size());
    std::vector<double> deviations;
    deviations.reserve(recs.size());
    for (const auto& record : recs)
        deviations.push_back(std::abs(record.err - median));
    std::sort(deviations.begin(), deviations.end());
    const double mad = deviations[deviations.size() / 2];
    const double robustSigma = 1.4826 * mad;
    const double thresh = std::max(median * 1.35, median + 3.0 * robustSigma);

    std::vector<size_t> eligibleIndices;
    eligibleIndices.reserve(recs.size());
    for (auto& r : recs) {
        if (r.err <= thresh)
            eligibleIndices.push_back(static_cast<size_t>(r.idx));
    }
    if (eligibleIndices.size() < kMinCalibrationFrames) {
        APP_LOG_EVENT_WARNING("camera.calibration",
                              "camera_intrinsics_refine",
                              "skip",
                              {{"phase", "selection"},
                               {"reason", "insufficient_inliers"},
                               {"eligible", QString::number(eligibleIndices.size())},
                               {"median_px", QString::number(median, 'f', 6)},
                               {"threshold_px", QString::number(thresh, 'f', 6)}});
        return false;
    }

    const std::vector<size_t> selectedIndices = selectRepresentativeViews(
        all_img, image_size, eligibleIndices, kMaxRefineOptimizationFrames);
    std::vector<std::vector<cv::Point3f>> good_obj;
    std::vector<std::vector<cv::Point2f>> good_img;
    good_obj.reserve(selectedIndices.size());
    good_img.reserve(selectedIndices.size());
    for (const size_t index : selectedIndices) {
        good_obj.push_back(all_obj[index]);
        good_img.push_back(all_img[index]);
    }

    const long long evaluationMs = elapsedMilliseconds(stageStartedAt);
    std::cout << "[cameraCalib] 全量快速评估: " << recs.size() << "/" << n
              << " 帧有效, " << eligibleIndices.size() << " 帧通过离群过滤; "
              << "精化优化选取 " << selectedIndices.size() << " 帧, 评估耗时 "
              << evaluationMs << " ms" << std::endl;
    APP_LOG_EVENT_INFO("camera.calibration",
                       "camera_intrinsics_refine",
                       "select_done",
                       {{"phase", "selection"},
                        {"evaluated_frames", QString::number(recs.size())},
                        {"eligible_frames", QString::number(eligibleIndices.size())},
                        {"selected_frames", QString::number(selectedIndices.size())},
                        {"cap", QString::number(kMaxRefineOptimizationFrames)},
                        {"median_px", QString::number(median, 'f', 6)},
                        {"mean_px", QString::number(initialMean, 'f', 6)},
                        {"p95_px", QString::number(initialP95, 'f', 6)},
                        {"mad_px", QString::number(mad, 'f', 6)},
                        {"threshold_px", QString::number(thresh, 'f', 6)},
                        {"evaluation_ms", QString::number(evaluationMs)}});

    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat refinedMatrix = K.clone();
    cv::Mat refinedDistortion = D.clone();
    double final_err = std::numeric_limits<double>::quiet_NaN();
    const auto optimizationStartedAt = Clock::now();
    APP_LOG_EVENT_INFO("camera.calibration",
                       "camera_intrinsics_refine",
                       "opt_start",
                       {{"phase", "optimization"},
                        {"selected_frames", QString::number(selectedIndices.size())},
                        {"solver", "LU"}});
    try {
        final_err = cv::calibrateCamera(good_obj, good_img, image_size,
                                        refinedMatrix, refinedDistortion, rvecs, tvecs,
                                        cv::CALIB_USE_INTRINSIC_GUESS |
                                            kFastCalibrationSolverFlag);
    } catch (const cv::Exception& e) {
        APP_LOG_EVENT_ERROR("camera.calibration",
                           "camera_intrinsics_refine",
                           "fail",
                           {{"phase", "optimization"},
                            {"selected_frames", QString::number(selectedIndices.size())},
                            {"elapsed_ms", QString::number(elapsedMilliseconds(optimizationStartedAt))},
                            {"exception", QString::fromUtf8(e.what())}});
        std::cerr << "[refineCam] 相机内参精化异常: " << e.what() << std::endl;
        return false;
    }

    if (!calibrationResultIsValid(final_err, refinedMatrix, refinedDistortion)) {
        APP_LOG_EVENT_ERROR("camera.calibration",
                           "camera_intrinsics_refine",
                           "fail",
                           {{"phase", "optimization"},
                            {"error_rms_px", QString::number(final_err, 'f', 6)},
                            {"selected_frames", QString::number(selectedIndices.size())}});
        return false;
    }
    const long long optimizationMs = elapsedMilliseconds(optimizationStartedAt);

    // 用全部有效帧快速复核精化结果，避免代表帧上的误差下降却损害整体。
    std::vector<double> validationErrors;
    validationErrors.reserve(recs.size());
    size_t validationFailures = 0;
    // 前后必须使用同一组初始 solvePnP 成功帧。若精化后把难解帧直接
    // 跳过，均值和 P95 会产生虚假的改善。
    for (const Rec& record : recs) {
        const size_t i = static_cast<size_t>(record.idx);
        cv::Mat rvec, tvec;
        if (!cv::solvePnP(all_obj[i], all_img[i], refinedMatrix,
                          refinedDistortion, rvec, tvec)) {
            ++validationFailures;
            continue;
        }
        std::vector<cv::Point2f> projected;
        cv::projectPoints(all_obj[i], rvec, tvec, refinedMatrix,
                          refinedDistortion, projected);
        if (projected.empty() || projected.size() != all_img[i].size()) {
            ++validationFailures;
            continue;
        }
        double errorSum = 0.0;
        for (size_t point = 0; point < projected.size(); ++point)
            errorSum += cv::norm(projected[point] - all_img[i][point]);
        const double validationFrameError =
            errorSum / static_cast<double>(projected.size());
        if (std::isfinite(validationFrameError)) {
            validationErrors.push_back(validationFrameError);
        } else {
            ++validationFailures;
        }
    }
    if (validationFailures != 0 || validationErrors.size() != recs.size()) {
        APP_LOG_EVENT_WARNING("camera.calibration",
                              "camera_intrinsics_refine",
                              "reject",
                              {{"phase", "validation"},
                               {"reason", "validation_frame_set_changed"},
                               {"expected_frames", QString::number(recs.size())},
                               {"valid_frames", QString::number(validationErrors.size())},
                               {"failed_frames", QString::number(validationFailures)}});
        return false;
    }
    std::sort(validationErrors.begin(), validationErrors.end());
    const double validationMedian = validationErrors[validationErrors.size() / 2];
    const size_t p95Index = static_cast<size_t>(
        0.95 * static_cast<double>(validationErrors.size() - 1));
    const double validationP95 = validationErrors[p95Index];
    const double validationMean = std::accumulate(
        validationErrors.begin(), validationErrors.end(), 0.0) /
        static_cast<double>(validationErrors.size());
    const double maximumMedian = maximumAcceptedError(
        median, kMaxMedianRegressionRatio, kMedianRegressionTolerancePx);
    const double maximumMean = maximumAcceptedError(
        initialMean, kMaxMeanRegressionRatio, kMeanRegressionTolerancePx);
    const double maximumP95 = maximumAcceptedError(
        initialP95, kMaxP95RegressionRatio, kP95RegressionTolerancePx);
    const bool validationIsFinite = std::isfinite(validationMedian) &&
                                    std::isfinite(validationMean) &&
                                    std::isfinite(validationP95);
    const bool distributionRegressed =
        validationMedian > maximumMedian ||
        validationMean > maximumMean ||
        validationP95 > maximumP95;
    if (!validationIsFinite || distributionRegressed) {
        APP_LOG_EVENT_WARNING("camera.calibration",
                              "camera_intrinsics_refine",
                              "reject",
                              {{"phase", "validation"},
                               {"reason", validationIsFinite
                                              ? "error_distribution_regression"
                                              : "non_finite_validation_error"},
                               {"before_median_px", QString::number(median, 'f', 6)},
                               {"after_median_px", QString::number(validationMedian, 'f', 6)},
                               {"max_median_px", QString::number(maximumMedian, 'f', 6)},
                               {"before_mean_px", QString::number(initialMean, 'f', 6)},
                               {"after_mean_px", QString::number(validationMean, 'f', 6)},
                               {"max_mean_px", QString::number(maximumMean, 'f', 6)},
                               {"before_p95_px", QString::number(initialP95, 'f', 6)},
                               {"after_p95_px", QString::number(validationP95, 'f', 6)},
                               {"max_p95_px", QString::number(maximumP95, 'f', 6)}});
        return false;
    }

    K = refinedMatrix;
    D = refinedDistortion;
    std::cout << "[refineCam] 精化完成: " << good_obj.size() << " 帧, "
              << optimizationMs << " ms, err=" << final_err << " px" << std::endl;
    std::cout << "[autoCalib] 相机矩阵:\n" << K << std::endl;
    APP_LOG_EVENT_INFO("camera.calibration",
                       "camera_intrinsics_refine",
                       "success",
                       {{"phase", "optimization"},
                        {"selected_frames", QString::number(selectedIndices.size())},
                        {"optimization_ms", QString::number(optimizationMs)},
                        {"total_ms", QString::number(elapsedMilliseconds(stageStartedAt))},
                        {"rms_px", QString::number(final_err, 'f', 6)}});
    APP_LOG_EVENT_INFO("camera.calibration",
                       "camera_intrinsics_validation",
                       "success",
                       {{"phase", "global_validation"},
                        {"frames", QString::number(validationErrors.size())},
                        {"before_median_px", QString::number(median, 'f', 6)},
                        {"before_mean_px", QString::number(initialMean, 'f', 6)},
                        {"before_p95_px", QString::number(initialP95, 'f', 6)},
                        {"after_mean_px", QString::number(validationMean, 'f', 6)},
                        {"after_median_px", QString::number(validationMedian, 'f', 6)},
                        {"after_p95_px", QString::number(validationP95, 'f', 6)},
                        {"max_mean_px", QString::number(maximumMean, 'f', 6)},
                        {"max_median_px", QString::number(maximumMedian, 'f', 6)},
                        {"max_p95_px", QString::number(maximumP95, 'f', 6)}});
    std::cout << "[cameraCalib] 全量复核: " << validationErrors.size()
              << " 帧, mean=" << validationMean
              << " px, median=" << validationMedian
              << " px, p95=" << validationP95 << " px" << std::endl;
    return true;
}

} // anonymous namespace

// ============================================================
// 公开 API
// ============================================================

bool loadImagesFromDisk(const std::string& save_dir,
                         std::vector<std::string>& img_files,
                         cv::Size& image_size) {
    namespace fs = std::filesystem;
    try {
        for (auto& entry : fs::directory_iterator(save_dir)) {
            std::string name = entry.path().filename().string();
            if (name.size() >= 4 && name.substr(name.size() - 4) == ".jpg")
                img_files.push_back(entry.path().string());
        }
    } catch (const fs::filesystem_error& e) {
        APP_LOG_EVENT_ERROR("camera.calibration",
                           "camera_input",
                           "fail",
                           {{"reason", "directory_scan"},
                            {"directory", QString::fromStdString(save_dir)},
                            {"message", QString::fromUtf8(e.what())}});
        std::cerr << "[cameraCalib] 读取图像目录失败: " << e.what() << std::endl;
        return false;
    }
    std::sort(img_files.begin(), img_files.end());
    if (img_files.empty()) {
        APP_LOG_EVENT_ERROR("camera.calibration",
                           "camera_input",
                           "fail",
                           {{"reason", "no_jpg_files"},
                            {"directory", QString::fromStdString(save_dir)}});
        return false;
    }
    cv::Mat first = cv::imread(img_files[0]);
    if (first.empty()) {
        APP_LOG_EVENT_ERROR("camera.calibration",
                           "camera_input",
                           "fail",
                           {{"reason", "first_image_unreadable"},
                            {"path", QString::fromStdString(img_files[0])}});
        return false;
    }
    image_size = first.size();
    return true;
}

bool calibrateCameraFromDisk(const std::string& save_dir,
                              CharucoDetector& detector,
                              cv::Mat& camera_matrix,
                              cv::Mat& dist_coeffs) {

    const auto totalStartedAt = Clock::now();
    const auto& board_points = detector.getChessboardCorners3D();

    std::vector<std::string> img_files;
    cv::Size image_size;
    if (!loadImagesFromDisk(save_dir, img_files, image_size)) {
        APP_LOG_EVENT_ERROR("camera.calibration",
                            "camera_pipeline",
                            "fail",
                            {{"stage", "image_discovery"},
                             {"reason", "input_load_failed"},
                             {"directory", QString::fromStdString(save_dir)},
                             {"elapsed_ms", QString::number(elapsedMilliseconds(totalStartedAt))}});
        return false;
    }

    APP_LOG_EVENT_INFO("camera.calibration",
                       "camera_pipeline",
                       "start",
                       {{"directory", QString::fromStdString(save_dir)},
                        {"discovered_images", QString::number(img_files.size())},
                        {"image_w", QString::number(image_size.width)},
                        {"image_h", QString::number(image_size.height)},
                        {"initial_cap", QString::number(kMaxInitialOptimizationFrames)},
                        {"refine_cap", QString::number(kMaxRefineOptimizationFrames)}});
    std::cout << "[cameraCalib] 发现 " << img_files.size()
              << " 张图像，开始检测 ChArUco 角点" << std::endl;

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    size_t unreadableImages = 0;
    size_t detectionFailures = 0;
    size_t insufficientCorners = 0;
    const auto detectionStartedAt = Clock::now();

    for (size_t fileIndex = 0; fileIndex < img_files.size(); ++fileIndex) {
        const auto& path = img_files[fileIndex];
        cv::Mat img = cv::imread(path);
        if (img.empty()) {
            ++unreadableImages;
            continue;
        }

        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detector.detectCorners(img, corners, ids)) {
            ++detectionFailures;
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
        if (pts3.size() >= 16) {
            object_points.push_back(pts3);
            image_points.push_back(pts2);
        } else {
            ++insufficientCorners;
        }

        const size_t processed = fileIndex + 1;
        if (processed % 25 == 0 || processed == img_files.size()) {
            APP_LOG_EVENT_DEBUG("camera.calibration",
                                "camera_input",
                                "progress",
                                {{"processed", QString::number(processed)},
                                 {"total", QString::number(img_files.size())},
                                 {"valid", QString::number(object_points.size())},
                                 {"unreadable", QString::number(unreadableImages)},
                                 {"detection_failed", QString::number(detectionFailures)},
                                 {"insufficient_corners", QString::number(insufficientCorners)},
                                 {"elapsed_ms", QString::number(elapsedMilliseconds(detectionStartedAt))}});
        }
    }

    const long long detectionMs = elapsedMilliseconds(detectionStartedAt);
    std::cout << "[cameraCalib] 角点检测完成: 有效 " << object_points.size()
              << "/" << img_files.size() << " 帧, " << detectionMs << " ms" << std::endl;
    APP_LOG_EVENT_INFO("camera.calibration",
                       "camera_input",
                       "complete",
                       {{"total", QString::number(img_files.size())},
                        {"valid", QString::number(object_points.size())},
                        {"unreadable", QString::number(unreadableImages)},
                        {"detection_failed", QString::number(detectionFailures)},
                        {"insufficient_corners", QString::number(insufficientCorners)},
                        {"elapsed_ms", QString::number(detectionMs)}});

    if (object_points.size() < 5) {
        APP_LOG_EVENT_ERROR("camera.calibration",
                            "camera_pipeline",
                            "fail",
                            {{"stage", "detection"},
                             {"reason", "insufficient_frames"},
                             {"valid_frames", QString::number(object_points.size())},
                             {"minimum", "5"}});
        std::cerr << "[autoCalib] 帧数不足，无法标定相机内参 (need >=5, got "
                  << object_points.size() << ")" << std::endl;
        return false;
    }

    // 保存全部有效帧，仅供快速 solvePnP 误差评估；非线性优化会先限帧。
    auto all_obj = object_points;
    auto all_img = image_points;

    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    if (!calibrateCameraFiltered(object_points, image_points,
                                 image_size, camera_matrix, dist_coeffs)) {
        APP_LOG_EVENT_ERROR("camera.calibration",
                            "camera_pipeline",
                            "fail",
                            {{"stage", "initial_optimization"},
                             {"reason", "initial_stage_failed"},
                             {"elapsed_ms", QString::number(elapsedMilliseconds(totalStartedAt))}});
        return false;
    }

    // 全部帧只做快速误差评估，第二次非线性优化限制为代表性内点子集。
    const bool refined = refineCameraIntrinsics(camera_matrix, dist_coeffs,
                                                all_obj, all_img, image_size);
    if (!refined) {
        APP_LOG_EVENT_WARNING("camera.calibration",
                              "camera_pipeline",
                              "skip",
                              {{"stage", "refinement"},
                               {"reason", "not_applied"},
                               {"using_initial_intrinsics", "1"}});
        std::cout << "[cameraCalib] 精化未应用，保留初始内参" << std::endl;
    }

    const long long totalMs = elapsedMilliseconds(totalStartedAt);
    APP_LOG_EVENT_INFO("camera.calibration",
                       "camera_pipeline",
                       "success",
                       {{"discovered", QString::number(img_files.size())},
                        {"valid", QString::number(all_obj.size())},
                        {"refined", refined ? "1" : "0"},
                        {"total_ms", QString::number(totalMs)}});
    std::cout << "[cameraCalib] 相机内参标定完成，总耗时 " << totalMs
              << " ms" << std::endl;
    return true;
}
