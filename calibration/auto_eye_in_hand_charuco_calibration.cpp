#include "auto_eye_in_hand_charuco_calibration.hpp"

#include "cameras/AICamera.hpp"


#include "robotapi.h"
#include "basestruct.h"

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>
#include <random>
#include <array>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <limits>

#include "utils/robot_util.hpp"
#include "utils/calib_transform.hpp"
#include "hand_eye_gui/DiagnosticLogger.hpp"
#include "calibration_utils.hpp"
#include "calibration_session.hpp"
#include "intrinsic_identity.hpp"
#include "semi_auto_eye_in_hand_calibration.hpp"
#include "mechanical_settling_config.hpp"

using namespace calib_transform;

std::string g_save_dir = "";
AI_det_callback g_callback = nullptr;

// ============================================================
// 内部数据结构
// ============================================================

struct CapturedFrame {
    cv::Mat T_base_flange;              // 4x4 机器人法兰位姿
    std::vector<cv::Point2f> corners;   // ChArUco 角点（像素）
    std::vector<int> ids;               // ChArUco 角点 ID
    cv::Mat image;                      // 原始图像（用于重检测）
};

namespace {

constexpr int kPoseSearchDiscardFrames = 1;
constexpr int kMinimumObservationCorners = 12;
constexpr double kTargetBoardAreaRatio = 0.275;
constexpr double kTargetPlaneTiltDegrees = 2.0;
constexpr double kMaximumPlaneTiltDegrees = 8.0;

struct VisualAlignmentMetrics {
    bool image_valid = false;
    bool pose_valid = false;
    int corners = 0;
    int pose_corners = 0;
    double center_offset_ratio = std::numeric_limits<double>::infinity();
    double area_ratio = 0.0;
    double roll_degrees = std::numeric_limits<double>::infinity();
    double plane_tilt_degrees = std::numeric_limits<double>::infinity();
    double reprojection_rms = std::numeric_limits<double>::infinity();
};

struct ObservationReferencePose {
    std::array<double, ROB_AXIS_NUM> joint{};
    std::array<double, ROB_AXIS_NUM> flange_euler{};
};

static bool waitForPoseSearchSettling(
    const std::shared_ptr<AICamera>& camera,
    int* discarded_frames = nullptr) {
    constexpr int pollMs = 20;
    const int requestedMs = mechanicalSettlingWaitMs();
    int waitedMs = 0;
    while (waitedMs < requestedMs) {
        if (robotMotionCancellationRequested())
            return false;
        const int sliceMs = std::min(pollMs, requestedMs - waitedMs);
        std::this_thread::sleep_for(std::chrono::milliseconds(sliceMs));
        waitedMs += sliceMs;
    }

    int discarded = 0;
    for (int index = 0; index < kPoseSearchDiscardFrames; ++index) {
        if (robotMotionCancellationRequested())
            return false;
        if (camera && !camera->captureOneFrame(500).empty())
            ++discarded;
    }
    if (discarded_frames)
        *discarded_frames = discarded;
    return true;
}

static bool measureVisualAlignment(
    const std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    VisualAlignmentMetrics& metrics) {
    metrics = VisualAlignmentMetrics{};
    if (!camera)
        return false;

    const cv::Mat frame = camera->captureOneFrame(1500);
    if (frame.empty())
        return false;
    metrics.image_valid = true;

    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
    detector.detectCorners(frame, corners, ids);
    metrics.corners = static_cast<int>(ids.size());
    if (corners.size() < 2 || corners.size() != ids.size())
        return true;

    cv::Point2f boardCenter(0.0f, 0.0f);
    for (const cv::Point2f& corner : corners)
        boardCenter += corner;
    boardCenter *= 1.0f / static_cast<float>(corners.size());
    const cv::Point2f imageCenter(frame.cols * 0.5f, frame.rows * 0.5f);
    metrics.center_offset_ratio =
        cv::norm(boardCenter - imageCenter) /
        static_cast<double>(std::min(frame.cols, frame.rows));

    std::vector<cv::Point2f> hull;
    cv::convexHull(corners, hull);
    const cv::Size boardSize = detector.getChessboardSize();
    const int totalCorners = (boardSize.width - 1) * (boardSize.height - 1);
    const double observedArea = std::abs(cv::contourArea(hull));
    const double estimatedBoardArea = observedArea * totalCorners /
        static_cast<double>(std::max<std::size_t>(corners.size(), 1));
    metrics.area_ratio = estimatedBoardArea /
        static_cast<double>(frame.cols * frame.rows);
    metrics.roll_degrees = std::abs(
        computeBoardAngle(corners, ids, boardSize.width, boardSize.height,
                          frame.cols, frame.rows) * 180.0 / CV_PI);

    if (detector.getCameraMatrix().empty())
        return true;

    cv::Mat rvec;
    cv::Mat tvec;
    std::vector<cv::Point2f> poseCorners;
    std::vector<int> poseIds;
    if (!detector.detectPose(frame, rvec, tvec, poseCorners, poseIds,
                             kMinimumObservationCorners)) {
        return true;
    }

    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    if (rotation.depth() != CV_64F)
        rotation.convertTo(rotation, CV_64F);
    const double normalZ = std::clamp(
        std::abs(rotation.at<double>(2, 2)), 0.0, 1.0);
    metrics.plane_tilt_degrees = std::acos(normalZ) * 180.0 / CV_PI;
    metrics.pose_corners = static_cast<int>(poseIds.size());

    const std::vector<cv::Point3f>& boardCorners =
        detector.getChessboardCorners3D();
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    objectPoints.reserve(poseIds.size());
    imagePoints.reserve(poseIds.size());
    for (std::size_t index = 0; index < poseIds.size(); ++index) {
        const int id = poseIds[index];
        if (id < 0 || id >= static_cast<int>(boardCorners.size()))
            continue;
        objectPoints.push_back(boardCorners[id]);
        imagePoints.push_back(poseCorners[index]);
    }
    if (objectPoints.size() < static_cast<std::size_t>(kMinimumObservationCorners))
        return true;

    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, detector.getCameraMatrix(),
                      detector.getDistCoeffs(), projected);
    double squaredError = 0.0;
    for (std::size_t index = 0; index < projected.size(); ++index) {
        const cv::Point2f residual = projected[index] - imagePoints[index];
        squaredError += residual.dot(residual);
    }
    metrics.reprojection_rms = std::sqrt(
        squaredError / static_cast<double>(projected.size()));
    metrics.pose_valid = true;
    return true;
}

static std::string formatVisualAlignmentMetrics(
    const VisualAlignmentMetrics& metrics) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4)
           << "corners=" << metrics.corners
           << " pose_corners=" << metrics.pose_corners
           << " center_ratio=" << metrics.center_offset_ratio
           << " area_ratio=" << metrics.area_ratio
           << " roll_deg=" << metrics.roll_degrees
           << " tilt_deg=" << metrics.plane_tilt_degrees
           << " reproj_px=" << metrics.reprojection_rms;
    return stream.str();
}

static double visualAlignmentCost(const VisualAlignmentMetrics& metrics,
                                  int totalCorners) {
    if (!metrics.pose_valid)
        return std::numeric_limits<double>::infinity();
    const int missingCorners = std::max(0, totalCorners - metrics.pose_corners);
    return metrics.plane_tilt_degrees + missingCorners * 0.25 +
        metrics.center_offset_ratio * 2.0;
}

// ============================================================
// 评价函数 + XY 搜索（共用）
// ============================================================

// 评价当前位姿检测质量
static std::tuple<double, int, cv::Point2f> evaluatePose(
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector) {
    if (!camera) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "pose_evaluation",
                            "fail",
                            {{"reason", "null_camera"}});
        return {0, 0, cv::Point2f(0, 0)};
    }
    cv::Mat f = camera->captureOneFrame(1500);
    if (f.empty()) {
        APP_LOG_EVENT_WARNING("calibration.pose_search",
                              "pose_evaluation",
                              "fail",
                              {{"reason", "empty_frame"}});
        return {0, 0, cv::Point2f(0, 0)};
    }

        if (nullptr!=g_callback) {
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = now.time_since_epoch();
            auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();

            std::string file_id = std::to_string(nanoseconds);

            std::string img_name = g_save_dir + "/" +  file_id  + ".jpg";
            cv::imwrite(img_name, f);

            int call_ret = g_callback((char*)img_name.c_str(), (int)img_name.length(), 1);
            if (call_ret == -1) {
                APP_LOG_EVENT_WARNING("calibration.pose_search",
                                      "pose_evaluation",
                                      "skip",
                                      {{"reason", "cancelled"},
                                       {"stage", "capture_callback"}});
                return {-std::numeric_limits<double>::infinity(), 0,
                        cv::Point2f(0, 0)};
            }
        }


    double cx = f.cols / 2.0;
    double cy = f.rows / 2.0;
    std::vector<cv::Point2f> c;
    std::vector<int> i;
    if (!detector.detectCorners(f, c, i))
        return {0, (int)i.size(), cv::Point2f(0, 0)};
    cv::Size board_sz = detector.getChessboardSize();

    std::vector<cv::Point2f> h;
    cv::convexHull(c, h);
    cv::Point2f bc(0, 0);
    for (auto& pt : c) { bc.x += pt.x; bc.y += pt.y; }
    bc.x /= (float)c.size(); bc.y /= (float)c.size();
    double pd = ((bc.x-cx)*(bc.x-cx) + (bc.y-cy)*(bc.y-cy));
    double angle_diff = -computeBoardAngle(c, i, board_sz.width, board_sz.height, f.cols, f.rows) * 180.0 / CV_PI;
    double hull_area = cv::contourArea(h);
    double    score = (double)i.size() * 100000.0 + hull_area - pd - fabs(angle_diff) * 100;
    return {score, (int)i.size(), bc};
}

// 优先使用操作员已经示教、且当前能看见标定板的安全观察位姿。只有当前
// 位姿无法形成有效观测时，才回退到历史的绝对法兰姿态。
static bool selectInitialObservationPose(
    int robotHandle,
    const std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    double flangeEuler[ROB_AXIS_NUM],
    std::string* failureMessage) {
    if (!camera) {
        if (failureMessage)
            *failureMessage = "自动寻找最佳位姿失败：相机未初始化";
        return false;
    }

    if (!waitForPoseSearchSettling(camera)) {
        if (failureMessage)
            *failureMessage = "自动寻找最佳位姿失败：等待初始观察位姿稳定时取消";
        return false;
    }
    const cv::Mat initialFrame = camera->captureOneFrame(1500);
    if (initialFrame.empty()) {
        if (failureMessage)
            *failureMessage = "自动寻找最佳位姿失败：相机采集不到图像";
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "initial_observation",
                            "fail",
                            {{"reason", "empty_frame"}});
        return false;
    }

    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
    detector.detectCorners(initialFrame, corners, ids);
    if (ids.size() >= 4) {
        double tcpAxisAngle[ROB_AXIS_NUM] = {};
        const CRresult tcpResult = cr_get_tcpActualPose(
            (RobotHandle)robotHandle, tcpAxisAngle);
        if (tcpResult != sucess ||
            !convertTcpAxisPos2FlangeEule(robotHandle, tcpAxisAngle,
                                          flangeEuler)) {
            if (failureMessage)
                *failureMessage = "自动寻找最佳位姿失败：读取示教观察位姿失败";
            APP_LOG_EVENT_ERROR("calibration.pose_search",
                                "initial_observation",
                                "fail",
                                {{"reason", "taught_pose_read_failed"},
                                 {"error", QString::number(tcpResult)}});
            return false;
        }
        APP_LOG_EVENT_INFO("calibration.pose_search",
                           "initial_observation",
                           "selected",
                           {{"source", "current_taught_pose"},
                            {"corners", QString::number(ids.size())},
                            {"minimum_reliable_corners",
                             QString::number(kMinimumObservationCorners)}});
        return true;
    }

    APP_LOG_EVENT_WARNING("calibration.pose_search",
                          "initial_observation",
                          "fallback",
                          {{"source", "legacy_absolute_flange_pose"},
                           {"reason", "board_not_visible_from_current_pose"},
                           {"corners", QString::number(ids.size())},
                           {"target_rx", "-180"},
                           {"target_ry", "0"}});
    if (!rotateFlangeToHorizon(robotHandle, flangeEuler)) {
        if (failureMessage)
            *failureMessage = "机器人回退水平姿态调整失败";
        return false;
    }
    if (!waitForPoseSearchSettling(camera)) {
        if (failureMessage)
            *failureMessage = "机器人回退水平姿态后稳定等待失败";
        return false;
    }
    return true;
}

enum class VisualAlignmentCandidateResult {
    Measured,
    Rejected,
    Failed
};

static VisualAlignmentCandidateResult moveAndMeasureVisualAlignment(
    int robotHandle,
    const std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    const std::array<double, ROB_AXIS_NUM>& target,
    VisualAlignmentMetrics& metrics) {
    std::array<double, ROB_AXIS_NUM> movableTarget = target;
    double currentJoint[ROB_AXIS_NUM] = {};
    double targetJoint[ROB_AXIS_NUM] = {};
    const CRresult jointResult = cr_get_jointActualPos(
        (RobotHandle)robotHandle, currentJoint);
    if (jointResult != sucess ||
        !convertFlangeOulePos2Joint(robotHandle, movableTarget.data(),
                                    targetJoint)) {
        APP_LOG_EVENT_WARNING("calibration.pose_search",
                              "visual_alignment_candidate",
                              "reject",
                              {{"reason", "ik_unavailable"},
                               {"error", QString::number(jointResult)},
                               {"target_rx", QString::number(target[3], 'f', 3)},
                               {"target_ry", QString::number(target[4], 'f', 3)}});
        return VisualAlignmentCandidateResult::Rejected;
    }

    double maximumJointStep = 0.0;
    int maximumJointStepAxis = 0;
    for (int axis = 0; axis < ROB_AXIS_NUM; ++axis) {
        const double step = std::abs(targetJoint[axis] - currentJoint[axis]);
        if (step > maximumJointStep) {
            maximumJointStep = step;
            maximumJointStepAxis = axis;
        }
    }
    constexpr double maximumVisualAlignmentJointStepDegrees = 20.0;
    if (maximumJointStep > maximumVisualAlignmentJointStepDegrees) {
        APP_LOG_EVENT_WARNING("calibration.pose_search",
                              "visual_alignment_candidate",
                              "reject",
                              {{"reason", "joint_step_gate_exceeded"},
                               {"maximum_step_deg",
                                QString::number(maximumJointStep, 'f', 3)},
                               {"maximum_step_axis",
                                QString("J%1").arg(maximumJointStepAxis + 1)},
                               {"limit_deg", QString::number(
                                    maximumVisualAlignmentJointStepDegrees,
                                    'f', 1)},
                               {"target_rx", QString::number(target[3], 'f', 3)},
                               {"target_ry", QString::number(target[4], 'f', 3)}});
        return VisualAlignmentCandidateResult::Rejected;
    }

    if (moveToJointPos(robotHandle, targetJoint) != sucess) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "visual_alignment_motion",
                            "fail",
                            {{"reason", "move_failed"},
                             {"target_rx", QString::number(target[3], 'f', 3)},
                             {"target_ry", QString::number(target[4], 'f', 3)},
                             {"target_rz", QString::number(target[5], 'f', 3)}});
        return VisualAlignmentCandidateResult::Failed;
    }
    int discardedFrames = 0;
    if (!waitForPoseSearchSettling(camera, &discardedFrames))
        return VisualAlignmentCandidateResult::Failed;
    if (!measureVisualAlignment(camera, detector, metrics))
        return VisualAlignmentCandidateResult::Failed;
    APP_LOG_EVENT_DEBUG("calibration.pose_search",
                        "visual_alignment_sample",
                        "measured",
                        {{"target_rx", QString::number(target[3], 'f', 3)},
                         {"target_ry", QString::number(target[4], 'f', 3)},
                         {"target_rz", QString::number(target[5], 'f', 3)},
                         {"settling_ms", QString::number(mechanicalSettlingWaitMs())},
                         {"discarded_frames", QString::number(discardedFrames)},
                         {"metrics", QString::fromStdString(
                             formatVisualAlignmentMetrics(metrics))}});
    return VisualAlignmentCandidateResult::Measured;
}

// 手眼外参尚未知时，不能可靠地把相机坐标系中的法向误差一次转换到法兰
// 坐标系。这里用受限的小角度坐标下降直接观察 PnP 法向改善，既不依赖旧
// 手眼矩阵，也不会产生大范围盲动。
static bool alignCameraPlaneToBoard(
    int robotHandle,
    const std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    std::string* failureMessage) {
    if (detector.getCameraMatrix().empty()) {
        APP_LOG_EVENT_INFO("calibration.pose_search",
                           "visual_plane_alignment",
                           "skip",
                           {{"reason", "intrinsics_unavailable"},
                            {"fallback", "image_geometry_only"}});
        return true;
    }

    double tcpAxisAngle[ROB_AXIS_NUM] = {};
    double flangeEuler[ROB_AXIS_NUM] = {};
    const CRresult tcpResult = cr_get_tcpActualPose(
        (RobotHandle)robotHandle, tcpAxisAngle);
    if (tcpResult != sucess ||
        !convertTcpAxisPos2FlangeEule(robotHandle, tcpAxisAngle, flangeEuler)) {
        if (failureMessage)
            *failureMessage = "视觉调正失败：无法读取当前法兰位姿";
        return false;
    }

    std::array<double, ROB_AXIS_NUM> seed{};
    std::copy(flangeEuler, flangeEuler + ROB_AXIS_NUM, seed.begin());
    std::array<double, ROB_AXIS_NUM> bestPose = seed;
    VisualAlignmentMetrics bestMetrics;
    if (!waitForPoseSearchSettling(camera) ||
        !measureVisualAlignment(camera, detector, bestMetrics) ||
        !bestMetrics.pose_valid) {
        if (failureMessage)
            *failureMessage = "视觉调正失败：可信内参下无法获得有效 ChArUco PnP 位姿";
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "visual_plane_alignment",
                            "fail",
                            {{"reason", "initial_pnp_invalid"},
                             {"metrics", QString::fromStdString(
                                 formatVisualAlignmentMetrics(bestMetrics))}});
        return false;
    }

    const cv::Size boardSize = detector.getChessboardSize();
    const int totalCorners = (boardSize.width - 1) * (boardSize.height - 1);
    double bestCost = visualAlignmentCost(bestMetrics, totalCorners);
    APP_LOG_EVENT_INFO("calibration.pose_search",
                       "visual_plane_alignment",
                       "start",
                       {{"strategy", "bounded_coordinate_descent"},
                        {"target_tilt_deg",
                         QString::number(kTargetPlaneTiltDegrees, 'f', 2)},
                        {"maximum_total_axis_offset_deg", "10.0"},
                        {"metrics", QString::fromStdString(
                            formatVisualAlignmentMetrics(bestMetrics))}});

    constexpr std::array<double, 4> steps{{4.0, 2.0, 1.0, 0.5}};
    constexpr double maximumAxisOffsetDegrees = 10.0;
    for (std::size_t iteration = 0;
         iteration < steps.size() &&
         bestMetrics.plane_tilt_degrees > kTargetPlaneTiltDegrees;
         ++iteration) {
        const double step = steps[iteration];
        const std::array<double, ROB_AXIS_NUM> iterationOrigin = bestPose;
        std::array<double, ROB_AXIS_NUM> iterationBestPose = bestPose;
        VisualAlignmentMetrics iterationBestMetrics = bestMetrics;
        double iterationBestCost = bestCost;

        for (int axis : {3, 4}) {
            for (double direction : {-1.0, 1.0}) {
                std::array<double, ROB_AXIS_NUM> candidate = iterationOrigin;
                candidate[axis] += direction * step;
                if (std::abs(candidate[axis] - seed[axis]) >
                    maximumAxisOffsetDegrees) {
                    continue;
                }

                VisualAlignmentMetrics candidateMetrics;
                const VisualAlignmentCandidateResult candidateResult =
                    moveAndMeasureVisualAlignment(robotHandle, camera, detector,
                                                  candidate, candidateMetrics);
                if (candidateResult ==
                    VisualAlignmentCandidateResult::Rejected) {
                    continue;
                }
                if (candidateResult == VisualAlignmentCandidateResult::Failed) {
                    if (failureMessage)
                        *failureMessage = "视觉调正失败：候选姿态运动或采图失败";
                    return false;
                }
                const double candidateCost = visualAlignmentCost(
                    candidateMetrics, totalCorners);
                if (candidateCost + 0.05 < iterationBestCost) {
                    iterationBestCost = candidateCost;
                    iterationBestPose = candidate;
                    iterationBestMetrics = candidateMetrics;
                }
            }
        }

        VisualAlignmentMetrics confirmedMetrics;
        const VisualAlignmentCandidateResult confirmationResult =
            moveAndMeasureVisualAlignment(robotHandle, camera, detector,
                                          iterationBestPose,
                                          confirmedMetrics);
        if (confirmationResult != VisualAlignmentCandidateResult::Measured ||
            !confirmedMetrics.pose_valid) {
            if (failureMessage)
                *failureMessage = "视觉调正失败：无法回到本轮最佳姿态";
            return false;
        }
        iterationBestMetrics = confirmedMetrics;
        iterationBestCost = visualAlignmentCost(confirmedMetrics, totalCorners);
        if (iterationBestCost + 0.05 >= bestCost) {
            VisualAlignmentMetrics recoveredMetrics;
            const VisualAlignmentCandidateResult recoveryResult =
                moveAndMeasureVisualAlignment(robotHandle, camera, detector,
                                              bestPose, recoveredMetrics);
            if (recoveryResult != VisualAlignmentCandidateResult::Measured ||
                !recoveredMetrics.pose_valid) {
                if (failureMessage)
                    *failureMessage = "视觉调正失败：无法恢复上一最佳姿态";
                return false;
            }
            bestMetrics = recoveredMetrics;
            bestCost = visualAlignmentCost(bestMetrics, totalCorners);
            APP_LOG_EVENT_INFO("calibration.pose_search",
                               "visual_plane_alignment",
                               "converged",
                               {{"reason", "no_further_improvement"},
                                {"iteration", QString::number(iteration)},
                                {"metrics", QString::fromStdString(
                                    formatVisualAlignmentMetrics(bestMetrics))}});
            break;
        }
        bestPose = iterationBestPose;
        bestMetrics = iterationBestMetrics;
        bestCost = iterationBestCost;
    }

    VisualAlignmentMetrics finalMetrics;
    if (!measureVisualAlignment(camera, detector, finalMetrics) ||
        !finalMetrics.pose_valid) {
        if (failureMessage)
            *failureMessage = "视觉调正失败：最终姿态无法通过 PnP 复核";
        return false;
    }
    if (finalMetrics.plane_tilt_degrees > kMaximumPlaneTiltDegrees) {
        if (failureMessage) {
            *failureMessage = "视觉调正失败：相机光轴与标定板法向偏差 " +
                std::to_string(finalMetrics.plane_tilt_degrees) +
                "°，超过 8° 安全门限";
        }
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "visual_plane_alignment",
                            "fail",
                            {{"reason", "tilt_gate_exceeded"},
                             {"metrics", QString::fromStdString(
                                 formatVisualAlignmentMetrics(finalMetrics))}});
        return false;
    }

    APP_LOG_EVENT_INFO("calibration.pose_search",
                       "visual_plane_alignment",
                       "success",
                       {{"target_met", finalMetrics.plane_tilt_degrees <=
                            kTargetPlaneTiltDegrees ? "true" : "false"},
                        {"metrics", QString::fromStdString(
                            formatVisualAlignmentMetrics(finalMetrics))}});
    return true;
}

static bool validateFinalObservationPose(
    const std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    std::string* failureMessage) {
    VisualAlignmentMetrics metrics;
    if (!waitForPoseSearchSettling(camera) ||
        !measureVisualAlignment(camera, detector, metrics) ||
        !metrics.image_valid) {
        if (failureMessage)
            *failureMessage = "观察位姿质量门禁失败：无法采集最终复核图像";
        return false;
    }

    const cv::Size boardSize = detector.getChessboardSize();
    const int totalCorners = (boardSize.width - 1) * (boardSize.height - 1);
    const int requiredCorners = std::max(
        kMinimumObservationCorners,
        static_cast<int>(std::ceil(totalCorners * 0.8)));

    std::string reason;
    if (metrics.corners < requiredCorners) {
        reason = "insufficient_corners";
    } else if (metrics.center_offset_ratio > 0.18) {
        reason = "board_not_centered";
    } else if (metrics.area_ratio < 0.08 || metrics.area_ratio > 0.55) {
        reason = "board_area_out_of_range";
    } else if (metrics.roll_degrees > 5.0) {
        reason = "board_roll_exceeded";
    } else if (!detector.getCameraMatrix().empty() && !metrics.pose_valid) {
        reason = "pnp_invalid";
    } else if (metrics.pose_valid &&
               metrics.plane_tilt_degrees > kMaximumPlaneTiltDegrees) {
        reason = "plane_tilt_exceeded";
    } else if (metrics.pose_valid && metrics.reprojection_rms > 1.0) {
        reason = "pnp_reprojection_exceeded";
    }

    if (!reason.empty()) {
        if (failureMessage) {
            *failureMessage = "观察位姿质量门禁失败: " + reason + " (" +
                formatVisualAlignmentMetrics(metrics) + ")";
        }
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "observation_quality_gate",
                            "fail",
                            {{"reason", QString::fromStdString(reason)},
                             {"required_corners", QString::number(requiredCorners)},
                             {"metrics", QString::fromStdString(
                                 formatVisualAlignmentMetrics(metrics))}});
        return false;
    }

    APP_LOG_EVENT_INFO("calibration.pose_search",
                       "observation_quality_gate",
                       "pass",
                       {{"required_corners", QString::number(requiredCorners)},
                        {"target_center_ratio", "0.05"},
                        {"target_area_ratio", "0.20-0.30"},
                        {"target_roll_deg", "1.0"},
                        {"target_tilt_deg",
                         QString::number(kTargetPlaneTiltDegrees, 'f', 2)},
                        {"metrics", QString::fromStdString(
                            formatVisualAlignmentMetrics(metrics))}});
    return true;
}

static bool captureObservationReferencePose(
    int robotHandle,
    ObservationReferencePose& pose,
    std::string& error) {
    double tcpAxisAngle[ROB_AXIS_NUM] = {};
    const CRresult jointResult = cr_get_jointActualPos(
        (RobotHandle)robotHandle, pose.joint.data());
    const CRresult tcpResult = cr_get_tcpActualPose(
        (RobotHandle)robotHandle, tcpAxisAngle);
    if (jointResult != sucess || tcpResult != sucess) {
        error = "读取机器人实际关节或 TCP 位姿失败";
        return false;
    }
    if (!convertTcpAxisPos2FlangeEule(robotHandle, tcpAxisAngle,
                                      pose.flange_euler.data())) {
        error = "TCP 位姿转换为法兰欧拉位姿失败";
        return false;
    }
    return true;
}

static bool saveObservationReferencePose(
    const std::filesystem::path& destination,
    const ObservationReferencePose& pose,
    std::string& error) {
    if (destination.empty()) {
        error = "观察位姿输出路径为空";
        return false;
    }

    std::error_code fileError;
    std::filesystem::create_directories(destination.parent_path(), fileError);
    if (fileError) {
        error = "无法创建观察位姿目录: " + fileError.message();
        return false;
    }

    const std::filesystem::path temporary =
        destination.string() + ".tmp.yaml";
    try {
        cv::FileStorage storage(temporary.string(), cv::FileStorage::WRITE);
        if (!storage.isOpened()) {
            error = "无法创建观察位姿临时文件";
            return false;
        }
        storage << "joint_deg" << "[";
        for (double value : pose.joint)
            storage << value;
        storage << "]";
        storage << "flange_euler" << "[";
        for (double value : pose.flange_euler)
            storage << value;
        storage << "]";
        storage.release();
    } catch (const cv::Exception& exception) {
        std::filesystem::remove(temporary, fileError);
        error = "写入观察位姿失败: " + std::string(exception.what());
        return false;
    }

    const std::uintmax_t bytes = std::filesystem::file_size(
        temporary, fileError);
    if (fileError || bytes == 0) {
        std::filesystem::remove(temporary, fileError);
        error = "观察位姿临时文件校验失败";
        return false;
    }
    std::filesystem::rename(temporary, destination, fileError);
    if (fileError) {
        std::filesystem::remove(temporary);
        error = "发布观察位姿失败: " + fileError.message();
        return false;
    }

    APP_LOG_EVENT_INFO("calibration.pose_search",
                       "observation_reference_pose",
                       "published",
                       {{"path", QString::fromStdString(destination.string())},
                        {"joint_count", QString::number(ROB_AXIS_NUM)},
                        {"flange_value_count", QString::number(ROB_AXIS_NUM)}});
    return true;
}

static std::filesystem::path observationReferencePosePath(
    const std::string& outputYaml,
    const std::string& intrinsicSourceFile) {
    std::filesystem::path parameterDirectory;
    if (!outputYaml.empty()) {
        const std::filesystem::path outputPath(outputYaml);
        parameterDirectory = outputPath.has_parent_path()
            ? outputPath.parent_path() : std::filesystem::path(".");
    } else if (!intrinsicSourceFile.empty()) {
        const std::filesystem::path intrinsicPath(intrinsicSourceFile);
        parameterDirectory = intrinsicPath.has_parent_path()
            ? intrinsicPath.parent_path() : std::filesystem::path(".");
    }
    if (parameterDirectory.empty())
        return {};
    return parameterDirectory / "eye_in_hand_observation_pose.yaml";
}



// 尝试一组候选欧拉位姿，选评分最高的
// candidates: 欧拉位姿列表 [x,y,z,rx,ry,rz] (mm, deg)
// best_joint: 输出，最佳候选的关节角
// 返回最佳评分（运动或基础设施失败时返回负无穷，调用方必须立即终止）
// 注意：函数会临时运动到候选位姿做评估，返回前会回到原始位置
static constexpr double kCandidateSearchFailure =
    -std::numeric_limits<double>::infinity();

static double tryCandidatePoses(int robot_handle,
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    const char* search_phase,
    const std::vector<std::array<double, 6>>& candidates,
    double bestFlangeEule[6]) {
    // 保存原始关节角
    double initJointPos[ROB_AXIS_NUM], tcp_aa[ROB_AXIS_NUM];
    CRresult ret = cr_get_jointActualPos((RobotHandle)robot_handle, initJointPos);
    if (ret != sucess) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "candidate_search",
                            "fail",
                            {{"reason", "joint_read"},
                             {"error", QString::number(ret)}});
        return kCandidateSearchFailure;
    }
    ret = cr_get_tcpActualPose(robot_handle, tcp_aa);
    if (ret != sucess) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "candidate_search",
                            "fail",
                            {{"reason", "tcp_read"},
                             {"error", QString::number(ret)}});
        return kCandidateSearchFailure;
    }
    
    double initialFlangeEule[ROB_AXIS_NUM];
    double bestFlangeEule_j[ROB_AXIS_NUM];
    double best_s = -1e18;
    bool found = false;
    if (!convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, initialFlangeEule)) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "candidate_search",
                            "fail",
                            {{"reason", "tcp_to_flange_conversion"}});
        return kCandidateSearchFailure;
    }
    const cv::Size boardSize = detector.getChessboardSize();
    const int totalCorners = (boardSize.width - 1) * (boardSize.height - 1);
    int candidateIndex = 0;
    for (const auto& euler : candidates) {
             
        if(moveToFlangeEulePos(robot_handle, (double*)euler.data())!= true) {
            if (robotMotionCancellationRequested()) {
                APP_LOG_EVENT_WARNING("calibration.pose_search",
                                      "candidate_search",
                                      "cancelled",
                                      {{"stage", "robot_move"}});
                return kCandidateSearchFailure;
            }
            APP_LOG_EVENT_ERROR("calibration.pose_search",
                                "candidate_search",
                                "fail",
                                {{"reason", "candidate_motion_failed"},
                                 {"target", QString("[%1,%2,%3,%4,%5,%6]")
                                     .arg(euler[0], 0, 'f', 3).arg(euler[1], 0, 'f', 3)
                                     .arg(euler[2], 0, 'f', 3).arg(euler[3], 0, 'f', 3)
                                     .arg(euler[4], 0, 'f', 3).arg(euler[5], 0, 'f', 3)}});
            return kCandidateSearchFailure;
        }

        double s; int n; cv::Point2f bc;
        std::tie(s, n, bc) = evaluatePose(camera, detector);
        APP_LOG_EVENT_DEBUG("calibration.pose_search",
                           "candidate_evaluation",
                           "sampled",
                           {{"phase", search_phase ? search_phase : "unknown"},
                            {"index", QString::number(candidateIndex)},
                            {"total", QString::number(candidates.size())},
                            {"offset_x_mm", QString::number(euler[0] - initialFlangeEule[0], 'f', 1)},
                            {"offset_y_mm", QString::number(euler[1] - initialFlangeEule[1], 'f', 1)},
                            {"offset_z_mm", QString::number(euler[2] - initialFlangeEule[2], 'f', 1)},
                            {"corners", QString::number(n)},
                            {"total_corners", QString::number(totalCorners)},
                            {"score", QString::number(s, 'f', 3)},
                            {"board_center_x", QString::number(bc.x, 'f', 1)},
                            {"board_center_y", QString::number(bc.y, 'f', 1)}});
        if (s > best_s) {
            best_s = s;
            std::copy(euler.data(), euler.data() + ROB_AXIS_NUM, bestFlangeEule_j);
            found = true;
        }

        if (nullptr!=g_callback) {
            int call_ret = g_callback(nullptr, 0, 1);
                if (-1==call_ret)
            {
                std::cerr << "\r[autoCalib] 用户取消了  采集进度"  << std::endl;
                APP_LOG_EVENT_WARNING("calibration.pose_search",
                                      "candidate_search",
                                      "cancelled",
                                      {{"reason", "cancelled"},
                                       {"stage", "candidate"}});
                return kCandidateSearchFailure;
            }            
        }

        if (n >= totalCorners) {
            APP_LOG_EVENT_INFO("calibration.pose_search",
                               "candidate_search",
                               "early_stop",
                               {{"phase", search_phase ? search_phase : "unknown"},
                                {"reason", "all_corners_visible"},
                                {"index", QString::number(candidateIndex)},
                                {"corners", QString::number(n)},
                                {"total_corners", QString::number(totalCorners)}});
            break;
        }
        ++candidateIndex;

    }

    // 回到原始位置（由调用方决定是否运动到最佳位姿）
    const CRresult restoreRet = moveToJointPos(robot_handle, initJointPos);
    if (restoreRet != sucess) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "candidate_search",
                            "fail",
                            {{"reason", "restore_position"},
                             {"error", QString::number(restoreRet)}});
        return kCandidateSearchFailure;
    }
    if (!found){          
        return kCandidateSearchFailure;}

    std::copy(bestFlangeEule_j, bestFlangeEule_j + ROB_AXIS_NUM, bestFlangeEule);
    return best_s;
}

// 多步长 XY 搜索（Phase 2 / Phase 4 共用）
static bool searchXYSteps(int robot_handle,
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    const double* steps, int num_steps,
    double tcp_aa[6]) {
    CRresult ret = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
    if (ret != sucess) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "xy_search",
                            "fail",
                            {{"reason", "tcp_read"},
                             {"error", QString::number(ret)}});
        return false;
    }

    double flangeEulePos[ROB_AXIS_NUM] = {};
    if (!convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos)) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "xy_search",
                            "fail",
                            {{"reason", "tcp_to_flange_conversion"}});
        return false;
    }

    for (int i = 0; i < num_steps; i++) {
        double step = steps[i];
        double score; int n; cv::Point2f bc;
        std::tie(score, n, bc) = evaluatePose(camera, detector);
        std::vector<std::array<double, 6>> xy_candidates;
        // 沿邻接的环形路径走完 8 个方向，避免旧笛卡尔顺序在对角点之间来回跨越。
        const std::array<std::array<int, 2>, 8> xyOrder{{
            {{-1, 0}}, {{-1, -1}}, {{0, -1}}, {{1, -1}},
            {{1, 0}}, {{1, 1}}, {{0, 1}}, {{-1, 1}}
        }};
        for (const auto& offset : xyOrder) {
            const int sx = offset[0];
            const int sy = offset[1];
            xy_candidates.push_back({flangeEulePos[0] + sx*step, flangeEulePos[1] + sy*step, flangeEulePos[2],
                                      flangeEulePos[3], flangeEulePos[4], flangeEulePos[5]});
        }
        double bestFlangeEule[ROB_AXIS_NUM];
        const double candidateScore = tryCandidatePoses(
            robot_handle, camera, detector, "coarse_xy",
            xy_candidates, bestFlangeEule);
        if (!std::isfinite(candidateScore)) {
            APP_LOG_EVENT_ERROR("calibration.pose_search",
                                "xy_search",
                                "fail",
                                {{"reason", "candidate_search_failed"},
                                 {"step_index", QString::number(i)},
                                 {"step_mm", QString::number(step)}});
            return false;
        }
        if (candidateScore > score) {
            if (!moveToFlangeEulePos(robot_handle, bestFlangeEule)) {
                APP_LOG_EVENT_ERROR("calibration.pose_search",
                                    "xy_search",
                                    "fail",
                                    {{"reason", "move_best"},
                                     {"step_index", QString::number(i)},
                                     {"step_mm", QString::number(step)}});
                return false;
            }
            //memcpy(flangeEulePos, bestFlangeEule, sizeof(bestFlangeEule));
            ret = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
            if (ret != sucess ||
                !convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos)) {
                APP_LOG_EVENT_ERROR("calibration.pose_search",
                                    "xy_search",
                                    "fail",
                                    {{"reason", "refresh_pose"},
                                     {"step_index", QString::number(i)},
                                     {"tcp_error", QString::number(ret)}});
                return false;
            }
        }

        if (nullptr!=g_callback) {
            int call_ret = g_callback(nullptr, 0, 1);
            if (-1==call_ret)
            {
                std::cerr << "\r[autoCalib] 用户取消了  采集进度"  << std::endl;

                APP_LOG_EVENT_WARNING("calibration.pose_search",
                                      "xy_search",
                                      "skip",
                                      {{"reason", "cancelled"},
                                       {"step_index", QString::number(i)}});
                return false;
            }            
        }           

    }
    if (!moveToTcpAxisPos(robot_handle, tcp_aa)) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "xy_search",
                            "fail",
                            {{"reason", "final_move"}});
        return false;
    }
    return true;
}

// 一次公式反算找最佳位姿：板居中、面积 25-30%、长边平行
bool autoFindBestPose(int robot_handle,
                      std::shared_ptr<AICamera>& camera,
                      CharucoDetector& detector,
                      const std::string& save_dir,
                      AI_det_callback callback,
                      std::string* failure_message) {

    if (failure_message)
        failure_message->clear();

    const int MAX_REFINE_ZRZ = 6;
    const double Z_STEP = 100.0;
    const double Z_MAX_OFFSET = 600.0;
    const double Z_MAX_APPROACH_OFFSET = 200.0;

    // ---- 优先采用当前示教观察位姿；看不到板时才回退历史水平姿态 ----
    double flangeEulePos[ROB_AXIS_NUM] = {};
    double tcp_aa[ROB_AXIS_NUM] = {};
    if (!selectInitialObservationPose(robot_handle, camera, detector,
                                      flangeEulePos, failure_message)) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "best_pose",
                            "fail",
                            {{"phase", "initial_observation"},
                             {"reason", "observation_pose_selection_failed"}});
        return false;
    }
    CRresult tcpRet = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
    if (tcpRet != sucess) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "best_pose",
                            "fail",
                            {{"phase", "initial_tcp_read"},
                             {"error", QString::number(tcpRet)}});
        return false;
    }
    APP_LOG_EVENT_INFO("calibration.pose_search",
                       "best_pose",
                       "start",
                       {{"handle", QString::number(robot_handle)},
                        {"save_dir", QString::fromStdString(save_dir)}});

    // 标定板总角点数
    cv::Size board_sz = detector.getChessboardSize();
    int total_corners = (board_sz.width - 1) * (board_sz.height - 1);


    {
        namespace fs = std::filesystem;
        if (fs::exists(save_dir))
            fs::remove_all(save_dir);
        fs::create_directories(save_dir);
    }    
    


    // ================================================================
    // Phase 1: Z 搜索 — 码太少时向上找更好的高度
    // ================================================================
    cv::Mat frame = camera->captureOneFrame(1500);
    double cur_score;
    int cur_codes;
    cv::Point2f cur_bc;
    std::tie(cur_score, cur_codes, cur_bc) = evaluatePose(camera, detector);

    // 只有完整识别 24/24 时才跳过粗搜索。0 个角点通常意味着起始距离
    // 过远，恰恰需要用粗粒度 Z/XY 搜索恢复，不能直接进入精细拟合。
    if (cur_codes < total_corners) {
        std::cout << "[autoFindBestPose] Phase1: Z 搜索 (当前码=" << cur_codes
                  << "/" << total_corners << ")" << std::endl;
        APP_LOG_EVENT_INFO("calibration.pose_search",
                           "coarse_search",
                           "start",
                           {{"phase", "coarse_z"},
                            {"reason", cur_codes == 0
                                 ? "recover_board_not_detected" : "incomplete_corners"},
                            {"corners", QString::number(cur_codes)},
                            {"total_corners", QString::number(total_corners)}});

        // 现场零角点帧中标定板位于画面中央但尺寸过小。先向靠近标定板的
        // 方向搜索，未改善时再向远离方向扩展视野；两组之间都会回到起点，
        // 避免从 -200 mm 直接跨越到 +600 mm。
        std::vector<std::array<double, 6>> approachCandidates;
        for (int idx = 1; idx * Z_STEP <= Z_MAX_APPROACH_OFFSET; ++idx) {
            approachCandidates.push_back({
                flangeEulePos[0], flangeEulePos[1], flangeEulePos[2] - idx * Z_STEP,
                flangeEulePos[3], flangeEulePos[4], flangeEulePos[5]});
        }
        double bestFlangeEule[ROB_AXIS_NUM];
        double best_s = tryCandidatePoses(
            robot_handle, camera, detector, "coarse_z_approach",
            approachCandidates, bestFlangeEule);
        if (!std::isfinite(best_s)) {
            APP_LOG_EVENT_ERROR("calibration.pose_search",
                                "best_pose",
                                "fail",
                                {{"phase", "z_search"},
                                 {"reason", "candidate_search_failed"}});
            return false;
        }
        if (best_s <= cur_score) {
            std::vector<std::array<double, 6>> retreatCandidates;
            for (int idx = 1; idx * Z_STEP <= Z_MAX_OFFSET; ++idx) {
                retreatCandidates.push_back({
                    flangeEulePos[0], flangeEulePos[1], flangeEulePos[2] + idx * Z_STEP,
                    flangeEulePos[3], flangeEulePos[4], flangeEulePos[5]});
            }
            best_s = tryCandidatePoses(
                robot_handle, camera, detector, "coarse_z_retreat",
                retreatCandidates, bestFlangeEule);
            if (!std::isfinite(best_s)) {
                APP_LOG_EVENT_ERROR("calibration.pose_search",
                                    "best_pose",
                                    "fail",
                                    {{"phase", "z_search"},
                                     {"reason", "retreat_candidate_search_failed"}});
                return false;
            }
        }
        if (best_s > cur_score) {
            if (!moveToFlangeEulePos(robot_handle, bestFlangeEule)) {
                APP_LOG_EVENT_ERROR("calibration.pose_search",
                                    "best_pose",
                                    "fail",
                                    {{"phase", "z_search"},
                                     {"reason", "move_best"}});
                return false;
            }
            tcpRet = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
            if (tcpRet != sucess) {
                APP_LOG_EVENT_ERROR("calibration.pose_search",
                                    "best_pose",
                                    "fail",
                                    {{"phase", "z_search"},
                                     {"reason", "tcp_read"},
                                     {"error", QString::number(tcpRet)}});
                return false;
            }
            std::cout << "[autoFindBestPose] Phase1 选定 Z=" << tcp_aa[2] << std::endl;
        }
        if (!moveToTcpAxisPos(robot_handle, tcp_aa)) {
            APP_LOG_EVENT_ERROR("calibration.pose_search",
                                "best_pose",
                                "fail",
                                {{"phase", "z_search"},
                                 {"reason", "final_move"}});
            return false;
        }
    } else {
        APP_LOG_EVENT_INFO("calibration.pose_search",
                           "phase_skip",
                           "skip",
                           {{"phase", "coarse_z"},
                            {"reason", "all_corners_visible"},
                            {"corners", QString::number(cur_codes)},
                            {"total_corners", QString::number(total_corners)}});
    }

    // ================================================================
    // Phase 2: XY 搜索 — 码不全时找更好的水平位置
    // ================================================================
    
    std::tie(cur_score, cur_codes, cur_bc) = evaluatePose(camera, detector);

    if (cur_codes < total_corners) {
        std::cout << "[autoFindBestPose] Phase2: XY 搜索 (码=" << cur_codes
                  << "/" << total_corners << ")" << std::endl;
        APP_LOG_EVENT_INFO("calibration.pose_search",
                           "coarse_search",
                           "start",
                           {{"phase", "coarse_xy"},
                            {"reason", cur_codes == 0
                                 ? "recover_board_not_detected" : "incomplete_corners"},
                            {"corners", QString::number(cur_codes)},
                            {"total_corners", QString::number(total_corners)}});
        const double steps[] = {240, 120, 60};
        if (!searchXYSteps(robot_handle, camera, detector,
                           steps, std::size(steps), tcp_aa)) {
            APP_LOG_EVENT_ERROR("calibration.pose_search",
                                "best_pose",
                                "fail",
                                {{"phase", "coarse_xy"},
                                 {"reason", "xy_search_failed"}});
            return false;
        }
    } else {
        APP_LOG_EVENT_INFO("calibration.pose_search",
                           "phase_skip",
                           "skip",
                           {{"phase", "coarse_xy"},
                            {"reason", "all_corners_visible"},
                            {"corners", QString::number(cur_codes)},
                            {"total_corners", QString::number(total_corners)}});
    }

    // 已有可信内参时，通过受限的小角度试探直接让相机光轴对准板面法向。
    // 没有内参的首轮 MVS bootstrap 会在函数内明确降级，不使用伪 PnP。
    if (!alignCameraPlaneToBoard(robot_handle, camera, detector,
                                 failure_message)) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "best_pose",
                            "fail",
                            {{"phase", "visual_plane_alignment"},
                             {"reason", "plane_alignment_failed"}});
        return false;
    }
    tcpRet = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
    if (tcpRet != sucess ||
        !convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos)) {
        if (failure_message)
            *failure_message = "视觉调正后读取机器人实际位姿失败";
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "best_pose",
                            "fail",
                            {{"phase", "visual_plane_alignment"},
                             {"reason", "actual_pose_refresh_failed"},
                             {"error", QString::number(tcpRet)}});
        return false;
    }
    frame = camera->captureOneFrame(1500);

    // ================================================================
    // Phase 3: Z+rz 拟合（不动 XY）
    // ================================================================    
    for (int iter = 0; iter < MAX_REFINE_ZRZ; iter++) {
        cv::Mat frame = camera->captureOneFrame(1500);
        if (frame.empty()) {
            if (failure_message)
                *failure_message = "自动寻找最佳位姿失败：相机采集不到图像";
            APP_LOG_EVENT_ERROR("calibration.pose_search",
                                "best_pose",
                                "fail",
                                {{"phase", "zrz"},
                                 {"reason", "empty_frame"},
                                 {"iter", QString::number(iter)}});
            std::cerr << "[autoFindBestPose] 采集图像失败" << std::endl;
            return false;
        }
        
        if (callback) {

            std::string img_name = save_dir + "/" +
                (iter < 10 ? "200" : iter < 100 ? "20" : "") +
                std::to_string(iter) + ".jpg";
            cv::imwrite(img_name, frame);            

            int call_ret = callback((char*)img_name.c_str(), (int)img_name.length(), 1);
            if (-1==call_ret)
            {
                std::cerr << "\r[autoCalib] 用户取消了  采集进度"  << std::endl;

                return false;
            }
            
        }     

        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detector.detectCorners(frame, corners, ids) || ids.size() < 4) {
            if (iter == 0) {
                if (failure_message) {
                    *failure_message =
                        "自动寻找最佳位姿失败：粗搜索后仍未检测到足够的标定板角点（" +
                        std::to_string(ids.size()) + "/" +
                        std::to_string(total_corners) + "）";
                }
                APP_LOG_EVENT_ERROR("calibration.pose_search",
                                    "best_pose",
                                    "fail",
                                    {{"phase", "zrz"},
                                     {"reason", "insufficient_corners_after_coarse_search"},
                                     {"iter", QString::number(iter)},
                                     {"corners", QString::number(ids.size())},
                                     {"minimum", "4"},
                                     {"total_corners", QString::number(total_corners)}});
                std::cerr << "[autoFindBestPose] 初始帧检测失败" << std::endl;
                return false;
            }
            std::cout << "[autoFindBestPose] 检测失败，保持当前位置" << std::endl;
            return true;
        }
        
        if (!convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos)) {
            APP_LOG_EVENT_ERROR("calibration.pose_search",
                                "best_pose",
                                "fail",
                                {{"phase", "zrz"},
                                 {"reason", "tcp_to_flange"},
                                 {"iter", QString::number(iter)}});
            return false;
        }
        //ret = cr_AxisAngle2Eule(tcp_aa, eulePos);
        // 利用像素面积比估算 Z 调整（area ∝ 1/Z²）
        std::vector<cv::Point2f> hull;
        cv::convexHull(corners, hull);
        double hull_area = cv::contourArea(hull);
        double board_area = hull_area * total_corners / std::max((int)corners.size(), 1);
        double area_ratio = board_area / (double)(frame.cols * frame.rows);
        double Z_desired = flangeEulePos[2] * std::sqrt(
            area_ratio / kTargetBoardAreaRatio);
        double dZ =  (Z_desired - flangeEulePos[2]); 
        double dRz_deg = -computeBoardAngle(corners, ids, board_sz.width, board_sz.height, frame.cols, frame.rows) * 180.0 / CV_PI;

        if (std::abs(dZ) < 30.0 && std::abs(dRz_deg) < 1.0) {
            std::cout << "[autoFindBestPose] Z+rz 收敛, iter=" << iter << std::endl;
            break;
        }

        // 在 0.2~1.0 比例下尝试 Z+rz 组合，选评分最高的
        double cur_s; int n; cv::Point2f bc;
        std::tie(cur_s, n, bc) = evaluatePose(camera, detector);

        std::vector<std::array<double, 6>> zrz_candidates;
        for (int fracStep = 1; fracStep <= 2; ++fracStep) {
            const double frac = fracStep * 0.5;
            // 保持 Rz 与当前位姿连续；强行限制到 [-90, 90] 会制造 180° 的 J6 跳变。
            zrz_candidates.push_back({flangeEulePos[0], flangeEulePos[1], flangeEulePos[2] + dZ * frac,
                                      flangeEulePos[3], flangeEulePos[4], flangeEulePos[5] + dRz_deg * frac});
        }

        double bestFlangeEulePos[ROB_AXIS_NUM];
        const double candidateScore = tryCandidatePoses(
            robot_handle, camera, detector, "refine_zrz",
            zrz_candidates, bestFlangeEulePos);
        if (!std::isfinite(candidateScore)) {
            APP_LOG_EVENT_ERROR("calibration.pose_search",
                                "best_pose",
                                "fail",
                                {{"phase", "zrz"},
                                 {"reason", "candidate_search_failed"},
                                 {"iter", QString::number(iter)}});
            return false;
        }
        if (candidateScore > cur_s) {
            if (!moveToFlangeEulePos(robot_handle, bestFlangeEulePos)) {
                APP_LOG_EVENT_ERROR("calibration.pose_search",
                                    "best_pose",
                                    "fail",
                                    {{"phase", "zrz"},
                                     {"reason", "move_best"},
                                     {"iter", QString::number(iter)}});
                return false;
            }
            tcpRet = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
            if (tcpRet != sucess ||
                !convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos)) {
                APP_LOG_EVENT_ERROR("calibration.pose_search",
                                    "best_pose",
                                    "fail",
                                    {{"phase", "zrz"},
                                     {"reason", "refresh_pose"},
                                     {"iter", QString::number(iter)},
                                     {"tcp_error", QString::number(tcpRet)}});
                return false;
            }
        } else {
            std::cout << "[autoFindBestPose] Z+rz 无改善, iter=" << iter << std::endl;
            break;
        }
    }
    if (!moveToTcpAxisPos(robot_handle, tcp_aa)) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "best_pose",
                            "fail",
                            {{"phase", "zrz"},
                             {"reason", "final_move"}});
        return false;
    }
    frame = camera->captureOneFrame(1500);
    // ================================================================
    // Phase 4: XY 精细搜索
    // ================================================================

    std::cout << "[autoFindBestPose] Phase4: XY 精细搜索..." << std::endl;
    const double steps_fine[] = {60, 30, 15};    
    if (!searchXYSteps(robot_handle, camera, detector,
                       steps_fine, std::size(steps_fine), tcp_aa)) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "best_pose",
                            "fail",
                            {{"phase", "fine_xy"},
                             {"reason", "xy_search_failed"}});
        return false;
    }

    // 运动到最终最佳位姿（tcp_aa 已包含最优平移+旋转）
    {
        if (!moveToTcpAxisPos(robot_handle, tcp_aa)) {
            APP_LOG_EVENT_ERROR("calibration.pose_search",
                                "best_pose",
                                "fail",
                                {{"phase", "final_position"},
                                 {"reason", "final_move"}});
            return false;
        }
        frame = camera->captureOneFrame(1500);       
    }

    if (!validateFinalObservationPose(camera, detector, failure_message)) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "best_pose",
                            "fail",
                            {{"phase", "observation_quality_gate"},
                             {"reason", "quality_gate_failed"}});
        return false;
    }

    APP_LOG_EVENT_INFO("calibration.pose_search",
                       "best_pose",
                       "success",
                       {{"status", "complete"}});
    return true;
}

// ============================================================
// 计算并验证 3 个候选 Z 值：迭代逼近目标边长比
// ============================================================

static bool computeZValues(int robot_handle,
    std::shared_ptr<AICamera>& camera,
    int cameraToTcpEnd,
    CharucoDetector& detector,
    double z_out_[3]) {
    double tcp_aa[ROB_AXIS_NUM] = {};
    double oriFlangeEulePos[ROB_AXIS_NUM] = {};
    //double orig_joint[ROB_AXIS_NUM];
    const CRresult tcpRet = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
    if (tcpRet != sucess) {
        APP_LOG_EVENT_ERROR("calibration.height",
                            "z_values",
                            "fail",
                            {{"reason", "tcp_read"},
                             {"error", QString::number(tcpRet)}});
        return false;
    }
    if (!convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, oriFlangeEulePos)) {
        APP_LOG_EVENT_ERROR("calibration.height",
                            "z_values",
                            "fail",
                            {{"reason", "tcp_to_flange_conversion"}});
        return false;
    }
    // 估计相机到标定板的距离
    cv::Mat frame;
    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
    double heightCamera = 0.0;
    if (!estimateCameraHeight(camera, detector, robot_handle, oriFlangeEulePos,
                               frame, corners, ids, heightCamera)) {
        return false;
    }

    // 标定板在基坐标系下的 Z 坐标
    double z_board = oriFlangeEulePos[2] - heightCamera;
    const double MIN_SAFE_HEIGHT = 300.0;   // 最低 30cm

    // 通过 IK 搜索最大可达高度（保持当前 rx=-180, ry=0 姿态）
    double euler[ROB_AXIS_NUM];
    double joint[ROB_AXIS_NUM];
    std::copy(oriFlangeEulePos, oriFlangeEulePos + ROB_AXIS_NUM, euler);
    double max_reach = MIN_SAFE_HEIGHT;
    for (int heightMm = 1400; heightMm > static_cast<int>(MIN_SAFE_HEIGHT); heightMm -= 10) {
        const double h = static_cast<double>(heightMm);
        euler[2] = z_board + h;
        if (convertFlangeOulePos2Joint(robot_handle, euler, joint)) {
            max_reach = h;
            break;
        }
    }

    // 最大有效高度 = 80% 可达（留余量）
    double z_high_h = std::min(max_reach * 0.9, max_reach - 100.0);
    double z_low_h  = std::max(MIN_SAFE_HEIGHT, z_high_h - 600.0);

    // 保证至少 500mm 范围
    if (z_high_h - z_low_h < 500.0)
        z_low_h = std::max(MIN_SAFE_HEIGHT, z_high_h - 500.0);

    double z_mid_h = (z_low_h + z_high_h) / 2.0;

    z_out_[2] = z_board + z_low_h;
    z_out_[1] = z_board + z_mid_h;
    z_out_[0] = z_board + z_high_h;

    std::cout << "[autoCalib] Z 值: low=" << z_out_[2]
              << ", mid=" << z_out_[1]
              << ", high=" << z_out_[0]
              << " mm (范围 " << (z_high_h - z_low_h) << " mm)"
              << ", max_reach=" << max_reach << " mm" << std::endl;
    return true;
}

// ============================================================
// Phase 3: 生成约 360 个高覆盖候选 + IK 过滤
// ============================================================

bool generateCandidates(int robot_handle,
                        std::shared_ptr<AICamera>& camera,
                        const double z_values[3],
                        std::vector<PoseCandidate>& candidates,
                        int* planned_count = nullptr) {
    double euler[ROB_AXIS_NUM] = {};
    double tcp_aa[ROB_AXIS_NUM] = {};
    double flangeEulePos[ROB_AXIS_NUM] = {};
    double joint[ROB_AXIS_NUM] = {};
    double axisAngle[ROB_AXIS_NUM] = {};
    const CRresult tcpRet = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
    if (tcpRet != sucess) {
        APP_LOG_EVENT_ERROR("calibration.candidates",
                            "candidate_generation",
                            "fail",
                            {{"reason", "tcp_read"},
                             {"error", QString::number(tcpRet)}});
        return false;
    }
    if (!convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos)) {
        APP_LOG_EVENT_ERROR("calibration.candidates",
                            "candidate_generation",
                            "fail",
                            {{"reason", "tcp_to_flange_conversion"}});
        return false;
    }
    (void)camera;
    double initialJoint[ROB_AXIS_NUM] = {};
    const CRresult jointRet = cr_get_jointActualPos((RobotHandle)robot_handle, initialJoint);
    if (jointRet != sucess) {
        APP_LOG_EVENT_ERROR("calibration.candidates",
                            "candidate_generation",
                            "fail",
                            {{"reason", "joint_read"},
                             {"error", QString::number(jointRet)}});
        return false;
    }
    int total = 0, ik_ok = 0;

    // 每层 5 个 XY 覆盖点 × 24 个姿态，共 3 × 5 × 24 = 360 个候选。
    // 距离越远，标定板在画面中的边界余量越大，允许更大的 X/Y 倾角；
    // 近距离层使用较小倾角保证标定板完整可见；同一 XY 点增加对称组合姿态，
    // 用更高采样冗余换取批次稳定性。
    const std::array<double, 3> tiltByDistance{{18.0, 14.0, 10.0}};
    for (int zi = 0; zi < 3; ++zi) {
        const double z = z_values[zi];
        const double dlt = 360.0 * (3 - zi) / 3.0;
        const double tilt = tiltByDistance[zi];
        const std::array<std::array<double, 2>, 5> xyOffsets{{
            {{0.0, 0.0}}, {{-dlt, -dlt}}, {{-dlt, dlt}},
            {{dlt, -dlt}}, {{dlt, dlt}}
        }};

        for (size_t xyIndex = 0; xyIndex < xyOffsets.size(); ++xyIndex) {
            const double rz = ((zi + static_cast<int>(xyIndex)) % 2 == 0) ? 15.0 : -15.0;
            const std::array<std::array<double, 3>, 24> attitudeOffsets{{
                {{0.0, 0.0, 0.0}},
                {{tilt, 0.0, rz}},
                {{-tilt, 0.0, -rz}},
                {{0.0, tilt, -rz}},
                {{0.0, -tilt, rz}},
                {{tilt * 0.7, -tilt * 0.7, rz}},
                {{-tilt * 0.7, tilt * 0.7, -rz}},
                {{tilt * 0.7, tilt * 0.7, -rz}},
                {{-tilt * 0.7, -tilt * 0.7, rz}},
                {{tilt * 0.5, tilt * 0.5, 0.0}},
                {{-tilt * 0.5, -tilt * 0.5, 0.0}},
                {{tilt * 0.5, -tilt * 0.5, 0.0}},
                {{-tilt * 0.5, tilt * 0.5, 0.0}},
                {{tilt, tilt * 0.35, 0.0}},
                {{-tilt, -tilt * 0.35, 0.0}},
                {{tilt * 0.35, tilt, 0.0}},
                {{-tilt * 0.35, -tilt, 0.0}},
                {{0.0, 0.0, rz}},
                {{0.0, 0.0, -rz}},
                {{0.0, 0.0, 2.0 * rz}},
                {{0.0, 0.0, -2.0 * rz}},
                {{tilt * 0.75, 0.0, -rz}},
                {{0.0, tilt * 0.75, rz}},
                {{-tilt * 0.6, -tilt * 0.6, -rz}}
            }};

            for (const auto& attitude : attitudeOffsets) {
                ++total;
                euler[0] = flangeEulePos[0] + xyOffsets[xyIndex][0];
                euler[1] = flangeEulePos[1] + xyOffsets[xyIndex][1];
                euler[2] = z;
                euler[3] = flangeEulePos[3] + attitude[0];
                euler[4] = flangeEulePos[4] + attitude[1];
                euler[5] = flangeEulePos[5] + attitude[2];

                const int sourceIndex = total - 1;
                const CRresult eulerRet = cr_Eule2AxisAngle(euler, axisAngle);
                const CRresult ikRet = eulerRet == sucess
                    ? cr_kineInverse((RobotHandle)robot_handle, axisAngle,
                                     initialJoint, joint)
                    : eulerRet;
                if (ikRet == sucess) {
                    PoseCandidate candidate{};
                    std::copy(joint, joint + ROB_AXIS_NUM, candidate.joint);
                    std::copy(axisAngle, axisAngle + ROB_AXIS_NUM,
                              candidate.axis_angle);
                    std::copy(euler, euler + ROB_AXIS_NUM,
                              candidate.flange_euler);
                    candidate.source_index = sourceIndex;
                    candidates.push_back(candidate);
                    ++ik_ok;
                }
            }
        }
    }

    CandidatePlanningOptions planningOptions;
    // 全自动候选网格不做数量压缩：规划上限与当前 IK 可达候选数一致。
    // 这样 maximum_candidates 不会把 360 个原始候选再次裁剪成小集合。
    planningOptions.maximum_candidates = candidates.size();
    planningOptions.joint_resolver = [robot_handle](
        const PoseCandidate& candidate,
        const std::array<double, 6>& reference,
        std::array<double, 6>& resolved,
        int& error_code) {
        double target[ROB_AXIS_NUM]{};
        double referenceJoint[ROB_AXIS_NUM]{};
        std::copy(candidate.axis_angle,
                  candidate.axis_angle + ROB_AXIS_NUM, target);
        std::copy(reference.begin(), reference.end(), referenceJoint);
        const CRresult result = cr_kineInverse((RobotHandle)robot_handle,
                                               target, referenceJoint,
                                               resolved.data());
        error_code = static_cast<int>(result);
        return result == sucess;
    };
    CandidatePlanningReport planningReport;
    std::string planningError;
    if (!planPoseCandidates(candidates, initialJoint, planningOptions,
                            planningReport, planningError)) {
        APP_LOG_EVENT_ERROR("calibration.candidates",
                            "candidate_ordering",
                            "fail",
                            {{"strategy", "nearest_joint_greedy"},
                             {"reason", QString::fromStdString(planningError)}});
        return false;
    }
    std::cout << "[autoCalib] 360 candidates generated, IK reachable: "
              << ik_ok << "/" << total << std::endl;
    if (planned_count)
        *planned_count = total;
    APP_LOG_EVENT_INFO("calibration.candidates",
                       "candidate_generation",
                       "success",
                       {{"strategy", "coverage_3x5x24"},
                        {"target", "360"},
                        {"total", QString::number(total)},
                        {"reachable", QString::number(ik_ok)},
                        {"maximum_candidates", QString::number(
                            planningOptions.maximum_candidates)},
                        {"ordered", QString::number(candidates.size())},
                        {"transitions", QString::number(
                            planningReport.transition_candidates)},
                        {"connected", QString::number(
                            planningReport.connected_candidates)},
                        {"disconnected", QString::number(
                            planningReport.disconnected_candidates)},
                        {"estimated_path_deg", QString::number(
                            planningReport.estimated_path_degrees, 'f', 3)},
                        {"max_step_deg", QString::number(
                            planningReport.maximum_joint_step_degrees, 'f', 3)},
                        {"max_j6_step_deg", QString::number(
                            planningReport.maximum_j6_step_degrees, 'f', 3)},
                        {"tilt_far_deg", QString::number(tiltByDistance[0], 'f', 1)},
                        {"tilt_mid_deg", QString::number(tiltByDistance[1], 'f', 1)},
                        {"tilt_near_deg", QString::number(tiltByDistance[2], 'f', 1)},
                        {"rz_deg", "15.0"}});
    return !candidates.empty();
}
} // anonymous namespace

// ============================================================
// 公开 API
// ============================================================

bool autoEyeInHandCharucoCalibration(
    int robot_handle,
    char* pchRobotIP,
    std::shared_ptr<AICamera>& camera,
    const CharucoBoardConfig& board_config,
    cv::InputOutputArray camera_matrix,
    cv::InputOutputArray dist_coeffs,
    int cameraToTcpEnd,
    CalibrationResult& result,
    const std::string& save_dir,
    const std::string& output_yaml,
    AI_det_callback callback,
    const std::string& intrinsic_source_file,
    CalibrationRuntimeCallback runtime_callback) {

    result = CalibrationResult{};
    CalibrationRuntimeStatus runtime_status = makeEmptyCalibrationRuntimeStatus();
    auto publish = [&](CalibrationRuntimeStage stage,
                       const std::string& detail = {}) {
        publishCalibrationRuntime(runtime_callback, runtime_status, stage, detail);
    };
    CalibrationSessionOptions session_options;
    session_options.base_directory = save_dir;
    session_options.mode = "online_auto_eye_in_hand";
    session_options.source_data_directory = "live_capture";
    session_options.board = board_config;
    session_options.robot_endpoint = pchRobotIP ? pchRobotIP : "";
    if (camera) {
        const CONFIG& camera_config = camera->getConfig();
        const CameraDeviceAuditInfo device_info = camera->getDeviceAuditInfo();
        session_options.camera_backend = camera->backendName();
        session_options.camera_serial = device_info.serial;
        session_options.camera_model = device_info.model;
        session_options.camera_firmware = device_info.firmware;
        session_options.camera_transport = device_info.transport;
        session_options.camera_endpoint = !device_info.current_ip.empty()
            ? device_info.current_ip
            : camera_config.cam_ip;
        // Session 记录打开后回读的实际流分辨率，而不是请求值
        session_options.image_width = device_info.actual_width > 0
            ? device_info.actual_width : camera_config.cam_width;
        session_options.image_height = device_info.actual_height > 0
            ? device_info.actual_height : camera_config.cam_height;
        session_options.camera_pixel_format = device_info.pixel_format;
        session_options.camera_exposure_us = device_info.exposure_us;
        session_options.camera_gain = device_info.gain;
        session_options.camera_trigger_mode = device_info.trigger_mode;
        session_options.camera_sdk_version = device_info.sdk_version;
    }
    bool use_provided_intrinsics =
        !camera_matrix.empty() && !dist_coeffs.empty();
    session_options.intrinsic_source = use_provided_intrinsics
        ? "provided_runtime" : "calibrated_from_collection";
    session_options.intrinsic_source_file = intrinsic_source_file;
    if (use_provided_intrinsics) {
        session_options.camera_matrix = camera_matrix.getMat().clone();
        session_options.dist_coeffs = dist_coeffs.getMat().clone();
    }    std::string session_error;
    publish(CalibrationRuntimeStage::Preparing, "create_session");
    std::unique_ptr<CalibrationSession> session =
        CalibrationSession::create(session_options, session_error);
    if (!session) {
        result.message = "无法创建标定审计会话: " + session_error;
        publish(CalibrationRuntimeStage::Failed, "session_create_failed");
        return false;
    }
    result.session_dir = session->directory();
    runtime_status.calibration_session_id = session->id();
    runtime_status.calibration_session_dir = session->directory();
    runtime_status.publication_state = "not_started";
    auto failSession = [&](const std::string& stage,
                           const std::string& reason) -> bool {
        const CalibrationRuntimeStage terminal =
            (reason == "cancelled")
                ? CalibrationRuntimeStage::Cancelled
                : CalibrationRuntimeStage::Failed;
        publish(terminal, reason);
        session->finalizeFailure(stage, reason, result.message);
        return false;
    };

    // ---- MVS 内参可信度策略（在任何机器人运动之前） ----
    // 已有且身份匹配的 K/D 直接复用；缺失、未绑定或分辨率变化时，
    // 进入采集后内参标定，并把新参数绑定到当前相机。只有明确绑定到
    // 另一台相机的文件才阻断，避免误覆盖其他设备的生产参数。
    std::string mvsIntrinsicSource;
    if (camera && camera->backendName() == "mvs") {
        const CameraDeviceAuditInfo gateInfo = camera->getDeviceAuditInfo();
        const CONFIG& gateCfg = camera->getConfig();
        intrinsic_gate::IntrinsicIdentity identity;
        std::string decisionReason;
        const intrinsic_gate::MvsIntrinsicDecision decision =
            intrinsic_gate::decideMvsIntrinsics(
                intrinsic_source_file,
                use_provided_intrinsics ? camera_matrix.getMat() : cv::Mat(),
                use_provided_intrinsics ? dist_coeffs.getMat() : cv::Mat(),
                gateInfo.serial, gateInfo.actual_width, gateInfo.actual_height,
                gateCfg.allow_legacy_intrinsics, identity, decisionReason);
        if (decision == intrinsic_gate::MvsIntrinsicDecision::Block) {
            result.message = "MVS 内参身份校验失败: " + decisionReason;
            APP_LOG_EVENT_ERROR("calibration",
                                "intrinsic_gate",
                                "blocked",
                                {{"reason", "mvs_intrinsic_identity_mismatch"},
                                 {"device_serial", QString::fromStdString(gateInfo.serial)},
                                 {"intrinsic_serial", QString::fromStdString(identity.serial)},
                                 {"actual_resolution", QString("%1x%2").arg(gateInfo.actual_width).arg(gateInfo.actual_height)},
                                 {"intrinsic_resolution", QString("%1x%2").arg(identity.width).arg(identity.height)}});
            return failSession("intrinsic_validation",
                               "mvs_intrinsic_identity_mismatch");
        }
        if (decision ==
            intrinsic_gate::MvsIntrinsicDecision::BootstrapFromCollection) {
            use_provided_intrinsics = false;
            mvsIntrinsicSource = "calibrated_from_collection";
            session->setIntrinsics(cv::Mat(), cv::Mat(),
                                   "calibrated_from_collection_pending");
            APP_LOG_EVENT_WARNING("calibration",
                                  "intrinsic_gate",
                                  "bootstrap_planned",
                                  {{"file", QString::fromStdString(intrinsic_source_file)},
                                   {"device_serial", QString::fromStdString(gateInfo.serial)},
                                   {"actual_resolution", QString("%1x%2").arg(gateInfo.actual_width).arg(gateInfo.actual_height)},
                                   {"reason", QString::fromStdString(decisionReason)}});
        } else {
            mvsIntrinsicSource = identity.hasAny()
                ? "external_file_validated" : "external_file_legacy_unbound";
            session->setIntrinsics(camera_matrix.getMat(), dist_coeffs.getMat(),
                                   mvsIntrinsicSource);
            APP_LOG_EVENT_INFO("calibration",
                               "intrinsic_gate",
                               "passed",
                               {{"source", QString::fromStdString(mvsIntrinsicSource)},
                                {"device_serial", QString::fromStdString(gateInfo.serial)},
                                {"intrinsic_serial", QString::fromStdString(identity.serial)},
                                {"actual_resolution", QString("%1x%2").arg(gateInfo.actual_width).arg(gateInfo.actual_height)},
                                {"detail", QString::fromStdString(decisionReason)}});
        }
    }

    g_save_dir = session->previewDirectory();
    g_callback = callback;

    std::cout << "\n========== 自动手眼标定开始 ==========" << std::endl;

    CharucoDetector detector(board_config);
    if (use_provided_intrinsics) {
        detector.setCameraParams(camera_matrix.getMat(), dist_coeffs.getMat());
        APP_LOG_EVENT_INFO("calibration",
                           "camera_intrinsics_source",
                           "selected",
                           {{"source", "provided"},
                            {"fx", QString::number(camera_matrix.getMat().at<double>(0, 0), 'f', 6)},
                            {"fy", QString::number(camera_matrix.getMat().at<double>(1, 1), 'f', 6)},
                            {"cx", QString::number(camera_matrix.getMat().at<double>(0, 2), 'f', 6)},
                            {"cy", QString::number(camera_matrix.getMat().at<double>(1, 2), 'f', 6)},
                            {"distortion_count", QString::number(dist_coeffs.getMat().total())}});
    }

    // ---- Phase 1b: 自动寻找最佳位姿 ----
    session->setStage("prepare");
    publish(CalibrationRuntimeStage::Preparing, "prepare");
    std::cout << "[autoCalib] 自动寻找最佳位姿..." << std::endl;
    publish(CalibrationRuntimeStage::PoseSearch, "pose_search");
    std::string best_pose_failure;
    if (!autoFindBestPose(robot_handle, camera, detector,
                            session->previewDirectory(), callback, &best_pose_failure)) {
        if (robotMotionCancellationRequested()) {
            result.message = "标定已取消";
        } else if (!best_pose_failure.empty()) {
            result.message = best_pose_failure;
        } else {
            result.message = "自动寻找最佳位姿失败，请查看诊断日志确认运动、相机或检测原因";
        }
        std::cerr << "[autoCalib] 自动寻找最佳位姿失败" << std::endl;
        return failSession("pose_search", robotMotionCancellationRequested()
            ? "cancelled" : "best_pose_search_failed");
    }

    ObservationReferencePose observationPose;
    std::string observationPoseError;
    if (!captureObservationReferencePose(robot_handle, observationPose,
                                         observationPoseError)) {
        result.message = "保存水平观察位姿前读取机器人反馈失败: " +
            observationPoseError;
        session->appendEvent("observation_reference_pose", "fail",
                             observationPoseError);
        return failSession("pose_search",
                           "observation_reference_pose_capture_failed");
    }
    const std::filesystem::path observationPosePath =
        observationReferencePosePath(output_yaml, intrinsic_source_file);
    if (!observationPosePath.empty()) {
        if (!saveObservationReferencePose(observationPosePath,
                                          observationPose,
                                          observationPoseError)) {
            result.message = "水平观察位姿保存失败: " + observationPoseError;
            session->appendEvent("observation_reference_pose", "fail",
                                 observationPoseError);
            return failSession("pose_search",
                               "observation_reference_pose_persist_failed");
        }
        session->appendEvent("observation_reference_pose", "published",
                             observationPosePath.string());
    } else {
        session->appendEvent("observation_reference_pose", "skip",
                             "production_parameter_path_unavailable");
        APP_LOG_EVENT_WARNING("calibration.pose_search",
                              "observation_reference_pose",
                              "skip",
                              {{"reason", "production_parameter_path_unavailable"}});
    }

    // ---- Phase 2: 计算动态 Z 值 ----
    double z_values[3] = {0.0, 0.0, 0.0};
    {
        if (computeZValues(robot_handle, camera, cameraToTcpEnd, detector, z_values)) {
            std::cout << "[autoCalib] 动态 Z 值: "
                      << z_values[0] << ", " << z_values[1] << ", " << z_values[2] << std::endl;
        } else {
            // generateCandidates() 需要绝对 Z 坐标。高度估计失败后无法可靠
            // 判断标定板高度和安全工作空间，不能把未初始化值或相对偏移量
            // 当作绝对坐标继续传给机器人 SDK。
            result.message = "Z 值计算失败，已停止标定以避免不安全运动";
            std::cerr << "[autoCalib] " << result.message << std::endl;
            return failSession("pose_generation", "z_value_estimation_failed");
        }
    }

    // ---- Phase 3: 生成候选 + IK 过滤 ----
    publish(CalibrationRuntimeStage::CandidateGeneration, "candidate_generation");
    std::vector<PoseCandidate> candidates;
    int planned_count = 0;
    if (!generateCandidates(robot_handle, camera, z_values, candidates, &planned_count)) {
        result.message = "无 IK 可达位姿";
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return failSession("pose_generation", "no_reachable_pose");
    }
    runtime_status.candidate_planned = planned_count;
    runtime_status.candidate_reachable = static_cast<int>(candidates.size());
    runtime_status.candidate_processed = 0;
    publish(CalibrationRuntimeStage::CandidateGeneration, "candidates_ready");

    // ---- Phase 4: 逐点运动 + 采集 ----
    session->setStage("collection");
    if (!executeCollection(robot_handle, camera, detector,
                            candidates, session->framesDirectory(), callback,
                            session.get(), session->robotPosesDirectory(),
                            &runtime_status, runtime_callback)) {
        result.message = "未采集到有效数据";
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return failSession("collection", robotMotionCancellationRequested()
            ? "cancelled" : "collection_failed");
    }

    // ---- Phase 5: 标定（直接从磁盘读取） ----
    cv::Mat K, D;
    if (!use_provided_intrinsics) {
        std::cout << "[autoCalib] 相机内参未知，从磁盘标定..." << std::endl;
        if (!calibrateCameraFromDisk(session->framesDirectory(), detector, K, D)) {
            result.message = "相机内参标定失败";
            std::cerr << "[autoCalib] " << result.message << std::endl;
            return failSession("intrinsic_calibration", "intrinsic_calibration_failed");
        }
        APP_LOG_EVENT_INFO("calibration",
                           "camera_intrinsics_source",
                           "selected",
                           {{"source", "calibrated_from_collection"},
                            {"fx", QString::number(K.at<double>(0, 0), 'f', 6)},
                            {"fy", QString::number(K.at<double>(1, 1), 'f', 6)},
                            {"cx", QString::number(K.at<double>(0, 2), 'f', 6)},
                            {"cy", QString::number(K.at<double>(1, 2), 'f', 6)},
                            {"distortion_count", QString::number(D.total())}});
    } else {
        camera_matrix.getMat().copyTo(K);
        dist_coeffs.getMat().copyTo(D);
        std::cout << "[autoCalib] 使用已有相机内参" << std::endl;
    }

    if (camera && camera->backendName() == "mvs" &&
        !use_provided_intrinsics) {
        const CameraDeviceAuditInfo deviceInfo = camera->getDeviceAuditInfo();
        const CONFIG& cameraConfig = camera->getConfig();
        intrinsic_gate::IntrinsicIdentity identity;
        identity.serial = deviceInfo.serial;
        identity.width = deviceInfo.actual_width > 0
            ? deviceInfo.actual_width : cameraConfig.cam_width;
        identity.height = deviceInfo.actual_height > 0
            ? deviceInfo.actual_height : cameraConfig.cam_height;
        identity.pixel_format = deviceInfo.pixel_format;
        std::string publicationError;
        bool published = false;
        if (intrinsic_source_file.empty()) {
            publicationError = "未指定 camera_params.yaml 输出路径";
        } else {
            published = intrinsic_gate::saveCameraParamsWithIdentity(
                intrinsic_source_file, K, D, identity,
                "calibrated_from_collection", publicationError);
        }
        if (!published) {
            result.message = "MVS 内参计算成功但发布失败: " + publicationError;
            APP_LOG_EVENT_ERROR(
                "calibration", "camera_intrinsics", "publish_failed",
                {{"file", QString::fromStdString(intrinsic_source_file)},
                 {"device_serial", QString::fromStdString(identity.serial)},
                 {"resolution", QString("%1x%2").arg(identity.width).arg(identity.height)},
                 {"error", QString::fromStdString(publicationError)}});
            return failSession("intrinsic_publication",
                               "mvs_intrinsic_publication_failed");
        }
        APP_LOG_EVENT_INFO(
            "calibration", "camera_intrinsics", "published",
            {{"file", QString::fromStdString(intrinsic_source_file)},
             {"source", "calibrated_from_collection"},
             {"device_serial", QString::fromStdString(identity.serial)},
             {"resolution", QString("%1x%2").arg(identity.width).arg(identity.height)},
             {"pixel_format", QString::fromStdString(identity.pixel_format)}});
    }
    session->setIntrinsics(K, D, !mvsIntrinsicSource.empty()
        ? mvsIntrinsicSource
        : (use_provided_intrinsics
            ? "provided_runtime" : "calibrated_from_collection"));

    if (!calibrateHandEyeFromDisk(session->framesDirectory(), K, D, board_config,
                                  result, output_yaml, session.get(),
                                  intrinsic_source_file, false,
                                  &runtime_status, runtime_callback))
        return false;

    K.copyTo(camera_matrix);
    D.copyTo(dist_coeffs);

    std::cout << "========== 自动手眼标定完成 ==========\n" << std::endl;
    return true;
}
