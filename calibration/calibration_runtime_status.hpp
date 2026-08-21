#ifndef CALIBRATION_RUNTIME_STATUS_HPP
#define CALIBRATION_RUNTIME_STATUS_HPP

#include <cmath>
#include <functional>
#include <limits>
#include <string>

/**
 * @brief 标定运行时结构化状态（纯 C++，不依赖 Qt）
 *
 * 供 Calibration Core → CalibrationController → QML 实时状态链使用。
 * 与 DiagnosticLogger 日志字符串完全解耦。
 * 未产生的 double 指标使用 NaN；未产生的整数计数使用 -1（禁止用 0 冒充真实值）。
 */
enum class CalibrationRuntimeStage {
    Idle,
    Preparing,
    PoseSearch,
    CandidateGeneration,
    Collecting,
    PnP,
    HandEyeInitial,
    MadFiltering,
    HandEyeRefit,
    QualityValidation,
    Publishing,
    Completed,
    Failed,
    Cancelled
};

struct CalibrationRuntimeStatus {
    CalibrationRuntimeStage stage = CalibrationRuntimeStage::Idle;

    // candidate（未产生时为 -1）
    int candidate_planned = -1;
    int candidate_reachable = -1;
    int candidate_processed = -1;

    // collection（未产生时为 -1）
    int captured_frames = -1;
    int skipped_frames = -1;
    int current_candidate_index = -1;
    int current_corner_count = -1;

    // solve（未产生时为 -1）
    int pnp_valid_frames = -1;
    int mad_rejected_frames = -1;
    int final_inliers = -1;

    // final quality（未产生时为 NaN）
    double translation_rms_mm = std::numeric_limits<double>::quiet_NaN();
    double translation_p95_mm = std::numeric_limits<double>::quiet_NaN();
    double translation_max_mm = std::numeric_limits<double>::quiet_NaN();

    double rotation_rms_deg = std::numeric_limits<double>::quiet_NaN();
    double rotation_p95_deg = std::numeric_limits<double>::quiet_NaN();
    double rotation_max_deg = std::numeric_limits<double>::quiet_NaN();

    // solver consensus
    std::string selected_solver;
    bool consensus_available = false;
    bool consensus_passed = false;
    double consensus_max_translation_mm = std::numeric_limits<double>::quiet_NaN();
    double consensus_max_rotation_deg = std::numeric_limits<double>::quiet_NaN();

    // session / publication
    // publication_state: not_started | publishing | published | not_published | failed
    std::string calibration_session_id;
    std::string calibration_session_dir;
    std::string publication_state = "not_started";

    std::string detail;
};

inline CalibrationRuntimeStatus makeEmptyCalibrationRuntimeStatus()
{
    return CalibrationRuntimeStatus{};
}

using CalibrationRuntimeCallback =
    std::function<void(const CalibrationRuntimeStatus&)>;

inline void publishCalibrationRuntime(
    const CalibrationRuntimeCallback& callback,
    CalibrationRuntimeStatus& status,
    CalibrationRuntimeStage stage,
    const std::string& detail = {})
{
    status.stage = stage;
    if (!detail.empty())
        status.detail = detail;
    if (callback)
        callback(status);
}

#endif // CALIBRATION_RUNTIME_STATUS_HPP
