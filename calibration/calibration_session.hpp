#ifndef CALIBRATION_SESSION_HPP
#define CALIBRATION_SESSION_HPP

#include <opencv2/core.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "charuco_detector.hpp"

struct CalibrationFrameAudit {
    int frame_id = -1;
    int source_index = -1;
    std::string image_file;
    std::string image_sha256;
    std::uint64_t image_timestamp_ns = 0;
    std::uint64_t robot_timestamp_ns = 0;
    int image_width = 0;
    int image_height = 0;
    int corner_count = 0;
    double coverage = 0.0;
    double reprojection_rms_px = 0.0;
    double reprojection_p95_px = 0.0;
    double consistency_error_mm = 0.0;
    bool image_read_pass = false;
    bool charuco_pass = false;
    bool pnp_pass = false;
    bool quality_gate_pass = false;
    bool mad_filter_pass = false;
    bool final_accepted = false;
    std::string reject_stage;
    std::vector<std::string> reject_reasons;
    cv::Mat T_base_flange;
    cv::Mat T_camera_board;
};

struct HandEyeSolverAuditRecord {
    std::string phase;
    std::string method;
    bool success = false;
    std::string failure_reason;
    cv::Mat raw_transform;
    double raw_translation_rms_mm = 0.0;
    double raw_translation_p95_mm = 0.0;
    double raw_rotation_rms_deg = 0.0;
    double raw_rotation_p95_deg = 0.0;
    bool selected_medoid = false;
    double medoid_normalized_distance_sum = 0.0;
};

struct HandEyeSolverPairAuditRecord {
    std::string phase;
    std::string left_method;
    std::string right_method;
    double delta_translation_mm = 0.0;
    double delta_rotation_deg = 0.0;
};

struct HandEyeConsensusAuditRecord {
    std::string phase;
    int successful_solvers = 0;
    std::string medoid_method;
    double max_pair_translation_mm = 0.0;
    double max_pair_rotation_deg = 0.0;
    double translation_limit_mm = 0.0;
    double rotation_limit_deg = 0.0;
    bool passed = false;
    std::string failure_reason;
};

struct HandEyeOptimizationAuditRecord {
    bool performed = false;
    std::string phase;
    std::string method;
    cv::Mat initial_transform;
    cv::Mat optimized_transform;
    double delta_translation_mm = 0.0;
    double delta_rotation_deg = 0.0;
    double before_translation_rms_mm = 0.0;
    double after_translation_rms_mm = 0.0;
    double before_rotation_rms_deg = 0.0;
    double after_rotation_rms_deg = 0.0;
};

struct HandEyePhase2Audit {
    std::vector<HandEyeSolverAuditRecord> solvers;
    std::vector<HandEyeSolverPairAuditRecord> pairs;
    std::vector<HandEyeConsensusAuditRecord> consensus;
    std::vector<HandEyeOptimizationAuditRecord> optimizations;
};

struct HandEyeLooTrialAuditRecord {
    std::string mode;
    int removed_frame_id = -1;
    int removed_source_index = -1;
    bool success = false;
    std::string failure_reason;
    std::string selected_method;
    int final_inliers = 0;
    double delta_translation_mm = 0.0;
    double delta_rotation_deg = 0.0;
    bool filter_set_evaluated = false;
    std::vector<int> filter_set_added_source_indices;
    std::vector<int> filter_set_removed_source_indices;
};

struct HandEyeLooSummaryAuditRecord {
    std::string mode;
    int attempted = 0;
    int successful = 0;
    int failed = 0;
    double translation_mean_mm = 0.0;
    double translation_p95_mm = 0.0;
    double translation_max_mm = 0.0;
    int translation_max_frame_id = -1;
    int translation_max_source_index = -1;
    double rotation_mean_deg = 0.0;
    double rotation_p95_deg = 0.0;
    double rotation_max_deg = 0.0;
    int rotation_max_frame_id = -1;
    int rotation_max_source_index = -1;
    int trials_with_filter_set_changes = 0;
};

struct HandEyePhase3Audit {
    bool enabled = false;
    bool completed = false;
    std::string status = "not_run";
    std::string failure_reason;
    std::string full_selected_method;
    int worker_count = 0;
    cv::Mat full_transform;
    std::vector<int> full_inlier_source_indices;
    std::vector<HandEyeLooTrialAuditRecord> trials;
    std::vector<HandEyeLooSummaryAuditRecord> summaries;
};

struct CalibrationSessionOptions {
    std::string base_directory;
    std::string mode;
    std::string source_data_directory;
    std::string camera_serial;
    std::string camera_model;
    std::string camera_firmware;
    std::string camera_endpoint;
    std::string camera_stream = "color";
    int image_width = 0;
    int image_height = 0;
    std::string intrinsic_source;
    std::string intrinsic_source_file;
    std::string robot_endpoint;
    std::string robot_model;
    std::string robot_controller;
    CharucoBoardConfig board;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::string source_session_id;
    std::string source_manifest_sha256;
};

class CalibrationSession final {
public:
    static std::unique_ptr<CalibrationSession> create(
        const CalibrationSessionOptions& options,
        std::string& error);

    ~CalibrationSession();

    CalibrationSession(const CalibrationSession&) = delete;
    CalibrationSession& operator=(const CalibrationSession&) = delete;

    const std::string& id() const { return id_; }
    const std::string& directory() const { return directory_; }
    const std::string& framesDirectory() const { return frames_directory_; }
    const std::string& robotPosesDirectory() const { return robot_poses_directory_; }
    const std::string& previewDirectory() const { return preview_directory_; }

    void setStage(const std::string& stage);
    void setIntrinsics(const cv::Mat& camera_matrix,
                       const cv::Mat& dist_coeffs,
                       const std::string& source);
    void setImageSize(int width, int height);
    void setFrames(const std::vector<CalibrationFrameAudit>& frames);
    bool setHandEyePhase2Audit(const HandEyePhase2Audit& audit,
                               std::string& error);
    bool setHandEyePhase3Audit(const HandEyePhase3Audit& audit,
                               std::string& error);
    void appendEvent(const std::string& event,
                     const std::string& result,
                     const std::string& detail = "");

    bool importLegacyDataset(const std::string& source_directory,
                             std::string& error);
    bool writeRawRobotPose(int frame_id,
                           int candidate_index,
                           const double actual_joint[6],
                           const double flange_pose[6],
                           std::uint64_t image_timestamp_ns,
                           std::uint64_t robot_timestamp_ns,
                           std::string& error);

    bool finalizeSuccess(const cv::Mat& transform,
                         const cv::Mat& camera_matrix,
                         const cv::Mat& dist_coeffs,
                         double translation_rms_mm,
                         int valid_frames,
                         const std::string& message,
                         const std::string& production_yaml,
                         std::string& error);
    bool finalizeFailure(const std::string& stage,
                         const std::string& reason,
                         const std::string& message = "");

    static std::string sha256File(const std::string& path);
    static bool verifyManifest(const std::string& session_directory,
                               std::vector<std::string>& problems);
    static bool recoverPendingSessions(const std::string& base_directory,
                                       std::vector<std::string>& recovered_sessions,
                                       std::string& error);

private:
    explicit CalibrationSession(CalibrationSessionOptions options);

    bool initialize(std::string& error);
    bool writeSessionYaml(const std::string& lifecycle_status,
                          const std::string& result_status,
                          std::string& error);
    bool writeEffectiveConfig(std::string& error);
    bool writeReport(const std::string& status,
                     const std::string& failure_stage,
                     const std::string& failure_reason,
                     const cv::Mat& transform,
                     double translation_rms_mm,
                     int valid_frames,
                     const std::string& message,
                     std::string& error);
    bool writeManifest(std::string& error);
    bool appendDurableEvent(const std::string& event,
                            const std::string& result,
                            const std::string& detail,
                            std::string& error);
    void markInterruptedNoThrow();

    CalibrationSessionOptions options_;
    std::string id_;
    std::string timestamp_utc_;
    std::string directory_;
    std::string frames_directory_;
    std::string robot_poses_directory_;
    std::string preview_directory_;
    std::string current_stage_ = "created";
    std::vector<CalibrationFrameAudit> frames_;
    HandEyePhase2Audit hand_eye_phase2_audit_;
    HandEyePhase3Audit hand_eye_phase3_audit_;
    bool finalized_ = false;
    bool terminalization_attempted_ = false;
};

#endif  // CALIBRATION_SESSION_HPP
