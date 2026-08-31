#include "calibration_utils.hpp"
#include "calibration_session.hpp"

#include "robotapi.h"
#include "basestruct.h"
#include "utils/robot_util.hpp"
#include "utils/calib_transform.hpp"
#include "hand_eye_gui/DiagnosticLogger.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iomanip>
#include <cerrno>
#include <cstdlib>

namespace {

constexpr int kPostMotionSettlingMs = 1200;
constexpr int kPostMotionDiscardFrames = 1;
constexpr int kSettlingPollMs = 20;

std::uint64_t hostRealtimeNowNs()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}

bool waitForMechanicalSettling(int& waited_ms)
{
    waited_ms = 0;
    while (waited_ms < kPostMotionSettlingMs) {
        if (robotMotionCancellationRequested())
            return false;
        const int slice_ms = std::min(kSettlingPollMs,
                                      kPostMotionSettlingMs - waited_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(slice_ms));
        waited_ms += slice_ms;
    }
    return !robotMotionCancellationRequested();
}

bool parseDoubles(const std::string& line,
                  double* values,
                  size_t expectedCount)
{
    std::stringstream ss(line);
    size_t parsed = 0;
    for (; parsed < expectedCount; ++parsed) {
        std::string token;
        if (!std::getline(ss, token, ','))
            return false;
        if (token.empty())
            return false;

        char* end = nullptr;
        errno = 0;
        const double value = std::strtod(token.c_str(), &end);
        if (errno != 0 || end == token.c_str() || *end != '\0')
            return false;
        values[parsed] = value;
    }
    return true;
}

template <typename Values>
std::string formatAuditValues(const Values& values)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4) << "[";
    for (int axis = 0; axis < ROB_AXIS_NUM; ++axis) {
        if (axis > 0)
            stream << ",";
        stream << values[axis];
    }
    stream << "]";
    return stream.str();
}

std::string formatMotionDiagnostics(const RobotMotionDiagnostics& motion)
{
    return "outcome=" + motion.outcome +
        " phase=" + motion.phase +
        " origin=" + motion.origin +
        " sdk_function=" + motion.sdk_function +
        " result=" + std::to_string(motion.result) +
        " sdk_result=" + std::to_string(motion.sdk_result) +
        " stop_result=" + std::to_string(motion.stop_result) +
        " elapsed_ms=" + std::to_string(motion.elapsed_ms) +
        " timeout_ms=" + std::to_string(motion.timeout_ms) +
        " polls=" + std::to_string(motion.polls) +
        " joint_read_failures=" + std::to_string(motion.joint_read_failures) +
        " status_read_failures=" + std::to_string(motion.status_read_failures) +
        " stable_polls=" + std::to_string(motion.stable_polls) +
        " moving=" + std::string(motion.moving ? "true" : "false") +
        " initial_error_deg=" + std::to_string(motion.initial_error_degrees) +
        " initial=" + formatAuditValues(motion.initial) +
        " target=" + formatAuditValues(motion.target) +
        " actual=" + formatAuditValues(motion.actual);
}

} // namespace

int reconnectRobotHandle(int old_handle, char* pchRobotIP){
    CRresult result = error;
    cr_destroy_robot(old_handle);
    int waitingTime = 120000, try_time=0;
    while(result != sucess){
       std::cout << "[reconnectRobotHandle] wait to re-connect robot handle: " << ", waiting " << waitingTime << " milliseconds for " << try_time <<" connect\n" << std::flush;
       std::this_thread::sleep_for(std::chrono::milliseconds(waitingTime));
       result = cr_create_robot(&old_handle, pchRobotIP, 2323, "123");
       try_time ++;
    }
    if( result == sucess)
        std::cout << "[reconnectRobotHandle] success!!\n" << std::flush;
    return old_handle;
}

bool executeCollection_bk(int robot_handle,
                       char* pchRobotIP,
                       std::shared_ptr<AICamera>& camera,
                       CharucoDetector& detector,
                       const std::vector<PoseCandidate>& candidates,
                       const std::string& save_dir,
                       AI_det_callback callback) {
    int total = (int)candidates.size();
    int captured = 0;

    if (save_dir.empty()) {
        std::cerr << "[autoCalib] 保存目录为空" << std::endl;
        return false;
    }
    // 准备保存目录
    std::string pos_file, joint_file;
    {
        namespace fs = std::filesystem;
        if (fs::exists(save_dir))
            fs::remove_all(save_dir);
        fs::create_directories(save_dir);
    }
    pos_file = save_dir + "/rob_pos.csv";
    joint_file = save_dir + "/rob_joint.csv";

    for (int i = 0; i < total; i++) {
        std::cout << "\r[autoCalib] 采集进度: " << (i + 1) << "/" << total
                  << " (有效: " << captured << ")"
                  << std::flush;

        int progress = 0;
        {
            int bar_width = 60;
            int filled = (i + 1) * bar_width / total;
            std::cout << "\n[";
            for (int k = 0; k < bar_width; k++)
                std::cout << (k < filled ? '#' : ' ');
            progress = (i + 1) * 100 / total;
            std::cout << "] " << progress << "%\r\033[A" << std::flush;
        }
        double init_joint[ROB_AXIS_NUM], init_joint2[ROB_AXIS_NUM], dst_joint[ROB_AXIS_NUM];
        double init_eule[ROB_AXIS_NUM], init_eule2[ROB_AXIS_NUM], dst_eule[ROB_AXIS_NUM];
        cr_get_jointActualPos((RobotHandle)robot_handle, init_joint);
        convertJointToFlangeEule(robot_handle, init_joint, init_eule);
        convertJointToFlangeEule(robot_handle, candidates[i].joint, dst_eule);
        memcpy(dst_eule+3, init_eule+3, 3 * sizeof(double));
        convertFlangeOulePos2Joint(robot_handle, dst_eule, dst_joint);
        CRresult ret_move = moveToJointPosByLine(robot_handle, dst_joint);
        if(sucess != ret_move){
            cr_get_jointActualPos((RobotHandle)robot_handle, init_joint2);
            convertJointToFlangeEule(robot_handle, init_joint2, init_eule2);
            moveToJointPos(robot_handle, init_joint);
            moveToJointPosByLine(robot_handle, dst_joint);
            if(operate_timeout == ret_move){
                // robot_handle 是按值传入的。在此处销毁并重连只会更新局部副本，
                // 控制器仍持有已经销毁的旧句柄。安全做法是终止本轮标定，
                // 由控制器在工作线程结束后统一执行重连。
                std::cerr << "\n[autoCalib] 机器人运动超时，终止本轮标定，请重新连接机器人"
                          << std::endl;
                return false;
            }
            continue;
        }

        cv::Mat frame = camera->captureOneFrame(1500);
        if (frame.empty()) {
            std::cerr << "\n[autoCalib] 采集图像失败 (pose " << i << ")" << std::endl;
            continue;
        }

        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detector.detectCorners(frame, corners, ids))
            continue;
        if(ids.size() < 12)
           continue;    
           
        // double tcp_pose[ROB_AXIS_NUM];
        // CRresult ret = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_pose);
        // if (ret != sucess) continue;

        // double flange_pose__[ROB_AXIS_NUM];
        // if(!convertTcpAxisPos2FlangPos(robot_handle, tcp_pose, flange_pose__))
        //     continue;

        // 获取实际关节角
        double actual_joint[ROB_AXIS_NUM], flange_pose[ROB_AXIS_NUM];
        cr_get_jointActualPos((RobotHandle)robot_handle, actual_joint);
        cr_kineForward(robot_handle, actual_joint, flange_pose);

        // 保存到磁盘
        {
            std::string img_name = save_dir + "/" +
                (captured < 10 ? "00" : captured < 100 ? "0" : "") +
                std::to_string(captured) + ".jpg";
            cv::imwrite(img_name, frame);
            if (callback) {
                int call_ret = callback((char*)img_name.c_str(), (int)img_name.length(), progress);
                if (-1==call_ret)
                {
                    std::cerr << "\r[autoCalib] 用户取消了  采集进度"  << std::endl;
                    return false;
                }
            }
        }
        // rob_pos.csv: 法兰轴角位姿 (mm, deg)
        {
            std::ofstream f(pos_file, std::ios::app);
            f << std::fixed << std::setprecision(3);
            for (int j = 0; j < 6; j++)
                f << flange_pose[j] << (j < 5 ? "," : "\n");
        }
        // rob_joint.csv: 关节角 (deg)
        {
            std::ofstream f(joint_file, std::ios::app);
            f << std::fixed << std::setprecision(3);
            for (int j = 0; j < 6; j++)
                f << actual_joint[j] << (j < 5 ? "," : "\n");
        }

        captured++;
    }

    std::cout << "\n[autoCalib] 采集完成。有效帧: " << captured
              << "/" << total
              << ", 数据保存到: " << save_dir << std::endl;
    return captured > 0;
}



bool executeCollection(int robot_handle,
                       char* pchRobotIP,
                       std::shared_ptr<AICamera>& camera,
                       CharucoDetector& detector,
                       const std::vector<PoseCandidate>& candidates,
                       const std::string& save_dir,
                       AI_det_callback callback,
                       CalibrationSession* session,
                       const std::string& robot_pose_dir,
                       CalibrationRuntimeStatus* runtime_status,
                       const CalibrationRuntimeCallback& runtime_callback) {
    int total = (int)candidates.size();
    const int capture_targets = static_cast<int>(std::count_if(
        candidates.begin(), candidates.end(),
        [](const PoseCandidate& candidate) { return candidate.capture; }));
    int captured = 0;
    int skipped = 0;
    int transitions_completed = 0;

    auto publishCollection = [&](const std::string& detail = {}) {
        if (!runtime_status)
            return;
        runtime_status->captured_frames = captured;
        runtime_status->skipped_frames = skipped;
        runtime_status->candidate_processed = captured + skipped +
                                              transitions_completed;
        publishCalibrationRuntime(runtime_callback, *runtime_status,
                                  CalibrationRuntimeStage::Collecting, detail);
    };

    APP_LOG_EVENT_INFO("calibration.collection",
                       "collection",
                       "start",
                       {{"handle", QString::number(robot_handle)},
                        {"candidates", QString::number(total)},
                        {"capture_targets", QString::number(capture_targets)},
                        {"transition_moves", QString::number(
                            total - capture_targets)},
                        {"save_dir", QString::fromStdString(save_dir)}});

    if (runtime_status) {
        runtime_status->candidate_reachable = total;
        runtime_status->current_candidate_index = -1;
        runtime_status->current_corner_count = -1;
        publishCollection("collection_start");
    }

    if (save_dir.empty()) {
        if (session)
            session->appendEvent("collection", "fail", "save_directory_empty");
        std::cerr << "[autoCalib] 保存目录为空" << std::endl;
        return false;
    }
    // 准备保存目录
    std::string pos_file, joint_file;
    {
        namespace fs = std::filesystem;
        if (fs::exists(save_dir))
            fs::remove_all(save_dir);
        fs::create_directories(save_dir);
        if (!robot_pose_dir.empty()) {
            if (fs::exists(robot_pose_dir))
                fs::remove_all(robot_pose_dir);
            fs::create_directories(robot_pose_dir);
        }
    }
    const std::string pose_output_dir = robot_pose_dir.empty()
        ? save_dir : robot_pose_dir;
    pos_file = pose_output_dir + "/rob_pos.csv";
    joint_file = pose_output_dir + "/rob_joint.csv";

    for (int i = 0; i < total; i++) {
#ifndef QT_GUI
        std::cout << "\r[autoCalib] 采集进度: " << (i + 1) << "/" << total
                  << " (有效: " << captured << ")"
                  << std::flush;
#endif

        if (runtime_status) {
            runtime_status->current_candidate_index = i;
            runtime_status->current_corner_count = 0;
        }

        int progress = 0;
        {
            progress = (i + 1) * 100 / total;
#ifndef QT_GUI
            int bar_width = 60;
            int filled = (i + 1) * bar_width / total;
            std::cout << "\n[";
            for (int k = 0; k < bar_width; k++)
                std::cout << (k < filled ? '#' : ' ');
            std::cout << "] " << progress << "%\r\033[A" << std::flush;
#endif
        }        
        if (session) {
            session->appendEvent(
                "candidate_motion", "start",
                "ordered=" + std::to_string(i) +
                " from_source=" + std::to_string(
                    candidates[i].previous_source_index) +
                " source=" + std::to_string(candidates[i].source_index) +
                " role=" + std::string(
                    candidates[i].capture ? "capture" : "transition") +
                " axis_steps_deg=" + formatAuditValues(
                    candidates[i].step_from_previous) +
                " target=" + formatAuditValues(candidates[i].joint));
        }
        RobotMotionDiagnostics motion_diagnostics;
        CRresult ret_move = moveToJointPos(robot_handle, candidates[i].joint,
                                           &motion_diagnostics);
        if(sucess != ret_move){
            if (ret_move == robot_motion_cancelled ||
                robotMotionCancellationRequested()) {
                if (session)
                    session->appendEvent("collection_frame", "cancelled",
                        "candidate=" + std::to_string(i));
                APP_LOG_EVENT_WARNING("calibration.collection",
                                      "collection",
                                      "cancelled",
                                      {{"phase", "robot_move"},
                                       {"index", QString::number(i)},
                                       {"captured", QString::number(captured)}});
                if (runtime_status) {
                    publishCalibrationRuntime(runtime_callback, *runtime_status,
                                              CalibrationRuntimeStage::Cancelled,
                                              "collection_cancelled");
                }
                return false;
            }
            APP_LOG_EVENT_WARNING("calibration.collection",
                                  "collection",
                                  "skip",
                                  {{"phase", "robot_move"},
                                   {"reason", "move_failed"},
                                   {"index", QString::number(i)},
                                   {"total", QString::number(total)},
                                   {"error", QString::number(ret_move)}});
            if (session)
                session->appendEvent("collection_frame", "skip",
                    "candidate=" + std::to_string(i) +
                    " source=" + std::to_string(candidates[i].source_index) +
                    " reason=move_failed error=" + std::to_string(ret_move) +
                    " " + formatMotionDiagnostics(motion_diagnostics));
            if(operate_timeout == ret_move){
                APP_LOG_EVENT_ERROR("calibration.collection",
                                    "collection",
                                    "fail",
                                    {{"phase", "robot_move"},
                                     {"reason", "motion_timeout"},
                                     {"index", QString::number(i)},
                                     {"captured", QString::number(captured)}});
                std::cerr << "\n[autoCalib] 机器人运动超时，终止本轮标定，请重新连接机器人"
                          << std::endl;
                if (session)
                    session->appendEvent("collection", "fail", "motion_timeout");
                if (runtime_status) {
                    publishCalibrationRuntime(runtime_callback, *runtime_status,
                                              CalibrationRuntimeStage::Failed,
                                              "motion_timeout");
                }
                return false;
            }
            // 后续路径以上一个节点已经到达为前提。运动失败后继续执行会绕过
            // 规划时的相邻关节门禁，因此必须终止并重新规划。
            if (session)
                session->appendEvent("collection", "fail",
                    "candidate_route_invalid_after_move_failure ordered=" +
                    std::to_string(i));
            if (runtime_status) {
                publishCalibrationRuntime(runtime_callback, *runtime_status,
                                          CalibrationRuntimeStage::Failed,
                                          "candidate_route_invalid");
            }
            return false;
        }
        if (session) {
            session->appendEvent(
                "candidate_motion", "success",
                "ordered=" + std::to_string(i) +
                " source=" + std::to_string(candidates[i].source_index) +
                " " + formatMotionDiagnostics(motion_diagnostics));
        }

        if (!candidates[i].capture) {
            ++transitions_completed;
            if (session) {
                session->appendEvent(
                    "candidate_transition", "success",
                    "ordered=" + std::to_string(i) +
                    " source=" + std::to_string(candidates[i].source_index) +
                    " joint=" + formatAuditValues(candidates[i].joint));
            }
            APP_LOG_EVENT_INFO("calibration.collection",
                               "candidate_transition",
                               "success",
                               {{"index", QString::number(i)},
                                {"source", QString::number(
                                    candidates[i].source_index)},
                                {"completed", QString::number(
                                    transitions_completed)}});
            publishCollection("candidate_transition_complete");
            continue;
        }

        // moveToJointPos() 已等待控制器连续报告停止，并保留了 100 ms 的短窗口。
        // 标定采集还需要给机械结构额外的衰减时间，避免“控制器已停止但末端仍在微振”时
        // 将旧图像与稍后的机器人反馈位姿配成一组。
        const std::uint64_t motion_completed_timestamp_ns = hostRealtimeNowNs();
        APP_LOG_EVENT_DEBUG("calibration.collection",
                           "mechanical_settling",
                           "start",
                           {{"index", QString::number(i)},
                            {"requested_ms", QString::number(kPostMotionSettlingMs)},
                            {"discard_frames", QString::number(kPostMotionDiscardFrames)}});
        int settling_waited_ms = 0;
        if (!waitForMechanicalSettling(settling_waited_ms)) {
            if (session)
                session->appendEvent("collection_frame", "cancelled",
                    "candidate=" + std::to_string(i) + " phase=mechanical_settling");
            APP_LOG_EVENT_WARNING("calibration.collection",
                                  "mechanical_settling",
                                  "cancelled",
                                  {{"index", QString::number(i)},
                                   {"waited_ms", QString::number(settling_waited_ms)}});
            if (runtime_status) {
                publishCalibrationRuntime(runtime_callback, *runtime_status,
                                          CalibrationRuntimeStage::Cancelled,
                                          "mechanical_settling_cancelled");
            }
            return false;
        }

        int discarded_frames = 0;
        int empty_discard_frames = 0;
        for (int discard = 0; discard < kPostMotionDiscardFrames; ++discard) {
            if (robotMotionCancellationRequested()) {
                if (session)
                    session->appendEvent("collection_frame", "cancelled",
                        "candidate=" + std::to_string(i) + " phase=camera_flush");
                APP_LOG_EVENT_WARNING("calibration.collection",
                                      "mechanical_settling",
                                      "cancelled",
                                      {{"index", QString::number(i)},
                                       {"phase", "camera_flush"}});
                if (runtime_status) {
                    publishCalibrationRuntime(runtime_callback, *runtime_status,
                                              CalibrationRuntimeStage::Cancelled,
                                              "camera_flush_cancelled");
                }
                return false;
            }
            if (camera->captureOneFrame(500).empty())
                ++empty_discard_frames;
            else
                ++discarded_frames;
        }
        APP_LOG_EVENT_DEBUG("calibration.collection",
                           "mechanical_settling",
                           "complete",
                           {{"index", QString::number(i)},
                            {"requested_ms", QString::number(kPostMotionSettlingMs)},
                            {"discarded_frames", QString::number(discarded_frames)},
                            {"empty_discard_frames", QString::number(empty_discard_frames)}});

        // 偶发的曝光或帧同步问题不应该直接浪费一个有价值的机器人姿态。
        // 首帧合格立即结束；只有首帧不合格时才最多重试两次。
        constexpr int kMaximumCaptureAttempts = 3;
        cv::Mat frame;
        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        int capture_attempts = 0;
        int empty_attempts = 0;
        int detection_failures = 0;
        std::uint64_t image_timestamp_ns = 0;
        for (int attempt = 1; attempt <= kMaximumCaptureAttempts; ++attempt) {
            ++capture_attempts;
            cv::Mat candidate_frame = camera->captureOneFrame(1500);
            const std::uint64_t candidate_timestamp_ns = hostRealtimeNowNs();
            if (candidate_frame.empty()) {
                ++empty_attempts;
                continue;
            }

            std::vector<cv::Point2f> candidate_corners;
            std::vector<int> candidate_ids;
            const bool detected = detector.detectCorners(
                candidate_frame, candidate_corners, candidate_ids);
            if (!detected)
                ++detection_failures;

            if (frame.empty() || candidate_ids.size() > ids.size()) {
                frame = candidate_frame;
                corners = std::move(candidate_corners);
                ids = std::move(candidate_ids);
                image_timestamp_ns = candidate_timestamp_ns;
            }
            if (detected && ids.size() >= 12)
                break;
        }

        if (frame.empty()) {
            if (session)
                session->appendEvent("collection_frame", "skip",
                    "candidate=" + std::to_string(i) + " reason=capture_empty");
            APP_LOG_EVENT_WARNING("calibration.collection",
                                  "collection",
                                  "skip",
                                  {{"phase", "image_capture"},
                                   {"reason", "capture_empty"},
                                   {"index", QString::number(i)},
                                   {"captured", QString::number(captured)},
                                   {"attempts", QString::number(capture_attempts)},
                                   {"empty_attempts", QString::number(empty_attempts)}});
            std::cerr << "\n[autoCalib] 采集图像失败 (pose " << i << ")" << std::endl;
            ++skipped;
            publishCollection("candidate_skip_capture_empty");
            continue;
        }
        if (ids.empty()) {
            if (session)
                session->appendEvent("collection_frame", "skip",
                    "candidate=" + std::to_string(i) + " reason=detection_failed");
            APP_LOG_EVENT_WARNING("calibration.collection",
                                  "collection",
                                  "skip",
                                  {{"phase", "corner_detection"},
                                   {"reason", "detection_failed"},
                                   {"index", QString::number(i)},
                                   {"image_w", QString::number(frame.cols)},
                                   {"image_h", QString::number(frame.rows)},
                                   {"attempts", QString::number(capture_attempts)},
                                   {"detection_failures", QString::number(detection_failures)}});
            if (runtime_status)
                runtime_status->current_corner_count = 0;
            ++skipped;
            publishCollection("candidate_skip_detection_failed");
            continue;
        }
        if(ids.size() < 12) {
            if (session)
                session->appendEvent("collection_frame", "skip",
                    "candidate=" + std::to_string(i) + " reason=insufficient_corners corners=" +
                    std::to_string(ids.size()));
            APP_LOG_EVENT_WARNING("calibration.collection",
                                  "collection",
                                  "skip",
                                  {{"phase", "corner_detection"},
                                   {"reason", "insufficient_corners"},
                                   {"index", QString::number(i)},
                                   {"corners", QString::number(ids.size())},
                                   {"minimum", "12"},
                                   {"attempts", QString::number(capture_attempts)}});
            if (runtime_status)
                runtime_status->current_corner_count = static_cast<int>(ids.size());
            ++skipped;
            publishCollection("candidate_skip_insufficient_corners");
            continue;
        }
        if (capture_attempts > 1) {
            APP_LOG_EVENT_WARNING("calibration.collection",
                               "collection_capture_retry",
                               "recovered",
                               {{"index", QString::number(i)},
                                {"attempts", QString::number(capture_attempts)},
                                {"empty_attempts", QString::number(empty_attempts)},
                                {"detection_failures", QString::number(detection_failures)},
                                {"corners", QString::number(ids.size())}});
        }
           
        // double tcp_pose[ROB_AXIS_NUM];
        // CRresult ret = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_pose);
        // if (ret != sucess) continue;

        // double flange_pose__[ROB_AXIS_NUM];
        // if(!convertTcpAxisPos2FlangPos(robot_handle, tcp_pose, flange_pose__))
        //     continue;

        // 获取实际关节角
        double actual_joint[ROB_AXIS_NUM] = {};
        double flange_pose[ROB_AXIS_NUM] = {};
        CRresult poseRet = cr_get_jointActualPos((RobotHandle)robot_handle, actual_joint);
        if (poseRet != sucess) {
            if (session)
                session->appendEvent("collection_frame", "skip",
                    "candidate=" + std::to_string(i) + " reason=joint_read_failed");
            APP_LOG_EVENT_ERROR("calibration.collection",
                                "collection",
                                "fail",
                                {{"phase", "pose_save"},
                                 {"reason", "joint_read_failed"},
                                 {"index", QString::number(i)},
                                 {"error", QString::number(poseRet)}});
            ++skipped;
            publishCollection("candidate_skip_joint_read_failed");
            continue;
        }
        poseRet = cr_kineForward(robot_handle, actual_joint, flange_pose);
        if (poseRet != sucess) {
            if (session)
                session->appendEvent("collection_frame", "skip",
                    "candidate=" + std::to_string(i) + " reason=forward_kinematics_failed");
            APP_LOG_EVENT_ERROR("calibration.collection",
                                "collection",
                                "fail",
                                {{"phase", "pose_save"},
                                 {"reason", "forward_kinematics_failed"},
                                 {"index", QString::number(i)},
                                 {"error", QString::number(poseRet)}});
            ++skipped;
            publishCollection("candidate_skip_forward_kinematics_failed");
            continue;
        }
        const std::uint64_t robot_timestamp_ns = hostRealtimeNowNs();

        // 保存到磁盘
        {
            std::string img_name = save_dir + "/" +
                (captured < 10 ? "00" : captured < 100 ? "0" : "") +
                std::to_string(captured) + ".jpg";
            if (!cv::imwrite(img_name, frame)) {
                if (session)
                    session->appendEvent("collection_frame", "skip",
                        "candidate=" + std::to_string(i) + " reason=image_write_failed");
                APP_LOG_EVENT_ERROR("calibration.collection",
                                    "collection",
                                    "fail",
                                    {{"phase", "image_save"},
                                     {"reason", "image_write_failed"},
                                     {"index", QString::number(i)},
                                     {"path", QString::fromStdString(img_name)}});
                ++skipped;
                publishCollection("candidate_skip_image_write_failed");
                continue;
            }
            if (callback) {
                int call_ret = callback((char*)img_name.c_str(), (int)img_name.length(), progress);
                if (-1==call_ret)
                {
                    std::cerr << "\r[autoCalib] 用户取消了  采集进度"  << std::endl;
                    APP_LOG_EVENT_WARNING("calibration.collection",
                                          "collection",
                                          "skip",
                                          {{"phase", "user_intervention"},
                                           {"reason", "cancelled"},
                                           {"index", QString::number(i)},
                                           {"captured", QString::number(captured)}});
                    if (session)
                        session->appendEvent("collection_frame", "cancelled",
                            "candidate=" + std::to_string(i));
                    if (runtime_status) {
                        publishCalibrationRuntime(runtime_callback, *runtime_status,
                                                  CalibrationRuntimeStage::Cancelled,
                                                  "user_cancelled");
                    }
                    return false;
                }
            }
        }
        // rob_pos.csv: 法兰轴角位姿 (mm, deg)
        {
            std::ofstream f(pos_file, std::ios::app);
            f << std::fixed << std::setprecision(3);
            for (int j = 0; j < 6; j++)
                f << flange_pose[j] << (j < 5 ? "," : "\n");
            f.flush();
            if (!f) {
                APP_LOG_EVENT_ERROR("calibration.collection",
                                    "collection",
                                    "fail",
                                    {{"phase", "pose_save"},
                                     {"reason", "robot_pose_csv_write_failed"},
                                     {"index", QString::number(i)},
                                     {"path", QString::fromStdString(pos_file)}});
                if (session)
                    session->appendEvent("collection", "fail",
                        "robot_pose_csv_write_failed");
                return false;
            }
        }
        // rob_joint.csv: 关节角 (deg)
        {
            std::ofstream f(joint_file, std::ios::app);
            f << std::fixed << std::setprecision(3);
            for (int j = 0; j < 6; j++)
                f << actual_joint[j] << (j < 5 ? "," : "\n");
            f.flush();
            if (!f) {
                APP_LOG_EVENT_ERROR("calibration.collection",
                                    "collection",
                                    "fail",
                                    {{"phase", "pose_save"},
                                     {"reason", "robot_joint_csv_write_failed"},
                                     {"index", QString::number(i)},
                                     {"path", QString::fromStdString(joint_file)}});
                if (session)
                    session->appendEvent("collection", "fail",
                        "robot_joint_csv_write_failed");
                return false;
            }
        }
        if (session) {
            std::string audit_error;
            if (!session->writeRawRobotPose(captured, i, actual_joint, flange_pose,
                                            motion_completed_timestamp_ns,
                                            kPostMotionSettlingMs,
                                            discarded_frames,
                                            image_timestamp_ns, robot_timestamp_ns,
                                            audit_error)) {
                APP_LOG_EVENT_ERROR("calibration.collection",
                                    "collection_audit",
                                    "fail",
                                    {{"reason", "raw_pose_write_failed"},
                                     {"index", QString::number(i)},
                                     {"message", QString::fromStdString(audit_error)}});
                std::cerr << "\n[autoCalib] 审计位姿写入失败: " << audit_error << std::endl;
                return false;
            }
        }

        captured++;
        if (runtime_status)
            runtime_status->current_corner_count = static_cast<int>(ids.size());
        if (session)
            session->appendEvent("collection_frame", "captured",
                "frame=" + std::to_string(captured - 1) +
                " candidate=" + std::to_string(i) +
                " corners=" + std::to_string(ids.size()) +
                " image_sha256=" + CalibrationSession::sha256File(
                    save_dir + "/" +
                    (captured - 1 < 10 ? "00" : captured - 1 < 100 ? "0" : "") +
                    std::to_string(captured - 1) + ".jpg"));
        APP_LOG_EVENT_DEBUG("calibration.collection",
                           "collection",
                           "captured",
                           {{"phase", "save"},
                            {"index", QString::number(i)},
                            {"captured", QString::number(captured)},
                            {"total", QString::number(total)},
                            {"corners", QString::number(ids.size())},
                            {"capture_attempts", QString::number(capture_attempts)},
                            {"settling_requested_ms", QString::number(kPostMotionSettlingMs)},
                            {"discarded_frames", QString::number(discarded_frames)},
                            {"motion_to_image_ms", QString::number(
                                static_cast<double>(image_timestamp_ns -
                                    motion_completed_timestamp_ns) / 1.0e6, 'f', 3)},
                            {"image_to_robot_ms", QString::number(
                                static_cast<double>(robot_timestamp_ns -
                                    image_timestamp_ns) / 1.0e6, 'f', 3)}});
        publishCollection("candidate_captured");
    }

    std::cout << "\n[autoCalib] 采集完成。有效帧: " << captured
              << "/" << capture_targets
              << ", 过渡动作: " << transitions_completed
              << ", 数据保存到: " << save_dir << std::endl;
    APP_LOG_EVENT_INFO("calibration.collection",
                       "collection",
                       "success",
                       {{"captured", QString::number(captured)},
                        {"capture_targets", QString::number(capture_targets)},
                        {"transition_moves", QString::number(
                            transitions_completed)},
                        {"route_entries", QString::number(total)},
                        {"save_dir", QString::fromStdString(save_dir)}});
    publishCollection("collection_complete");
    return captured > 0;
}

// ============================================================
// 图像/视觉辅助函数
// ============================================================

// 从 ChArUco 角点计算标定板与图像对应边的夹角 (rad)
// 找出图像中跨度最大的行或列方向，对比板长边→图像长边、板短边→图像短边
double computeBoardAngle(const std::vector<cv::Point2f>& corners,
                         const std::vector<int>& ids,
                         int squares_x, int squares_y,
                         int image_width, int image_height) {
    if (corners.size() < 2 || ids.size() < 2) return 0.0;

    int corners_per_row = squares_x - 1;
    int corners_per_col = squares_y - 1;
    bool long_is_x = (squares_x >= squares_y);

    struct LineInfo { cv::Point2f vec; double length; bool is_row; };
    std::vector<LineInfo> lines;

    // 按行分组，找每行最小/最大列号的两个端点
    for (int r = 0; r < corners_per_col; r++) {
        int min_col = squares_x, max_col = -1;
        cv::Point2f pt_min, pt_max;
        for (size_t i = 0; i < ids.size(); i++) {
            if (ids[i] / corners_per_row == r) {
                int col = ids[i] % corners_per_row;
                if (col < min_col) { min_col = col; pt_min = corners[i]; }
                if (col > max_col) { max_col = col; pt_max = corners[i]; }
            }
        }
        if (min_col <= max_col) {
            cv::Point2f v = pt_max - pt_min;
            lines.push_back({v, cv::norm(v), true});
        }
    }

    // 按列分组，找每列最小/最大行号的两个端点
    for (int c = 0; c < corners_per_row; c++) {
        int min_row = squares_y, max_row = -1;
        cv::Point2f pt_min, pt_max;
        for (size_t i = 0; i < ids.size(); i++) {
            if (ids[i] % corners_per_row == c) {
                int row = ids[i] / corners_per_row;
                if (row < min_row) { min_row = row; pt_min = corners[i]; }
                if (row > max_row) { max_row = row; pt_max = corners[i]; }
            }
        }
        if (min_row <= max_row) {
            cv::Point2f v = pt_max - pt_min;
            lines.push_back({v, cv::norm(v), false});
        }
    }

    if (lines.empty()) return 0.0;

    const LineInfo& best = *std::max_element(lines.begin(), lines.end(),
        [](const LineInfo& a, const LineInfo& b) { return a.length < b.length; });

    (void)image_width;
    (void)image_height;
    double angle = std::atan2(best.vec.y, best.vec.x);
    if (!long_is_x) //  (!ref_is_horiz)
        angle -= CV_PI / 2.0;

    if (angle > CV_PI / 2)
        angle = angle - CV_PI;
    if (angle < -CV_PI / 2)
        angle = CV_PI + angle;
    return angle;
}

// 计算角点凸包在两个方向上的像素跨度（用于基于像素变化估计距离）
// 返回最大方向跨度; out_w/out_h 为可选输出（宽度、高度）
float computePixelSpan(const std::vector<cv::Point2f>& corners,
                       float* out_w,
                       float* out_h) {
    float min_x = std::numeric_limits<float>::max(), max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max(), max_y = -std::numeric_limits<float>::max();
    for (auto& pt : corners) {
        min_x = std::min(min_x, pt.x); max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y); max_y = std::max(max_y, pt.y);
    }
    if (out_w) *out_w = max_x - min_x;
    if (out_h) *out_h = max_y - min_y;
    return std::max(max_x - min_x, max_y - min_y);
}

// 从两组具有共同 ID 的 ChArUco 角点中，找第一帧距离最远的点对，
// 返回该点对在两帧中的距离（用于双图像高度估计，避免个别角点缺失的影响）
bool computeCommonSpan(const std::vector<cv::Point2f>& corners1,
                       const std::vector<int>& ids1,
                       const std::vector<cv::Point2f>& corners2,
                       const std::vector<int>& ids2,
                       float& dist1, float& dist2) {
    // 收集共同 ID 的点
    std::vector<cv::Point2f> pts1, pts2;
    for (size_t i = 0; i < ids2.size(); i++) {
        for (size_t j = 0; j < ids1.size(); j++) {
            if (ids2[i] == ids1[j]) {
                pts1.push_back(corners1[j]);
                pts2.push_back(corners2[i]);
                break;
            }
        }
    }
    if (pts1.size() < 2) return false;

    // 找第一帧中距离最远的点对
    dist1 = 0; dist2 = 0;
    for (size_t i = 0; i < pts1.size(); i++) {
        for (size_t j = i + 1; j < pts1.size(); j++) {
            float d1 = cv::norm(pts1[i] - pts1[j]);
            if (d1 > dist1) {
                dist1 = d1;
                dist2 = cv::norm(pts2[i] - pts2[j]);
            }
        }
    }
    return true;
}

// 计算 ChArUco 角点凸包区域的平均深度（0=无效深度）
double computeAvgDepth(const cv::Mat& depth, const std::vector<cv::Point2f>& corners) {
    if (depth.empty() || depth.type() != CV_16UC1 || corners.size() < 3)
        return 0.0;

    std::vector<cv::Point2f> hull_f;
    cv::convexHull(corners, hull_f);
    std::vector<cv::Point2i> hull;
    for (auto& p : hull_f) hull.emplace_back(cv::Point2i(p));
    cv::Mat mask = cv::Mat::zeros(depth.size(), CV_8UC1);
    cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{hull}, cv::Scalar(255));

    double sum = 0.0;
    int cnt = 0;
    for (int r = 0; r < depth.rows; r++) {
        for (int c = 0; c < depth.cols; c++) {
            if (mask.at<uchar>(r, c)) {
                ushort d = depth.at<ushort>(r, c);
                if (d > 0 && d <= 3000) { sum += d; cnt++; }
            }
        }
    }
    return (cnt > 0) ? (sum / cnt) : 0.0;
}

// 将法兰 rx=-180, ry=0 旋转到水平姿态
bool rotateFlangeToHorizon(int robot_handle, double *pFlangeEulePos){
    double flangeEulePos[ROB_AXIS_NUM] = {};
    double tcp_aa[ROB_AXIS_NUM] = {};
    CRresult ret = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
    if (ret != sucess) {
        APP_LOG_EVENT_ERROR("calibration.robot",
                            "robot_pose",
                            "fail",
                            {{"phase", "rotate_horizontal"},
                             {"reason", "tcp_read_failed"},
                             {"handle", QString::number(robot_handle)},
                             {"error", QString::number(ret)}});
        return false;
    }
    if (!convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos)) {
        APP_LOG_EVENT_ERROR("calibration.robot",
                            "robot_pose",
                            "fail",
                            {{"phase", "rotate_horizontal"},
                             {"reason", "tcp_to_flange_failed"},
                             {"handle", QString::number(robot_handle)}});
        return false;
    }
    flangeEulePos[3] = -180.0;
    flangeEulePos[4] = 0.0;

    APP_LOG_EVENT_INFO("calibration.robot",
                       "robot_pose",
                       "start",
                       {{"phase", "rotate_horizontal"},
                        {"handle", QString::number(robot_handle)},
                        {"target_x", QString::number(flangeEulePos[0], 'f', 3)},
                        {"target_y", QString::number(flangeEulePos[1], 'f', 3)},
                        {"target_z", QString::number(flangeEulePos[2], 'f', 3)},
                        {"target_rx", QString::number(flangeEulePos[3], 'f', 3)},
                        {"target_ry", QString::number(flangeEulePos[4], 'f', 3)},
                        {"target_rz", QString::number(flangeEulePos[5], 'f', 3)}});
    if (!moveToFlangeEulePos(robot_handle, flangeEulePos)) {
        APP_LOG_EVENT_ERROR("calibration.robot",
                            "robot_pose",
                            "fail",
                            {{"phase", "rotate_horizontal"},
                             {"reason", "move_failed"},
                             {"handle", QString::number(robot_handle)}});
        return false;
    }
    if(pFlangeEulePos!= NULL){
      memcpy(pFlangeEulePos, flangeEulePos, sizeof(flangeEulePos));
    }
    APP_LOG_EVENT_INFO("calibration.robot",
                       "robot_pose",
                       "success",
                       {{"phase", "rotate_horizontal"},
                        {"handle", QString::number(robot_handle)}});
    return true;
}

// 将角度限制在 [-90, 90] 范围
float bound_camera_angle(float angle){
    if(angle > 90)
       return angle - 180;
    if(angle < -90)
       return 180 + angle;
    return angle;
}

// 估计相机到标定板的距离（深度相机优先，无深度时自动回退到双图像法）
// 同时输出采集到的 frame 和角点（后续 Z 值迭代需要）
bool estimateCameraHeight(
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    int robot_handle,
    const double oriFlangeEulePos[6],
    cv::Mat& out_frame,
    std::vector<cv::Point2f>& out_corners,
    std::vector<int>& out_ids,
    double& heightCamera) {

    if (!camera) {
        APP_LOG_EVENT_ERROR("calibration.height",
                            "height_estimate",
                            "fail",
                            {{"reason", "null_camera"}});
        return false;
    }

    // 采集一帧（含深度，如果有深度相机）
    cv::Mat depth;
    for (int i = 0; i < 3; i++) {
        const int captureRet = camera->captureOneFrameWithDepth(out_frame, depth);
        APP_LOG_EVENT_DEBUG("calibration.height",
                            "height_estimate",
                            "capture_attempt",
                            {{"attempt", QString::number(i + 1)},
                             {"ret", QString::number(captureRet)},
                             {"color_w", QString::number(out_frame.cols)},
                             {"color_h", QString::number(out_frame.rows)},
                             {"depth_w", QString::number(depth.cols)},
                             {"depth_h", QString::number(depth.rows)}});
        if (captureRet != 0 && out_frame.empty())
            continue;
        if (!out_frame.empty() && !depth.empty()) break;
    }
    if (out_frame.empty()) {
        APP_LOG_EVENT_ERROR("calibration.height",
                            "height_estimate",
                            "fail",
                            {{"reason", "color_capture_failed"},
                             {"attempts", "3"}});
        std::cout << "[estimateCameraHeight] 采集图像失败" << std::endl;
        return false;
    }

    if (!detector.detectCorners(out_frame, out_corners, out_ids) || out_ids.size() < 4) {
        APP_LOG_EVENT_WARNING("calibration.height",
                              "height_estimate",
                              "fail",
                              {{"reason", "insufficient_corners"},
                               {"corners", QString::number(out_ids.size())},
                               {"image_w", QString::number(out_frame.cols)},
                               {"image_h", QString::number(out_frame.rows)}});
        std::cout << "[estimateCameraHeight] ChArUco 角点过少" << std::endl;
        return false;
    }

    heightCamera = 0.0;
    bool heightOk = false;

    // 方案 A: 有深度相机 → 直接用深度数据
    if (!depth.empty() && depth.cols > 0 && depth.rows > 0) {
        double h = computeAvgDepth(depth, out_corners);
        if (h > 0.0) {
            heightCamera = h;
            heightOk = true;
            std::cout << "[autoCalib] 深度相机高度: " << heightCamera << " mm" << std::endl;
            APP_LOG_EVENT_INFO("calibration.height",
                               "height_estimate",
                               "success",
                               {{"method", "depth"},
                                {"height_mm", QString::number(heightCamera, 'f', 3)},
                                {"corners", QString::number(out_ids.size())},
                                {"depth_w", QString::number(depth.cols)},
                                {"depth_h", QString::number(depth.rows)}});
        } else {
            APP_LOG_EVENT_WARNING("calibration.height",
                                  "height_estimate",
                                  "skip",
                                  {{"reason", "depth_invalid"},
                                   {"corners", QString::number(out_ids.size())},
                                   {"depth_w", QString::number(depth.cols)},
                                   {"depth_h", QString::number(depth.rows)},
                                   {"valid_range_mm", "1..3000"}});
        }
    }

    // 方案 B: 无深度 → 双图像法
    if (!heightOk) {
        std::cout << "[estimateCameraHeight] 无深度数据，双图像法估计高度" << std::endl;

        double initJoint[6] = {};
        CRresult jointRet = cr_get_jointActualPos((RobotHandle)robot_handle, initJoint);
        if (jointRet != sucess) {
            APP_LOG_EVENT_ERROR("calibration.height",
                                "height_estimate",
                                "fail",
                                {{"reason", "joint_read"},
                                 {"error", QString::number(jointRet)}});
            return false;
        }
        const double Z_STEP = 150.0;

        double flangeUp[6];
        std::copy(oriFlangeEulePos, oriFlangeEulePos + 6, flangeUp);
        flangeUp[2] = oriFlangeEulePos[2] + Z_STEP;
        if (!moveToFlangeEulePos(robot_handle, flangeUp)) {
            APP_LOG_EVENT_ERROR("calibration.height",
                                "height_estimate",
                                "fail",
                                {{"reason", "move_up"},
                                 {"target_z", QString::number(flangeUp[2], 'f', 3)}});
            return false;
        }

        cv::Mat frame2 = camera->captureOneFrame(1500);
        if (frame2.empty()) {
            const CRresult restoreRet = moveToJointPos(robot_handle, initJoint);
            if (restoreRet != sucess)
                std::cerr << "[estimateCameraHeight] 第二帧采集失败且恢复位置失败 ret="
                          << restoreRet << std::endl;
            APP_LOG_EVENT_ERROR("calibration.height",
                                "height_estimate",
                                "fail",
                                {{"reason", "second_capture"},
                                 {"restore_error", QString::number(restoreRet)}});
            return false;
        }

        std::vector<cv::Point2f> corners2;
        std::vector<int> ids2;
        if (!detector.detectCorners(frame2, corners2, ids2) || ids2.size() < 4) {
            const CRresult restoreRet = moveToJointPos(robot_handle, initJoint);
            if (restoreRet != sucess)
                std::cerr << "[estimateCameraHeight] 第二帧检测失败且恢复位置失败 ret="
                          << restoreRet << std::endl;
            APP_LOG_EVENT_WARNING("calibration.height",
                                  "height_estimate",
                                  "skip",
                                  {{"reason", "second_detection"},
                                   {"corners", QString::number(ids2.size())},
                                   {"restore_error", QString::number(restoreRet)}});
            return false;
        }
        const CRresult restoreRet = moveToJointPos(robot_handle, initJoint);
        if (restoreRet != sucess) {
            APP_LOG_EVENT_ERROR("calibration.height",
                                "height_estimate",
                                "fail",
                                {{"reason", "restore_position"},
                                 {"error", QString::number(restoreRet)}});
            return false;
        }

        float dim1, dim2;
        if (!computeCommonSpan(out_corners, out_ids, corners2, ids2, dim1, dim2)) {
            std::cout << "[estimateCameraHeight] 两帧共同角点不足" << std::endl;
            return false;
        }

        if (dim1 > dim2 && (dim1 - dim2) > 0.5f) {
            heightCamera = Z_STEP * dim2 / (dim1 - dim2);
            std::cout << "[autoCalib] 双图像高度估计: " << heightCamera
                      << " mm (像素 " << dim1 << "→" << dim2 << " px)" << std::endl;
            APP_LOG_EVENT_INFO("calibration.height",
                               "height_estimate",
                               "success",
                               {{"method", "two_images"},
                                {"height_mm", QString::number(heightCamera, 'f', 3)},
                                {"span_before", QString::number(dim1, 'f', 3)},
                                {"span_after", QString::number(dim2, 'f', 3)}});
            heightOk = true;
        }
    }

    if (!heightOk) {
        APP_LOG_EVENT_ERROR("calibration.height",
                            "height_estimate",
                            "fail",
                            {{"reason", "no_valid_method"}});
        std::cout << "[estimateCameraHeight] 高度估计失败" << std::endl;
        return false;
    }
    return true;
}

// ============================================================
// 从 CSV 加载机器人位姿（法兰轴角 → 4x4）
// ============================================================
bool loadRobotPosesFromCSV(const std::string& csv_file,
                            std::vector<cv::Mat>& robot_poses) {
    std::ifstream f(csv_file);
    if (!f.is_open()) {
        std::cerr << "[calibUtil] 无法打开 " << csv_file << std::endl;
        return false;
    }

    std::string line;
    int count = 0;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        double vals[6];
        if (!parseDoubles(line, vals, 6)) {
            std::cerr << "[calibUtil] rob_pos.csv 行格式错误: " << line << std::endl;
            continue;
        }
        robot_poses.push_back(calib_transform::axisAnglePoseToTransform(vals));
        count++;
    }

    std::cout << "[calibUtil] 从 " << csv_file << " 加载 " << count << " 帧机器人位姿" << std::endl;
    return count > 0;
}

// ============================================================
// 将 rob_eule.csv（欧拉角）转换为 rob_pos.csv（轴角）
// ============================================================
bool convertEulerCsvToAxisCsv(const std::string& save_dir) {
    std::string euler_file = save_dir + "/rob_eule.csv";
    std::string axis_file = save_dir + "/rob_pos.csv";

    std::ifstream fin(euler_file);
    if (!fin.is_open()) {
        std::cerr << "[calibUtil] 无法打开 " << euler_file << std::endl;
        return false;
    }

    std::ofstream fout(axis_file);
    if (!fout.is_open()) {
        std::cerr << "[calibUtil] 无法创建 " << axis_file << std::endl;
        return false;
    }

    std::string line;
    int count = 0;
    while (std::getline(fin, line)) {
        if (line.empty()) continue;
        double euler[ROB_AXIS_NUM], axis[ROB_AXIS_NUM];
        if (!parseDoubles(line, euler, ROB_AXIS_NUM)) {
            std::cerr << "[calibUtil] rob_eule.csv 行格式错误: " << line << std::endl;
            continue;
        }
        if (cr_Eule2AxisAngle(euler, axis) != sucess) {
            std::cerr << "[calibUtil] 欧拉→轴角转换失败，第 " << (count + 1) << " 行" << std::endl;
            return false;
        }
        fout << std::fixed << std::setprecision(3);
        for (int j = 0; j < ROB_AXIS_NUM; j++)
            fout << axis[j] << (j < ROB_AXIS_NUM - 1 ? "," : "\n");
        count++;
    }

    std::cout << "[calibUtil] 转换完成: " << euler_file << " → " << axis_file
              << " (" << count << " 帧)" << std::endl;
    return count > 0;
}

// ============================================================
// 保存标定结果到 YAML
// ============================================================
bool saveCalibrationResult(const std::string& yaml_path,
                            const std::string& transform_key,
                            const cv::Mat& transform,
                            const cv::Mat& camera_matrix,
                            const cv::Mat& dist_coeffs,
                            double reproj_error_mm,
                            int valid_frames) {
    if (yaml_path.empty() || transform.empty() || camera_matrix.empty() ||
        dist_coeffs.empty()) {
        APP_LOG_EVENT_ERROR("calibration.output",
                            "yaml_output",
                            "fail",
                            {{"reason", "invalid_input"},
                             {"path", QString::fromStdString(yaml_path)},
                             {"transform_empty", transform.empty() ? "true" : "false"},
                             {"camera_empty", camera_matrix.empty() ? "true" : "false"},
                             {"distortion_empty", dist_coeffs.empty() ? "true" : "false"}});
        return false;
    }

    APP_LOG_EVENT_INFO("calibration.output",
                       "yaml_output",
                       "start",
                       {{"path", QString::fromStdString(yaml_path)},
                        {"transform_key", QString::fromStdString(transform_key)},
                        {"valid_frames", QString::number(valid_frames)},
                        {"error_mm", QString::number(reproj_error_mm, 'f', 6)}});
    const std::string temporary_path = yaml_path + ".tmp.yaml";
    try {
        cv::FileStorage out(temporary_path, cv::FileStorage::WRITE);
        if (!out.isOpened()) {
            APP_LOG_EVENT_ERROR("calibration.output",
                                "yaml_output",
                                "fail",
                                {{"path", QString::fromStdString(temporary_path)},
                                 {"reason", "open_failed"}});
            std::cerr << "[calibUtil] 无法写入 " << temporary_path << std::endl;
            return false;
        }
        out << transform_key.c_str() << transform;
        out << "camera_matrix" << camera_matrix;
        out << "distortion_coefficients" << dist_coeffs;
        out << "reprojection_error_mm" << reproj_error_mm;
        out << "valid_frames" << valid_frames;
        out.release();
    } catch (const cv::Exception& e) {
        std::error_code ignored;
        std::filesystem::remove(temporary_path, ignored);
        APP_LOG_EVENT_ERROR("calibration.output",
                            "yaml_output",
                            "fail",
                            {{"path", QString::fromStdString(yaml_path)},
                             {"reason", "opencv_exception"},
                             {"message", QString::fromUtf8(e.what())}});
        std::cerr << "[calibUtil] 写入 YAML 异常: " << e.what() << std::endl;
        return false;
    }

    std::error_code fileError;
    const auto bytes = std::filesystem::file_size(temporary_path, fileError);
    if (fileError || bytes == 0) {
        APP_LOG_EVENT_ERROR("calibration.output",
                            "yaml_output",
                            "fail",
                            {{"path", QString::fromStdString(yaml_path)},
                             {"reason", "file_verification"},
                             {"error", QString::fromStdString(fileError.message())},
                             {"bytes", QString::number(fileError ? 0 : static_cast<qulonglong>(bytes))}});
        return false;
    }
    std::filesystem::rename(temporary_path, yaml_path, fileError);
    if (fileError) {
        APP_LOG_EVENT_ERROR("calibration.output",
                            "yaml_output",
                            "fail",
                            {{"path", QString::fromStdString(yaml_path)},
                             {"reason", "atomic_publish_failed"},
                             {"error", QString::fromStdString(fileError.message())}});
        std::error_code ignored;
        std::filesystem::remove(temporary_path, ignored);
        return false;
    }
    std::cout << "[calibUtil] 结果已保存到: " << yaml_path << std::endl;
    APP_LOG_EVENT_INFO("calibration.output",
                       "yaml_output",
                       "success",
                       {{"path", QString::fromStdString(yaml_path)},
                        {"bytes", QString::number(static_cast<qulonglong>(bytes))},
                        {"valid_frames", QString::number(valid_frames)},
                        {"transform_key", QString::fromStdString(transform_key)}});
    return true;
}
