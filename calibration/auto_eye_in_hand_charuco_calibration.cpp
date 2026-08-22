#include "auto_eye_in_hand_charuco_calibration.hpp"

#include "cameras/AICamera.hpp"
#ifdef useORB
#include "cameras/OrbbecCamera.hpp"
#endif // useORB

#ifdef useOAK
#include "cameras/OAKCamera.hpp"
#endif // useOAK

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
#include "semi_auto_eye_in_hand_calibration.hpp"

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

// ============================================================
// 评价函数 + XY 搜索（共用）
// ============================================================

// 评价当前位姿检测质量
// use_center=true:  score = ids.size() * 10000 - 中心距像素
// use_center=false: score = ids.size() * 1000000 + 凸包面积
static std::tuple<double, int, cv::Point2f> evaluatePose(
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    bool use_center) {
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
    // if (use_center) {
    //     score -= pd;
    // } else {
    double hull_area = cv::contourArea(h);
    //double    score = (double)i.size() * 1000000.0 + hull_area - pd - fabs(angle_diff) * 100;
    double    score = (double)i.size() * 100000.0 + hull_area - pd - fabs(angle_diff) * 100;
    // }
    return {score, (int)i.size(), bc};
}



// 尝试一组候选欧拉位姿，选评分最高的
// candidates: 欧拉位姿列表 [x,y,z,rx,ry,rz] (mm, deg)
// use_center: 评分模式 (true=中心距, false=面积)
// best_joint: 输出，最佳候选的关节角
// 返回最佳评分（运动或基础设施失败时返回负无穷，调用方必须立即终止）
// 注意：函数会临时运动到候选位姿做评估，返回前会回到原始位置
static constexpr double kCandidateSearchFailure =
    -std::numeric_limits<double>::infinity();

static double tryCandidatePoses(int robot_handle,
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    bool use_center,
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
        std::tie(s, n, bc) = evaluatePose(camera, detector, use_center);
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
        std::tie(score, n, bc) = evaluatePose(camera, detector, false);
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
            robot_handle, camera, detector, false, "coarse_xy",
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

    const double TARGET_AREA_RATIO = 0.275;
    const int MAX_REFINE_ZRZ = 6;
    const double Z_STEP = 100.0;
    const double Z_MAX_OFFSET = 600.0;
    const double Z_MAX_APPROACH_OFFSET = 200.0;

    // ---- 固定 rx=-180°, ry=0°（标定板水平），保留 rz ----
    double flangeEulePos[ROB_AXIS_NUM] = {};
    double tcp_aa[ROB_AXIS_NUM] = {};
    if (!rotateFlangeToHorizon(robot_handle, flangeEulePos)) {
        APP_LOG_EVENT_ERROR("calibration.pose_search",
                            "best_pose",
                            "fail",
                            {{"phase", "initial_horizontal_rotation"},
                             {"reason", "rotate_flange_failed"}});
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
    std::tie(cur_score, cur_codes, cur_bc) = evaluatePose(camera, detector, false);

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
            robot_handle, camera, detector, false, "coarse_z_approach",
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
                robot_handle, camera, detector, false, "coarse_z_retreat",
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
    frame = camera->captureOneFrame(1500);
    // ================================================================
    // Phase 2: XY 搜索 — 码不全时找更好的水平位置
    // ================================================================
    
    std::tie(cur_score, cur_codes, cur_bc) = evaluatePose(camera, detector, false);

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
        double Z_desired = flangeEulePos[2] * std::sqrt(area_ratio / TARGET_AREA_RATIO);
        double dZ =  (Z_desired - flangeEulePos[2]); 
        double dRz_deg = -computeBoardAngle(corners, ids, board_sz.width, board_sz.height, frame.cols, frame.rows) * 180.0 / CV_PI;

        if (std::abs(dZ) < 30.0 && std::abs(dRz_deg) < 1.0) {
            std::cout << "[autoFindBestPose] Z+rz 收敛, iter=" << iter << std::endl;
            break;
        }

        // 在 0.2~1.0 比例下尝试 Z+rz 组合，选评分最高的
        double cur_s; int n; cv::Point2f bc;
        std::tie(cur_s, n, bc) = evaluatePose(camera, detector, false);

        std::vector<std::array<double, 6>> zrz_candidates;
        for (int fracStep = 1; fracStep <= 2; ++fracStep) {
            const double frac = fracStep * 0.5;
            // 保持 Rz 与当前位姿连续；强行限制到 [-90, 90] 会制造 180° 的 J6 跳变。
            zrz_candidates.push_back({flangeEulePos[0], flangeEulePos[1], flangeEulePos[2] + dZ * frac,
                                      flangeEulePos[3], flangeEulePos[4], flangeEulePos[5] + dRz_deg * frac});
        }

        double bestFlangeEulePos[ROB_AXIS_NUM];
        const double candidateScore = tryCandidatePoses(
            robot_handle, camera, detector, false, "refine_zrz",
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

// 旧方法：基于比例迭代的 Z 值搜索（替代上面的 IK 搜索方法）
// 移动机器人到不同高度，根据标定板在图像中的占比自适应决定 Z 值
static bool computeZValues_old(int robot_handle,
    std::shared_ptr<AICamera>& camera,
    int cameraToTcpEnd,
    CharucoDetector& detector,
    double z_out_[3]) {
    double tcp_aa[ROB_AXIS_NUM], tcpEulePos[ROB_AXIS_NUM], oriFlangeEulePos[ROB_AXIS_NUM], outZ[3];
    cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
    convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, oriFlangeEulePos);
    cr_AxisAngle2Eule(tcp_aa, tcpEulePos);
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

    // 标定板在图像中的尺寸
    float board_dim = computePixelSpan(corners);
    if (board_dim < 1.0f) return false;

    // 最小安全高度：离标定板至少 30cm
    double minH = z_board + 300.0;

    std::cout << "[autoCalib_old] z_board=" << z_board
              << ", heightCamera=" << heightCamera
              << ", board_dim=" << board_dim << " px" << std::endl;

    // 目标图像占比（标定板像素跨度 / 图像短边）
    // 占比小 → 相机离得远；占比大 → 相机离得近
    const double ratios[] = {0.05, 0.1, 0.30, 0.5,  0.2, 0.4, 0.6};
    double z_vals[9];
    const double IMG_SHORT = std::min(frame.cols, frame.rows);
    bool validZ[9] = {false};
    double initJoint[ROB_AXIS_NUM];
    cr_get_jointActualPos((RobotHandle)robot_handle, initJoint);

    for (int ri = 0; ri < 9; ri++) {
        double ratio = ratios[ri];
        double target_px = IMG_SHORT * ratio;    // 期望的像素跨度
        double z_est = z_board + heightCamera * (board_dim / target_px);

        // 迭代收敛到目标比例
        for (int iter = 0; iter < 20; iter++) {
            if (z_est < minH) {
                z_est = minH;
            }

            double flangeEule[ROB_AXIS_NUM];
            std::copy(oriFlangeEulePos, oriFlangeEulePos + ROB_AXIS_NUM, flangeEule);
            flangeEule[2] = z_est;

            if (!moveToFlangeEulePos(robot_handle, flangeEule)) {
                break;
            }

            cv::Mat f = camera->captureOneFrame(1500);
            if (f.empty()) break;

            std::vector<cv::Point2f> pts;
            std::vector<int> ids2;
            if (!detector.detectCorners(f, pts, ids2) || ids2.size() < 4) {
                break;
            }

            float dim = computePixelSpan(pts);
            if (dim < 1.0f) break;

            double actual_ratio = dim / IMG_SHORT;
            double err = actual_ratio - ratio;

            std::cout << "[autoCalib_old] ratio[" << ri << "] iter=" << iter
                      << " z=" << z_est << " actual_ratio=" << actual_ratio
                      << " err=" << err << std::endl;

            if (fabs(err) < 0.03) {
                z_vals[ri] = z_est;
                validZ[ri] = true;
                break;
            }

            // 自适应调整 Z
            double adjust = z_est * err / (ratio * 1.5);
            z_est -= adjust;
        }
    }

    // 回到初始位置
    moveToJointPos(robot_handle, initJoint);

    // 从有效 Z 值中选择 3 个：尽量覆盖高、中、低三个范围
    std::vector<double> sorted;
    for (int i = 0; i < 9; i++) {
        if (validZ[i]) sorted.push_back(z_vals[i]);
    }
    if (sorted.empty()) {
        std::cout << "[autoCalib_old] 未收敛到任何 Z 值" << std::endl;
        return false;
    }
    std::sort(sorted.begin(), sorted.end());

    outZ[0] = sorted.back();   // 最高
    outZ[2] = sorted.front();  // 最低
    if (sorted.size() >= 3)
        outZ[1] = sorted[sorted.size() / 2];
    else
        outZ[1] = (outZ[0] + outZ[2]) / 2.0;

    z_out_[0] = outZ[0];
    z_out_[1] = outZ[1];
    z_out_[2] = outZ[2];

    std::cout << "[autoCalib_old] Z 值: high=" << outZ[0]
              << ", mid=" << outZ[1]
              << ", low=" << outZ[2] << " mm" << std::endl;
    return true;
}

// ============================================================
// Phase 3: 生成约 360 个高覆盖候选 + IK 过滤
// ============================================================

static double jointPathDistance(const double lhs[ROB_AXIS_NUM],
                                const double rhs[ROB_AXIS_NUM]) {
    double sum = 0.0;
    for (int axis = 0; axis < ROB_AXIS_NUM; ++axis) {
        double delta = rhs[axis] - lhs[axis];
        if (axis == 5)
            delta = std::remainder(delta, 360.0);
        sum += delta * delta;
    }
    return std::sqrt(sum);
}

static void orderCandidatesByNearestJoint(std::vector<PoseCandidate>& candidates,
                                          const double initialJoint[ROB_AXIS_NUM]) {
    std::vector<PoseCandidate> ordered;
    ordered.reserve(candidates.size());
    std::vector<bool> used(candidates.size(), false);
    double current[ROB_AXIS_NUM];
    std::copy(initialJoint, initialJoint + ROB_AXIS_NUM, current);
    double pathDegrees = 0.0;
    double maximumStepDegrees = 0.0;
    double maximumJ6StepDegrees = 0.0;

    for (size_t count = 0; count < candidates.size(); ++count) {
        size_t bestIndex = candidates.size();
        double bestDistance = std::numeric_limits<double>::infinity();
        for (size_t index = 0; index < candidates.size(); ++index) {
            if (used[index])
                continue;
            const double distance = jointPathDistance(current, candidates[index].joint);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestIndex = index;
            }
        }
        if (bestIndex == candidates.size())
            break;

        PoseCandidate candidate = candidates[bestIndex];
        candidate.joint[5] += std::round((current[5] - candidate.joint[5]) / 360.0) * 360.0;
        const double j6Step = std::abs(candidate.joint[5] - current[5]);
        pathDegrees += bestDistance;
        maximumStepDegrees = std::max(maximumStepDegrees, bestDistance);
        maximumJ6StepDegrees = std::max(maximumJ6StepDegrees, j6Step);
        std::copy(candidate.joint, candidate.joint + ROB_AXIS_NUM, current);
        ordered.push_back(candidate);
        used[bestIndex] = true;
    }

    candidates.swap(ordered);
    APP_LOG_EVENT_INFO("calibration.candidates",
                       "candidate_ordering",
                       "success",
                       {{"strategy", "nearest_joint_greedy"},
                        {"candidates", QString::number(candidates.size())},
                        {"estimated_path_deg", QString::number(pathDegrees, 'f', 3)},
                        {"max_step_deg", QString::number(maximumStepDegrees, 'f', 3)},
                        {"max_j6_step_deg", QString::number(maximumJ6StepDegrees, 'f', 3)}});
}

bool generateCandidates(int robot_handle,
                        std::shared_ptr<AICamera>& camera,
                        const double z_values[3],
                        std::vector<PoseCandidate>& candidates,
                        int* planned_count = nullptr) {
    double euler[ROB_AXIS_NUM] = {};
    double tcp_aa[ROB_AXIS_NUM] = {};
    double flangeEulePos[ROB_AXIS_NUM] = {};
    double joint[ROB_AXIS_NUM] = {};
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

                if (convertFlangeOulePos2Joint(robot_handle, euler, joint)) {
                    PoseCandidate candidate{};
                    std::copy(joint, joint + ROB_AXIS_NUM, candidate.joint);
                    candidates.push_back(candidate);
                    ++ik_ok;
                }
            }
        }
    }

    orderCandidatesByNearestJoint(candidates, initialJoint);
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
                        {"ordered", QString::number(candidates.size())},
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
            : camera_config.cam_ip + ":" + std::to_string(camera_config.cam_port);
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
    const bool has_provided_intrinsics =
        !camera_matrix.empty() && !dist_coeffs.empty();
    session_options.intrinsic_source = has_provided_intrinsics
        ? "provided_runtime" : "calibrated_from_collection";
    session_options.intrinsic_source_file = intrinsic_source_file;
    if (has_provided_intrinsics) {
        session_options.camera_matrix = camera_matrix.getMat().clone();
        session_options.dist_coeffs = dist_coeffs.getMat().clone();
    }
    std::string session_error;
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

    g_save_dir = session->previewDirectory();
    g_callback = callback;

    std::cout << "\n========== 自动手眼标定开始 ==========" << std::endl;

    CharucoDetector detector(board_config);
    if (has_provided_intrinsics) {
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
    if (!rotateFlangeToHorizon(robot_handle, nullptr)) {
        result.message = "机器人水平姿态调整失败";
        APP_LOG_EVENT_ERROR("calibration",
                            "pipeline",
                            "fail",
                            {{"reason", "initial_rotation_failed"},
                             {"stage", "prepare"}});
        return failSession("prepare", "initial_rotation_failed");
    }
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
    if (!executeCollection(robot_handle, pchRobotIP, camera, detector,
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
    if (!has_provided_intrinsics) {
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
    session->setIntrinsics(K, D, has_provided_intrinsics
        ? "provided_runtime" : "calibrated_from_collection");

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
