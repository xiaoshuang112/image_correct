#include "calibration_utils.hpp"

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
                       AI_det_callback callback) {
    int total = (int)candidates.size();
    int captured = 0;

    APP_LOG_INFO("calibration.collection", QString("collection_begin handle=%1 candidates=%2 save_dir=%3")
        .arg(robot_handle).arg(total).arg(QString::fromStdString(save_dir)));

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
        CRresult ret_move = moveToJointPos(robot_handle, candidates[i].joint);
        if(sucess != ret_move){
            APP_LOG_WARNING("calibration.collection", QString("pose_move_failed index=%1 total=%2 error=%3")
                .arg(i).arg(total).arg(ret_move));
            if(operate_timeout == ret_move){
                APP_LOG_ERROR("calibration.collection", QString("collection_aborted reason=motion_timeout index=%1 captured=%2")
                    .arg(i).arg(captured));
                std::cerr << "\n[autoCalib] 机器人运动超时，终止本轮标定，请重新连接机器人"
                          << std::endl;
                return false;
            }
            continue;
        }

        cv::Mat frame = camera->captureOneFrame(1500);
        if (frame.empty()) {
            APP_LOG_WARNING("calibration.collection", QString("pose_image_capture_failed index=%1 captured=%2")
                .arg(i).arg(captured));
            std::cerr << "\n[autoCalib] 采集图像失败 (pose " << i << ")" << std::endl;
            continue;
        }

        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detector.detectCorners(frame, corners, ids)) {
            APP_LOG_WARNING("calibration.collection", QString("pose_detection_failed index=%1 image=%2x%3")
                .arg(i).arg(frame.cols).arg(frame.rows));
            continue;
        }
        if(ids.size() < 12) {
            APP_LOG_WARNING("calibration.collection", QString("pose_rejected index=%1 reason=insufficient_corners corners=%2 minimum=12")
                .arg(i).arg(ids.size()));
            continue;
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
            APP_LOG_ERROR("calibration.collection", QString("pose_joint_read_failed index=%1 error=%2")
                .arg(i).arg(poseRet));
            continue;
        }
        poseRet = cr_kineForward(robot_handle, actual_joint, flange_pose);
        if (poseRet != sucess) {
            APP_LOG_ERROR("calibration.collection", QString("pose_forward_kinematics_failed index=%1 error=%2")
                .arg(i).arg(poseRet));
            continue;
        }

        // 保存到磁盘
        {
            std::string img_name = save_dir + "/" +
                (captured < 10 ? "00" : captured < 100 ? "0" : "") +
                std::to_string(captured) + ".jpg";
            if (!cv::imwrite(img_name, frame)) {
                APP_LOG_ERROR("calibration.collection", QString("pose_image_write_failed index=%1 path=%2")
                    .arg(i).arg(QString::fromStdString(img_name)));
                continue;
            }
            if (callback) {
                int call_ret = callback((char*)img_name.c_str(), (int)img_name.length(), progress);
                if (-1==call_ret)
                {
                    std::cerr << "\r[autoCalib] 用户取消了  采集进度"  << std::endl;
                    APP_LOG_WARNING("calibration.collection", QString("collection_cancelled index=%1 captured=%2")
                        .arg(i).arg(captured));
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
        APP_LOG_DEBUG("calibration.collection", QString("pose_captured index=%1 captured=%2 total=%3 corners=%4")
            .arg(i).arg(captured).arg(total).arg(ids.size()));
    }

    std::cout << "\n[autoCalib] 采集完成。有效帧: " << captured
              << "/" << total
              << ", 数据保存到: " << save_dir << std::endl;
    APP_LOG_INFO("calibration.collection", QString("collection_complete captured=%1 total=%2 save_dir=%3")
        .arg(captured).arg(total).arg(QString::fromStdString(save_dir)));
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
        APP_LOG_ERROR("calibration.robot", QString("rotate_horizontal_tcp_read_failed handle=%1 error=%2")
            .arg(robot_handle).arg(ret));
        return false;
    }
    if (!convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos)) {
        APP_LOG_ERROR("calibration.robot", QString("rotate_horizontal_tcp_to_flange_failed handle=%1")
            .arg(robot_handle));
        return false;
    }
    flangeEulePos[3] = -180.0;
    flangeEulePos[4] = 0.0;

    APP_LOG_INFO("calibration.robot", QString("rotate_horizontal_begin handle=%1 target=[%2,%3,%4,%5,%6,%7]")
        .arg(robot_handle)
        .arg(flangeEulePos[0], 0, 'f', 3).arg(flangeEulePos[1], 0, 'f', 3)
        .arg(flangeEulePos[2], 0, 'f', 3).arg(flangeEulePos[3], 0, 'f', 3)
        .arg(flangeEulePos[4], 0, 'f', 3).arg(flangeEulePos[5], 0, 'f', 3));
    if (!moveToFlangeEulePos(robot_handle, flangeEulePos)) {
        APP_LOG_ERROR("calibration.robot", QString("rotate_horizontal_move_failed handle=%1")
            .arg(robot_handle));
        return false;
    }
    if(pFlangeEulePos!= NULL){
      memcpy(pFlangeEulePos, flangeEulePos, sizeof(flangeEulePos));
    }
    APP_LOG_INFO("calibration.robot", QString("rotate_horizontal_complete handle=%1")
        .arg(robot_handle));
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
        APP_LOG_ERROR("calibration.height", "height_estimation_failed reason=null_camera");
        return false;
    }

    // 采集一帧（含深度，如果有深度相机）
    cv::Mat depth;
    for (int i = 0; i < 3; i++) {
        const int captureRet = camera->captureOneFrameWithDepth(out_frame, depth);
        APP_LOG_DEBUG("calibration.height", QString("height_capture attempt=%1 result=%2 color=%3x%4 depth=%5x%6")
            .arg(i + 1).arg(captureRet)
            .arg(out_frame.cols).arg(out_frame.rows)
            .arg(depth.cols).arg(depth.rows));
        if (captureRet != 0 && out_frame.empty())
            continue;
        if (!out_frame.empty() && !depth.empty()) break;
    }
    if (out_frame.empty()) {
        APP_LOG_ERROR("calibration.height", "height_estimation_failed reason=color_capture_failed attempts=3");
        std::cout << "[estimateCameraHeight] 采集图像失败" << std::endl;
        return false;
    }

    if (!detector.detectCorners(out_frame, out_corners, out_ids) || out_ids.size() < 4) {
        APP_LOG_WARNING("calibration.height", QString("height_estimation_failed reason=insufficient_corners corners=%1 image=%2x%3")
            .arg(out_ids.size()).arg(out_frame.cols).arg(out_frame.rows));
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
            APP_LOG_INFO("calibration.height", QString("height_estimated method=depth height_mm=%1 corners=%2 depth=%3x%4")
                .arg(heightCamera, 0, 'f', 3).arg(out_ids.size())
                .arg(depth.cols).arg(depth.rows));
        } else {
            APP_LOG_WARNING("calibration.height", QString("depth_height_invalid corners=%1 depth=%2x%3 valid_range_mm=1..3000")
                .arg(out_ids.size()).arg(depth.cols).arg(depth.rows));
        }
    }

    // 方案 B: 无深度 → 双图像法
    if (!heightOk) {
        std::cout << "[estimateCameraHeight] 无深度数据，双图像法估计高度" << std::endl;

        double initJoint[6] = {};
        CRresult jointRet = cr_get_jointActualPos((RobotHandle)robot_handle, initJoint);
        if (jointRet != sucess) {
            APP_LOG_ERROR("calibration.height", QString("height_fallback_failed reason=joint_read error=%1")
                .arg(jointRet));
            return false;
        }
        const double Z_STEP = 150.0;

        double flangeUp[6];
        std::copy(oriFlangeEulePos, oriFlangeEulePos + 6, flangeUp);
        flangeUp[2] = oriFlangeEulePos[2] + Z_STEP;
        if (!moveToFlangeEulePos(robot_handle, flangeUp)) {
            APP_LOG_ERROR("calibration.height", QString("height_fallback_failed reason=move_up target_z=%1")
                .arg(flangeUp[2], 0, 'f', 3));
            return false;
        }

        cv::Mat frame2 = camera->captureOneFrame(1500);
        if (frame2.empty()) {
            const CRresult restoreRet = moveToJointPos(robot_handle, initJoint);
            if (restoreRet != sucess)
                std::cerr << "[estimateCameraHeight] 第二帧采集失败且恢复位置失败 ret="
                          << restoreRet << std::endl;
            APP_LOG_ERROR("calibration.height", QString("height_fallback_failed reason=second_capture restore_error=%1")
                .arg(restoreRet));
            return false;
        }

        std::vector<cv::Point2f> corners2;
        std::vector<int> ids2;
        if (!detector.detectCorners(frame2, corners2, ids2) || ids2.size() < 4) {
            const CRresult restoreRet = moveToJointPos(robot_handle, initJoint);
            if (restoreRet != sucess)
                std::cerr << "[estimateCameraHeight] 第二帧检测失败且恢复位置失败 ret="
                          << restoreRet << std::endl;
            APP_LOG_WARNING("calibration.height", QString("height_fallback_failed reason=second_detection corners=%1 restore_error=%2")
                .arg(ids2.size()).arg(restoreRet));
            return false;
        }
        const CRresult restoreRet = moveToJointPos(robot_handle, initJoint);
        if (restoreRet != sucess) {
            APP_LOG_ERROR("calibration.height", QString("height_fallback_failed reason=restore_position error=%1")
                .arg(restoreRet));
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
            APP_LOG_INFO("calibration.height", QString("height_estimated method=two_images height_mm=%1 span_before=%2 span_after=%3")
                .arg(heightCamera, 0, 'f', 3).arg(dim1).arg(dim2));
            heightOk = true;
        }
    }

    if (!heightOk) {
        APP_LOG_ERROR("calibration.height", "height_estimation_failed reason=no_valid_method");
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
        std::stringstream ss(line);
        double vals[6];
        for (int i = 0; i < 6; i++) {
            std::string token;
            if (!std::getline(ss, token, ',')) break;
            vals[i] = std::stod(token);
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
        std::stringstream ss(line);
        double euler[ROB_AXIS_NUM], axis[ROB_AXIS_NUM];
        for (int i = 0; i < ROB_AXIS_NUM; i++) {
            std::string token;
            if (!std::getline(ss, token, ',')) break;
            euler[i] = std::stod(token);
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
        APP_LOG_ERROR("calibration.output", QString("yaml_write_rejected path=%1 transform_empty=%2 camera_empty=%3 distortion_empty=%4")
            .arg(QString::fromStdString(yaml_path)).arg(transform.empty())
            .arg(camera_matrix.empty()).arg(dist_coeffs.empty()));
        return false;
    }

    APP_LOG_INFO("calibration.output", QString("yaml_write_begin path=%1 transform_key=%2 valid_frames=%3 error_mm=%4")
        .arg(QString::fromStdString(yaml_path), QString::fromStdString(transform_key))
        .arg(valid_frames).arg(reproj_error_mm, 0, 'f', 6));
    try {
        cv::FileStorage out(yaml_path, cv::FileStorage::WRITE);
        if (!out.isOpened()) {
            APP_LOG_ERROR("calibration.output", QString("yaml_write_failed path=%1 reason=open_failed")
                .arg(QString::fromStdString(yaml_path)));
            std::cerr << "[calibUtil] 无法写入 " << yaml_path << std::endl;
            return false;
        }
        out << transform_key.c_str() << transform;
        out << "camera_matrix" << camera_matrix;
        out << "distortion_coefficients" << dist_coeffs;
        out << "reprojection_error_mm" << reproj_error_mm;
        out << "valid_frames" << valid_frames;
        out.release();
    } catch (const cv::Exception& e) {
        APP_LOG_ERROR("calibration.output", QString("yaml_write_failed path=%1 reason=opencv_exception message=%2")
            .arg(QString::fromStdString(yaml_path), QString::fromUtf8(e.what())));
        std::cerr << "[calibUtil] 写入 YAML 异常: " << e.what() << std::endl;
        return false;
    }

    std::error_code fileError;
    const auto bytes = std::filesystem::file_size(yaml_path, fileError);
    if (fileError || bytes == 0) {
        APP_LOG_ERROR("calibration.output", QString("yaml_write_failed path=%1 reason=file_verification error=%2 bytes=%3")
            .arg(QString::fromStdString(yaml_path), QString::fromStdString(fileError.message()))
            .arg(fileError ? 0 : static_cast<qulonglong>(bytes)));
        return false;
    }
    std::cout << "[calibUtil] 结果已保存到: " << yaml_path << std::endl;
    APP_LOG_INFO("calibration.output", QString("yaml_write_complete path=%1 bytes=%2")
        .arg(QString::fromStdString(yaml_path)).arg(static_cast<qulonglong>(bytes)));
    return true;
}
