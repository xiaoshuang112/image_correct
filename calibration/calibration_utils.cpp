#include "calibration_utils.hpp"

#include "robotapi.h"
#include "basestruct.h"
#include "utils/robot_util.hpp"
#include "utils/calib_transform.hpp"

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
    int captured = 0, freeRobotHandle = 0;

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
                 std::cout << "\n[autoCalib] failure to move: " << i << "/" << total << ", waiting for reconnect\n" << std::flush;
                robot_handle = reconnectRobotHandle(robot_handle, pchRobotIP);
                freeRobotHandle = 1;
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
    if(freeRobotHandle)
       cr_destroy_robot(robot_handle);
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
    int captured = 0, freeRobotHandle = 0;

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
            if(operate_timeout == ret_move){
                 std::cout << "\n[autoCalib] failure to move: " << i << "/" << total << ", waiting for reconnect\n" << std::flush;
                robot_handle = reconnectRobotHandle(robot_handle, pchRobotIP);
                freeRobotHandle = 1;
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
    if(freeRobotHandle)
       cr_destroy_robot(robot_handle);
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

    bool on_long_side = (long_is_x && best.is_row && fabs(best.vec.x) >= fabs(best.vec.y))
                           || (!long_is_x && !best.is_row && fabs(best.vec.x) <= fabs(best.vec.y));
    bool ref_is_horiz = (image_width >= image_height) ? on_long_side : !on_long_side;

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
                if (d > 0 || d > 3000) { sum += d; cnt++; }
            }
        }
    }
    return (cnt > 0) ? (sum / cnt) : 0.0;
}

// 将法兰 rx=-180, ry=0 旋转到水平姿态
bool rotateFlangeToHorizon(int robot_handle, double *pFlangeEulePos){
    double flangeEulePos[ROB_AXIS_NUM], tcp_aa[ROB_AXIS_NUM];
    cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
    convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos);
    flangeEulePos[3] = -180.0;
    flangeEulePos[4] = 0.0;

    moveToFlangeEulePos(robot_handle, flangeEulePos);
    if(pFlangeEulePos!= NULL){
      memcpy(pFlangeEulePos, flangeEulePos, sizeof(flangeEulePos));
    }
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

    // 采集一帧（含深度，如果有深度相机）
    cv::Mat depth;
    for (int i = 0; i < 3; i++) {
        camera->captureOneFrameWithDepth(out_frame, depth);
        if (!out_frame.empty() && !depth.empty()) break;
    }
    if (out_frame.empty()) {
        std::cout << "[estimateCameraHeight] 采集图像失败" << std::endl;
        return false;
    }

    if (!detector.detectCorners(out_frame, out_corners, out_ids) || out_ids.size() < 4) {
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
        }
    }

    // 方案 B: 无深度 → 双图像法
    if (!heightOk) {
        std::cout << "[estimateCameraHeight] 无深度数据，双图像法估计高度" << std::endl;

        double initJoint[6];
        cr_get_jointActualPos((RobotHandle)robot_handle, initJoint);
        const double Z_STEP = 150.0;

        double flangeUp[6];
        std::copy(oriFlangeEulePos, oriFlangeEulePos + 6, flangeUp);
        flangeUp[2] = oriFlangeEulePos[2] + Z_STEP;
        moveToFlangeEulePos(robot_handle, flangeUp);

        cv::Mat frame2 = camera->captureOneFrame(1500);
        if (frame2.empty()) { moveToJointPos(robot_handle, initJoint); return false; }

        std::vector<cv::Point2f> corners2;
        std::vector<int> ids2;
        if (!detector.detectCorners(frame2, corners2, ids2) || ids2.size() < 4) {
            moveToJointPos(robot_handle, initJoint);
            return false;
        }
        moveToJointPos(robot_handle, initJoint);

        float dim1, dim2;
        if (!computeCommonSpan(out_corners, out_ids, corners2, ids2, dim1, dim2)) {
            std::cout << "[estimateCameraHeight] 两帧共同角点不足" << std::endl;
            return false;
        }

        if (dim1 > dim2 && (dim1 - dim2) > 0.5f) {
            heightCamera = Z_STEP * dim2 / (dim1 - dim2);
            std::cout << "[autoCalib] 双图像高度估计: " << heightCamera
                      << " mm (像素 " << dim1 << "→" << dim2 << " px)" << std::endl;
            heightOk = true;
        }
    }

    if (!heightOk) {
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
    cv::FileStorage out(yaml_path, cv::FileStorage::WRITE);
    if (!out.isOpened()) {
        std::cerr << "[calibUtil] 无法写入 " << yaml_path << std::endl;
        return false;
    }
    out << transform_key.c_str() << transform;
    out << "camera_matrix" << camera_matrix;
    out << "distortion_coefficients" << dist_coeffs;
    out << "reprojection_error_mm" << reproj_error_mm;
    out << "valid_frames" << valid_frames;
    out.release();
    std::cout << "[calibUtil] 结果已保存到: " << yaml_path << std::endl;
    return true;
}
