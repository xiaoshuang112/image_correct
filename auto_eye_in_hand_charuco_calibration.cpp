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

#include "utils/robot_util.hpp"


std::string g_save_dir = "";
AI_det_callback g_callback = nullptr;

// ============================================================
// 内部数据结构
// ============================================================

struct PoseCandidate {
    double axis_angle[6];   // TCP 轴角位姿 (mm, deg)
    double joint[6];        // 关节角 (deg)
};

struct CapturedFrame {
    cv::Mat T_base_flange;              // 4x4 机器人法兰位姿
    std::vector<cv::Point2f> corners;   // ChArUco 角点（像素）
    std::vector<int> ids;               // ChArUco 角点 ID
    cv::Mat image;                      // 原始图像（用于重检测）
};

// ============================================================
// 内部工具函数
// ============================================================




namespace {

// 前向声明（visual servoing 函数需要）
cv::Mat axisAnglePoseToTransform(const double pose[6]);

// 前向声明（computeAvgDepth 定义在后面，estimateCameraHeight 需要）
static double computeAvgDepth(const cv::Mat& depth, const std::vector<cv::Point2f>& corners);



// ============================================================
// 视觉伺服：公式直接反算最佳位姿
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

// 轴角位姿 → 4x4 变换矩阵 (已有，但需要非匿名可见)
// 用已有的 axisAnglePoseToTransform

// 4x4 → 轴角位姿 [x,y,z,rx,ry,rz] (mm, deg)
void transformToAxisAnglePose(const cv::Mat& T, double pose[6]) {
    pose[0] = T.at<double>(0, 3) * 1000.0;  // m → mm
    pose[1] = T.at<double>(1, 3) * 1000.0;
    pose[2] = T.at<double>(2, 3) * 1000.0;

    cv::Mat R = T(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    pose[3] = rvec.at<double>(0) * 180.0 / CV_PI;  // rad → deg
    pose[4] = rvec.at<double>(1) * 180.0 / CV_PI;
    pose[5] = rvec.at<double>(2) * 180.0 / CV_PI;
}

// 计算角点凸包在两个方向上的像素跨度（用于基于像素变化估计距离）
// 返回最大方向跨度; out_w/out_h 为可选输出（宽度、高度）
static float computePixelSpan(const std::vector<cv::Point2f>& corners,
                              float* out_w = nullptr,
                              float* out_h = nullptr) {
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
static bool computeCommonSpan(const std::vector<cv::Point2f>& corners1,
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

// 估计相机到标定板的距离（深度相机优先，无深度时自动回退到双图像法）
// 同时输出采集到的 frame 和角点（后续 Z 值迭代需要）
static bool estimateCameraHeight(
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
// 评价函数 + XY 搜索（共用）
// ============================================================

// 评价当前位姿检测质量
// use_center=true:  score = ids.size() * 10000 - 中心距像素
// use_center=false: score = ids.size() * 1000000 + 凸包面积
static std::tuple<double, int, cv::Point2f> evaluatePose(
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    bool use_center) {
    cv::Mat f = camera->captureOneFrame(1500);
    if (f.empty()) return {0, 0, cv::Point2f(0, 0)};

        if (nullptr!=g_callback) {
            auto now = std::chrono::high_resolution_clock::now();
            auto duration = now.time_since_epoch();
            auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();

            std::string file_id = std::to_string(nanoseconds);

            std::string img_name = g_save_dir + "/" +  file_id  + ".jpg";
            cv::imwrite(img_name, f);

            int call_ret = g_callback((char*)img_name.c_str(), (int)img_name.length(), 1);

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
// 返回最佳评分 (全失败返回 -1)
// 注意：函数会临时运动到候选位姿做评估，返回前会回到原始位置
static double tryCandidatePoses(int robot_handle,
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    bool use_center,
    const std::vector<std::array<double, 6>>& candidates,
    double bestFlangeEule[6]) {
    // 保存原始关节角
    double initJointPos[ROB_AXIS_NUM], tcp_aa[ROB_AXIS_NUM];
    if (cr_get_jointActualPos((RobotHandle)robot_handle, initJointPos) != sucess)
        return -1.0;
    if(cr_get_tcpActualPose(robot_handle, tcp_aa)!= sucess)
       return -1.0;
    
    double bestFlangeEule_j[ROB_AXIS_NUM];
    double best_s = -1e18;
    bool found = false;
    convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, bestFlangeEule_j);
    for (const auto& euler : candidates) {
             
        if(moveToFlangeEulePos(robot_handle, (double*)euler.data())!= true)
            continue;        

        double s; int n; cv::Point2f bc;
        std::tie(s, n, bc) = evaluatePose(camera, detector, use_center);
        if (s > best_s) {
            best_s = s;
            std::copy(euler.data(), euler.data() + ROB_AXIS_NUM, bestFlangeEule_j);
            found = true;
        }

        if (nullptr!=g_callback) {
            int call_ret = g_callback("", 0, 1);
            if (-1==call_ret)
            {
                std::cerr << "\r[autoCalib] 用户取消了  采集进度"  << std::endl;

                return -1.0;
            }            
        }    

    }

    // 回到原始位置（由调用方决定是否运动到最佳位姿）
    moveToJointPos(robot_handle, initJointPos);   
    if (!found){          
        return -1.0;}

    std::copy(bestFlangeEule_j, bestFlangeEule_j + ROB_AXIS_NUM, bestFlangeEule);
    return best_s;
}

// 多步长 XY 搜索（Phase 2 / Phase 4 共用）
static void searchXYSteps(int robot_handle,
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    const double* steps, int num_steps,
    double tcp_aa[6]) {
    cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);

    double flangeEulePos[ROB_AXIS_NUM];
    convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos);

    for (int i = 0; i < num_steps; i++) {
        double step = steps[i];
        double score; int n; cv::Point2f bc;
        std::tie(score, n, bc) = evaluatePose(camera, detector, false);
        std::vector<std::array<double, 6>> xy_candidates;
        for (int sx : {-1, 0, 1}) for (int sy : {-1, 0, 1}) {
            if (sx == 0 && sy == 0)
               continue;
            xy_candidates.push_back({flangeEulePos[0] + sx*step, flangeEulePos[1] + sy*step, flangeEulePos[2],
                                      flangeEulePos[3], flangeEulePos[4], flangeEulePos[5]});
        }
        double bestFlangeEule[ROB_AXIS_NUM];
        if (tryCandidatePoses(robot_handle, camera, detector,
                               false, xy_candidates, bestFlangeEule) > score) {
            moveToFlangeEulePos(robot_handle, bestFlangeEule);  
            //memcpy(flangeEulePos, bestFlangeEule, sizeof(bestFlangeEule));                    
            cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);   
            convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos);         
        }

        if (nullptr!=g_callback) {
            int call_ret = g_callback("", 0, 1);
            if (-1==call_ret)
            {
                std::cerr << "\r[autoCalib] 用户取消了  采集进度"  << std::endl;

                return;
            }            
        }           

    }
    moveToTcpAxisPos(robot_handle, tcp_aa);    
}

float bound_camera_angle(float angle){
    if(angle > 90)
       return angle - 180;
    if(angle < -90)
       return 180 + angle;
    return angle;
}

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

// 一次公式反算找最佳位姿：板居中、面积 25-30%、长边平行
bool autoFindBestPose(int robot_handle,
                      std::shared_ptr<AICamera>& camera,
                      CharucoDetector& detector,
                      const std::string& save_dir,
                      AI_det_callback callback) {

    const double TARGET_AREA_RATIO = 0.275;
    const int MAX_REFINE_ZRZ = 6;
    const double Z_STEP = 100.0;
    const double Z_MAX_OFFSET = 600.0;

    // ---- 固定 rx=-180°, ry=0°（标定板水平），保留 rz ----
    double flangeEulePos[ROB_AXIS_NUM], tcp_aa[ROB_AXIS_NUM];
    rotateFlangeToHorizon(robot_handle, flangeEulePos);
    cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);

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

    //if (cur_codes > 0 && cur_codes < total_corners) 
    {
        std::cout << "[autoFindBestPose] Phase1: Z 搜索 (当前码=" << cur_codes
                  << "/" << total_corners << ")" << std::endl;

        std::vector<std::array<double, 6>> z_candidates;
        for (int idx = 1; idx * Z_STEP <= Z_MAX_OFFSET; idx++)
            z_candidates.push_back({flangeEulePos[0], flangeEulePos[1], flangeEulePos[2] + idx * Z_STEP,
                                    flangeEulePos[3], flangeEulePos[4], flangeEulePos[5]});

        double bestFlangeEule[ROB_AXIS_NUM];
        double best_s = tryCandidatePoses(robot_handle, camera, detector,
                                           false, z_candidates, bestFlangeEule);
        if (best_s > cur_score) {
            moveToFlangeEulePos(robot_handle, bestFlangeEule);            
            cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);          
            std::cout << "[autoFindBestPose] Phase1 选定 Z=" << tcp_aa[2] << std::endl;
        }
        moveToTcpAxisPos(robot_handle, tcp_aa);        
    }
    frame = camera->captureOneFrame(1500);
    // ================================================================
    // Phase 2: XY 搜索 — 码不全时找更好的水平位置
    // ================================================================
    
    std::tie(cur_score, cur_codes, cur_bc) = evaluatePose(camera, detector, false);

    //if (cur_codes > 0 && cur_codes < total_corners)
     {
        std::cout << "[autoFindBestPose] Phase2: XY 搜索 (码=" << cur_codes
                  << "/" << total_corners << ")" << std::endl;
        const double steps[] = {240, 120, 60};
        searchXYSteps(robot_handle, camera, detector,
                       steps, std::size(steps), tcp_aa);
    }
    frame = camera->captureOneFrame(1500);

    // ================================================================
    // Phase 3: Z+rz 拟合（不动 XY）
    // ================================================================    
    for (int iter = 0; iter < MAX_REFINE_ZRZ; iter++) {
        cv::Mat frame = camera->captureOneFrame(1500);
        if (frame.empty()) {
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
                std::cerr << "[autoFindBestPose] 初始帧检测失败" << std::endl;
                return false;
            }
            std::cout << "[autoFindBestPose] 检测失败，保持当前位置" << std::endl;
            return true;
        }
        
        convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos);
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
        for (double frac = 0.5; frac <= 1.0 + 1e-9; frac += 0.5)
            zrz_candidates.push_back({flangeEulePos[0], flangeEulePos[1], flangeEulePos[2] + dZ * frac,
                                      flangeEulePos[3], flangeEulePos[4], bound_camera_angle(flangeEulePos[5] + dRz_deg * frac)});

        double bestFlangeEulePos[ROB_AXIS_NUM];
        if (tryCandidatePoses(robot_handle, camera, detector,
                               false, zrz_candidates, bestFlangeEulePos) > cur_s) {
            moveToFlangeEulePos(robot_handle, bestFlangeEulePos);            
            cv::Mat frame = camera->captureOneFrame(1500);            
            if(cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa) == sucess)                        
                convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos);
                flangeEulePos[5] = bound_camera_angle(flangeEulePos[5]);
        } else {
            std::cout << "[autoFindBestPose] Z+rz 无改善, iter=" << iter << std::endl;
            break;
        }
    }
    moveToTcpAxisPos(robot_handle, tcp_aa);    
    frame = camera->captureOneFrame(1500);
    // ================================================================
    // Phase 4: XY 精细搜索
    // ================================================================

    std::cout << "[autoFindBestPose] Phase4: XY 精细搜索..." << std::endl;
    const double steps_fine[] = {60, 30, 15};    
    searchXYSteps(robot_handle, camera, detector, steps_fine, std::size(steps_fine), tcp_aa);

    // 运动到最终最佳位姿（tcp_aa 已包含最优平移+旋转）
    {
        moveToTcpAxisPos(robot_handle, tcp_aa);
        frame = camera->captureOneFrame(1500);       
    }

    return true;
}

// 轴角位姿 → 4x4 变换矩阵 (mm,deg → m,rad)
cv::Mat axisAnglePoseToTransform(const double pose[6]) {
    double x = pose[0] / 1000.0;
    double y = pose[1] / 1000.0;
    double z = pose[2] / 1000.0;
    double rx = pose[3] * CV_PI / 180.0;
    double ry = pose[4] * CV_PI / 180.0;
    double rz = pose[5] * CV_PI / 180.0;
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx, ry, rz);
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    T.at<double>(0, 3) = x;
    T.at<double>(1, 3) = y;
    T.at<double>(2, 3) = z;
    return T;
}

// rvec/tvec → 4x4 变换矩阵 (T_camera_calib)
cv::Mat rvecTvecToTransform(const cv::Mat& rvec, const cv::Mat& tvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    T.at<double>(0, 3) = tvec.at<double>(0);
    T.at<double>(1, 3) = tvec.at<double>(1);
    T.at<double>(2, 3) = tvec.at<double>(2);
    return T;
}

// ============================================================
// 计算并验证 3 个候选 Z 值：迭代逼近目标边长比
// ============================================================

// 计算 ChArUco 角点凸包区域的平均深度（0=无效深度）
static double computeAvgDepth(const cv::Mat& depth, const std::vector<cv::Point2f>& corners) {
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

static bool computeZValues(int robot_handle,
    std::shared_ptr<AICamera>& camera,
    int cameraToTcpEnd,
    CharucoDetector& detector,
    double z_out_[3]) {
    double tcp_aa[ROB_AXIS_NUM], tcpEulePos[ROB_AXIS_NUM], oriFlangeEulePos[ROB_AXIS_NUM], outZ[3];
    //double orig_joint[ROB_AXIS_NUM];
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
    const double MIN_SAFE_HEIGHT = 300.0;   // 最低 30cm

    // 通过 IK 搜索最大可达高度（保持当前 rx=-180, ry=0 姿态）
    double euler[ROB_AXIS_NUM];
    double joint[ROB_AXIS_NUM];
    std::copy(oriFlangeEulePos, oriFlangeEulePos + ROB_AXIS_NUM, euler);
    double max_reach = MIN_SAFE_HEIGHT;
    for (double h = 1400.0; h > MIN_SAFE_HEIGHT; h -= 10.0) {
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
// Phase 3: 生成 729 候选 + IK 过滤
// ============================================================

bool generateCandidates(int robot_handle,
                        std::shared_ptr<AICamera>& camera,
                        const double z_values[3],
                        std::vector<PoseCandidate>& candidates) {
    double euler[ROB_AXIS_NUM], tcp_aa[ROB_AXIS_NUM], 
         flangeEulePos[ROB_AXIS_NUM], joint[ROB_AXIS_NUM];
    cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
    convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos);
    cv::Mat frame = camera->captureOneFrame(1500);
    //cr_AxisAngle2Eule(flangeAxisPos, flangeEulePos);
    int total = 0, ik_ok = 0;
    
    for (int zi = 0; zi < 3; zi++) { 
        double z = z_values[zi], dlt = 360 * (3-zi)/3, ddeg[]={5, 10, 10};
        const double ang_offsets[] = {-ddeg[zi], 0.0, ddeg[zi]};
        const double ang_offsetsZ[] = {-10, 0.0, 10};
        const double xy_offsets[] = {-dlt, 0.0, dlt};
    for (double dx : xy_offsets) {
    for (double dy : xy_offsets) {
    
    for (double drx : ang_offsets) {
    for (double dry : ang_offsets) {
    for (double drz : ang_offsetsZ) {
        total++;
        euler[0] = flangeEulePos[0] + dx;
        euler[1] = flangeEulePos[1] + dy;
        euler[2] = z;  // 绝对 Z 值（非偏移量）
        euler[3] = flangeEulePos[3] + drx;
        euler[4] = flangeEulePos[4] + dry;
        euler[5] = flangeEulePos[5] + drz;

        if(convertFlangeOulePos2Joint(robot_handle, euler, joint)){                  
            PoseCandidate cand;
            std::copy(joint, joint + 6, cand.joint);
            candidates.push_back(cand);
            ik_ok++;
        }
    }}}}}}

    std::cout << "[autoCalib] 729 candidates generated, IK reachable: "
              << ik_ok << "/" << total << std::endl;
    return !candidates.empty();
}

int reconnectRobotHandle(int old_handle, char* pchRobotIP){
    CRresult result = error;
    cr_destroy_robot(old_handle);
    int waitingTime = 120000, try_time=0;
    while(result != sucess){
        
       std::cout << "[reconnectRobotHandle] wait to re-connect robot handle: " << ", waiting " << waitingTime << " milliseconds for " << try_time <<" connect\n" << std::flush;
       std::this_thread::sleep_for(std::chrono::milliseconds(waitingTime));          
       result = cr_create_robot(&old_handle, pchRobotIP, 2323, "123");  // 机械臂连接        
       try_time ++;
    }    
    if( result == sucess)
        std::cout << "[reconnectRobotHandle] success!!\n" << std::flush;
    return old_handle;
}

// ============================================================
// Phase 4: 逐点运动 + 采集
// ============================================================

bool executeCollection(int robot_handle,
                       char* pchRobotIP,
                       std::shared_ptr<AICamera>& camera,
                       CharucoDetector& detector,
                       const std::vector<PoseCandidate>& candidates,
                       const std::string& save_dir,AI_det_callback callback) {
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
        CRresult  ret_move = moveToJointPos(robot_handle, candidates[i].joint);        
        if(sucess != ret_move){       
            //int waitingTime = 60000;
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
        double tcp_pose[ROB_AXIS_NUM];
        CRresult ret = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_pose);
        if (ret != sucess) continue;

        double flange_pose[ROB_AXIS_NUM];
        //ret = cr_TcpToFlangePose(tcp_pose, const_cast<double*>(tcp_offset), flange_pose);
        if(!convertTcpAxisPos2FlangPos(robot_handle, tcp_pose, flange_pose))
            continue;

        // 获取实际关节角
        double actual_joint[ROB_AXIS_NUM];
        cr_get_jointActualPos((RobotHandle)robot_handle, actual_joint);

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

// 手眼标定精化: Gauss-Newton 最小化基座标系下板位姿平移方差
// X_init: 初始手眼矩阵 4x4 (flange→camera, 单位: m)
// robot_poses: T_base_flange 列表
// camera_poses: T_camera_calib 列表
static cv::Mat refineHandEyePoseConsistency(
    const cv::Mat& X_init,
    const std::vector<cv::Mat>& robot_poses,
    const std::vector<cv::Mat>& camera_poses) {

    int n = (int)robot_poses.size();
    if (n < 3) return X_init.clone();

    // 参数化 p = [tx, ty, tz, rx, ry, rz]
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

    // 参数 → 4x4
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

    // 计算所有帧的残差和代价
    // r_i = trans(T_base_flange[i] * X * T_cam[i]) - mean(trans(...))
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
        // 组装右端项 R_full (3n × 1)
        cv::Mat R_full(3 * n, 1, CV_64F);
        for (int i = 0; i < n; i++) {
            R_full.at<double>(3 * i,     0) = res[i].at<double>(0);
            R_full.at<double>(3 * i + 1, 0) = res[i].at<double>(1);
            R_full.at<double>(3 * i + 2, 0) = res[i].at<double>(2);
        }

        // 有限差分雅可比 J_full (3n × 6)
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

        // 阻尼 Gauss-Newton: (J^T J + λI) Δp = -J^T r
        cv::Mat H = J_full.t() * J_full;
        cv::Mat g = J_full.t() * R_full;
        for (int j = 0; j < 6; j++)
            H.at<double>(j, j) += lambda;

        cv::Mat dp;
        if (!cv::solve(H, -g, dp, cv::DECOMP_SVD))
            break;

        // 尝试步进
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

bool estimateHandEyeTrans(
     const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    std::vector<cv::Mat> &robot_poses, 
    std::vector<cv::Mat> &camera_poses,
    CalibrationResult& result, 
    bool needFilter
){
     int valid = (int)robot_poses.size();
    if (valid < 5) {
        result.message = "有效帧数不足: " + std::to_string(valid) + " (need >=5)";
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    result.valid_frames = valid;

    // 分解 4x4 矩阵为旋转 + 平移
    std::vector<cv::Mat> R_gripper2base, t_gripper2base;
    std::vector<cv::Mat> R_target2cam, t_target2cam;
    for (int i = 0; i < valid; i++) {
        R_gripper2base.push_back(robot_poses[i](cv::Rect(0, 0, 3, 3)).clone());
        t_gripper2base.push_back(robot_poses[i](cv::Rect(3, 0, 1, 3)).clone());
        R_target2cam.push_back(camera_poses[i](cv::Rect(0, 0, 3, 3)).clone());
        t_target2cam.push_back(camera_poses[i](cv::Rect(3, 0, 1, 3)).clone());
    }

    // 运行多种手眼标定方法，选取平移一致性最好的结果
    struct CalibMethodResult {
        std::string name;
        cv::Mat T;
        double reproj_error;
        std::vector<double> per_frame_errors;
    };
    std::vector<CalibMethodResult> calib_results;
    std::vector<cv::HandEyeCalibrationMethod> calib_methods = {cv::CALIB_HAND_EYE_TSAI, cv::CALIB_HAND_EYE_PARK, cv::CALIB_HAND_EYE_HORAUD};
    std::vector<std::string> calib_method_names = {"TSAI", "PARK", "HORAUD"};

    for (size_t m = 0; m < calib_methods.size(); m++) {
        cv::Mat R_cam_gripper, t_cam_gripper;
        try {
            cv::calibrateHandEye(R_gripper2base, t_gripper2base,
                                  R_target2cam, t_target2cam,
                                  R_cam_gripper, t_cam_gripper,
                                  calib_methods[m]);
        } catch (const cv::Exception& e) {
            std::cerr << "[autoCalib] " << calib_method_names[m] << " 标定异常: " << e.what() << std::endl;
            continue;
        }

        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        R_cam_gripper.copyTo(T(cv::Rect(0, 0, 3, 3)));
        t_cam_gripper.copyTo(T(cv::Rect(3, 0, 1, 3)));

        // cv::Mat T = calib_results[best_idx].T.clone();

        // 非线性和一致性精化（Gauss-Newton，无需外部依赖）
        T = refineHandEyePoseConsistency(T, robot_poses, camera_poses);

        // 评估平移一致性：T_base_calib = T_base_flange * X * T_camera_calib
        std::vector<double> vErr;
        double sum_err = 0.0;
        cv::Mat mean_t(3, 1, CV_64F, cv::Scalar(0));
        for (int i = 0; i < valid; i++)
            mean_t += (robot_poses[i] * T * camera_poses[i])(cv::Rect(3, 0, 1, 3));
        mean_t /= valid;
        for (int i = 0; i < valid; i++) {
            cv::Mat d = (robot_poses[i] * T * camera_poses[i])(cv::Rect(3, 0, 1, 3)) - mean_t;
            vErr.push_back(std::sqrt(d.dot(d)));
            sum_err += d.dot(d);
        }
        calib_results.push_back({calib_method_names[m], T, std::sqrt(sum_err / valid), vErr});

        std::cout << "[autoCalib] " << calib_method_names[m]
                  << " -> 平移一致性误差: " << (std::sqrt(sum_err / valid) * 1000.0)
                  << " mm" << std::endl;
    }

    if (calib_results.empty()) {
        result.message = "所有手眼标定方法均失败";
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    // 选取误差最小的结果
    size_t best_idx = 0;
    for (size_t i = 1; i < calib_results.size(); i++) {
        if (calib_results[i].reproj_error < calib_results[best_idx].reproj_error)
            best_idx = i;
    }

    cv::Mat T = calib_results[best_idx].T.clone();

    // // 非线性和一致性精化（Gauss-Newton，无需外部依赖）
    // T = refineHandEyePoseConsistency(T, robot_poses, camera_poses);

    std::vector<double>& vErr = calib_results[best_idx].per_frame_errors;
    double rms_err = calib_results[best_idx].reproj_error;

    result.T_flange_camera = T.clone();
    result.camera_matrix = camera_matrix.clone();
    result.dist_coeffs = dist_coeffs.clone();
    result.reproj_error = rms_err;
    result.success = true;
    result.message = "标定成功, 有效帧: " + std::to_string(valid) +
                     ", 平均误差: " + std::to_string(rms_err * 1000.0) + " mm"
                     " (最优: " + calib_results[best_idx].name + ")";

    std::cout << "[autoCalib] " << result.message << std::endl;
    std::cout << "[autoCalib] 最优方法: " << calib_results[best_idx].name << std::endl;
    std::cout << "[autoCalib] T_flange_camera:\n" << result.T_flange_camera << std::endl;

    // 位姿分布分析
    if (valid > 0) {
        cv::Mat T0 = robot_poses[0] * T * camera_poses[0];
        double x_min = T0.at<double>(0, 3) * 1000.0, x_max = x_min;
        double y_min = T0.at<double>(1, 3) * 1000.0, y_max = y_min;
        double z_min = T0.at<double>(2, 3) * 1000.0, z_max = z_min;
        for (int i = 1; i < valid; i++) {
            cv::Mat T_base = robot_poses[i] * T * camera_poses[i];
            double x = T_base.at<double>(0, 3) * 1000.0;
            double y = T_base.at<double>(1, 3) * 1000.0;
            double z = T_base.at<double>(2, 3) * 1000.0;
            if (x < x_min) x_min = x; if (x > x_max) x_max = x;
            if (y < y_min) y_min = y; if (y > y_max) y_max = y;
            if (z < z_min) z_min = z; if (z > z_max) z_max = z;
        }
        std::cout << "[autoCalib] 标定板在基坐标系下的空间分布:" << std::endl;
        std::cout << "[autoCalib]   X: " << x_min << " ~ " << x_max << " mm, 跨度 " << (x_max - x_min) << " mm" << std::endl;
        std::cout << "[autoCalib]   Y: " << y_min << " ~ " << y_max << " mm, 跨度 " << (y_max - y_min) << " mm" << std::endl;
        std::cout << "[autoCalib]   Z: " << z_min << " ~ " << z_max << " mm, 跨度 " << (z_max - z_min) << " mm" << std::endl;
    }

    if(needFilter){
        int removed = 0;
        float thresh = 0.0015;
        for(int i=valid-1; i>=0; i--){
            if(vErr[i] > thresh){
                robot_poses.erase(robot_poses.begin() + i);
                camera_poses.erase(camera_poses.begin() + i);
                removed++;
            }
       }
        if (removed > 0) {
            std::cout << "[autoCalib] 离群剔除: " << removed
                      << " 帧 (阈值 " << thresh << "mm), 剩余 " << robot_poses.size() << " 帧" << std::endl;
        }
    }

    return true;
}} // anonymous namespace

// ============================================================
// 相机内参标定（从目录读取图像）
// ============================================================

bool loadImagesFromDisk(const std::string& save_dir,
                         std::vector<std::string>& img_files,
                         cv::Size& image_size) {
    namespace fs = std::filesystem;
    for (auto& entry : fs::directory_iterator(save_dir)) {
        std::string name = entry.path().filename().string();
        if (name.size() >= 4 && name.substr(name.size() - 4) == ".jpg")
            img_files.push_back(entry.path().string());
    }
    std::sort(img_files.begin(), img_files.end());
    if (img_files.empty()) return false;
    cv::Mat first = cv::imread(img_files[0]);
    if (first.empty()) return false;
    image_size = first.size();
    return true;
}

bool filterDataForCameraCalibration(std::vector<std::vector<cv::Point3f>>& object_points,
    std::vector<std::vector<cv::Point2f>>& image_points,
    const cv::Size& image_size,
    cv::Mat& camera_matrix,
    cv::Mat& dist_coeffs, int wantedNum){
    const int min_frames = 5;
    int batch_size = 50;
    float keepRatio =std::max<float>(0.3, std::min<float>((float)(wantedNum *1.5)  / image_points.size() + 0.1, 1.0));
    
    size_t n_total = object_points.size();
    int nTimes = n_total / batch_size;
    batch_size = n_total / nTimes + 1;
    

    struct FrameRecord {
        std::vector<cv::Point3f> obj_pts;
        std::vector<cv::Point2f> img_pts;
        double error;
    };
    std::vector<FrameRecord> good_frames;
   
    // ——— 分批处理 ———
    for (size_t start = 0; start < n_total; start += batch_size) {
        size_t end = std::min(start + batch_size, n_total);
        size_t n_batch = end - start;

        std::vector<std::vector<cv::Point3f>> batch_obj(
            object_points.begin() + start, object_points.begin() + end);
        std::vector<std::vector<cv::Point2f>> batch_img(
            image_points.begin() + start, image_points.begin() + end);

        cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
        std::vector<cv::Mat> rvecs, tvecs;
        double err = cv::calibrateCamera(batch_obj, batch_img, image_size,
                                          K, D, rvecs, tvecs);
        std::cout << "[autoCalib] 批次 " << (start / batch_size) << "/" <<nTimes
                  << ": , 重投影误差 " << err << " px" << std::endl;

        // 计算每帧投影误差
        std::vector<FrameRecord> batch_records;
        batch_records.reserve(n_batch);
        for (size_t i = 0; i < n_batch; i++) {
            std::vector<cv::Point2f> proj;
            cv::projectPoints(batch_obj[i], rvecs[i], tvecs[i], K, D, proj);
            double sum_err = 0.0;
            for (size_t j = 0; j < proj.size(); j++)
                sum_err += cv::norm(proj[j] - batch_img[i][j]);
            batch_records.push_back({batch_obj[i], batch_img[i], sum_err / proj.size()});
        }

        // 取前 30%（误差最小）
        std::sort(batch_records.begin(), batch_records.end(),
                  [](const FrameRecord& a, const FrameRecord& b) {
                      return a.error < b.error;
                  });
        size_t n_keep = std::max<int>(size_t(1), (int)(n_batch * keepRatio));
        for (size_t i = 0; i < n_keep; i++)
            good_frames.push_back(batch_records[i]);
        std::cout << "[autoCalib]  保留 " << n_keep << "/" << n_batch
                  << " 帧 (前"<< 100 * keepRatio <<"%, 最大误差 " << batch_records[n_keep - 1].error << " px)"                  
                  << std::endl;
    }

    // ——— 取误差最小的 max_good 帧做最终标定 ———
    std::sort(good_frames.begin(), good_frames.end(),
              [](const FrameRecord& a, const FrameRecord& b) {
                  return a.error < b.error;
              });

    size_t n_final = std::min(good_frames.size(), size_t(wantedNum));
    if (n_final < (size_t)min_frames) {
        // std::cerr << "[autoCalib] 合格帧不足 (" << n_final << "), 使用全部帧标定"
        //           << std::endl;
        // camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        // dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
        // std::vector<cv::Mat> rvecs, tvecs;
        // cv::calibrateCamera(object_points, image_points, image_size,
        //                      camera_matrix, dist_coeffs, rvecs, tvecs);
        return false;
    }

    std::cout << "[autoCalib] 保留 " << n_final << "/" << n_total
              << " 帧做最终标定" << std::endl;
    
    std::vector<std::vector<cv::Point3f>> final_obj;
    std::vector<std::vector<cv::Point2f>> final_img;
    final_obj.reserve(n_final);
    final_img.reserve(n_final);
    good_frames.erase(good_frames.begin() + n_final, good_frames.end());
    std::random_device rd;
    std::shuffle(good_frames.begin(), good_frames.end(), std::default_random_engine(rd()));
    for (size_t i = 0; i < n_final; i++) {
        final_obj.push_back(good_frames[i].obj_pts);
        final_img.push_back(good_frames[i].img_pts);
    }
    object_points = final_obj;
    image_points = final_img;
    return true;
}

// ——— 分批标定 + 逐帧过滤 + 最终标定 ———
// 每批最多 batch_size 帧，标定后只取误差最小的前 30% 进入候选池，
// 最后从候选池取误差最小的 max_good 帧做最终标定。
static bool calibrateCameraFiltered(
    const std::vector<std::vector<cv::Point3f>>& object_points,
    const std::vector<std::vector<cv::Point2f>>& image_points,
    const cv::Size& image_size,
    cv::Mat& camera_matrix,
    cv::Mat& dist_coeffs) {
    int n_final = 65;
    float fRatio = sqrt((float)n_final / image_points.size());
    int  wantedNum =std::max<int>( (int)(image_points.size() * fRatio), std::min<int>(image_points.size(), n_final * 2)); 
    std::vector<std::vector<cv::Point3f>> object_points_l = object_points;
    std::vector<std::vector<cv::Point2f>> image_points_l = image_points;
    if(filterDataForCameraCalibration(object_points_l, image_points_l, image_size, camera_matrix, dist_coeffs, wantedNum))
       filterDataForCameraCalibration(object_points_l, image_points_l, image_size, camera_matrix, dist_coeffs, n_final);
    

    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    double final_err = cv::calibrateCamera(object_points_l, image_points_l, image_size,
                                            camera_matrix, dist_coeffs,
                                            rvecs, tvecs);
    std::cout << "[autoCalib] 最终标定, " << n_final << " 帧, 重投影误差: "
              << final_err << " px" << std::endl;
    std::cout << "[autoCalib] 相机矩阵:\n" << camera_matrix << std::endl;
    return true;
}

// 相机内参精化: 用全部有效帧数据做离群过滤后重标定
// K, D 入/出: 初始值来自 filtered 管线, 返回精化结果
static void refineCameraIntrinsics(
    cv::Mat& K, cv::Mat& D,
    const std::vector<std::vector<cv::Point3f>>& all_obj,
    const std::vector<std::vector<cv::Point2f>>& all_img,
    const cv::Size& image_size) {

    int n = (int)all_obj.size(), miniNum = std::min<int>(50, all_obj.size());
    if (n < 5) return;

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
        recs.push_back({i, sum / proj.size()});
    }
    if ((int)recs.size() < 5) return;

    std::sort(recs.begin(), recs.end(),
              [](const Rec& a, const Rec& b) { return a.err < b.err; });
    
    double median = recs[miniNum / 2].err;
    double thresh = median * 1.2;

    std::vector<std::vector<cv::Point3f>> good_obj;
    std::vector<std::vector<cv::Point2f>> good_img;
    for (auto& r : recs) {
        if (r.err <= thresh) {
            good_obj.push_back(all_obj[r.idx]);
            good_img.push_back(all_img[r.idx]);
        }
    }
    std::cout << "[refineCam] 重投影过滤: " << good_obj.size() << "/" << n
              << " 帧保留 (median=" << median << " px, thresh=" << thresh << " px)" << std::endl;
    if ((int)good_obj.size() < 5) return;

    std::vector<cv::Mat> rvecs, tvecs;
    double final_err = cv::calibrateCamera(good_obj, good_img, image_size, K, D, rvecs, tvecs,
                         cv::CALIB_USE_INTRINSIC_GUESS);
    std::cout << "[refineCam] 精化完成: " << good_obj.size() << " 帧, err=" << final_err << " px" << std::endl;
    std::cout << "[autoCalib] 相机矩阵:\n" << K << std::endl;
}

bool calibrateCameraFromDisk(const std::string& save_dir,
                              CharucoDetector& detector,
                              cv::Mat& camera_matrix,
                              cv::Mat& dist_coeffs) {

    const auto& board_points = detector.getChessboardCorners3D();

    std::vector<std::string> img_files;
    cv::Size image_size;
    if (!loadImagesFromDisk(save_dir, img_files, image_size))
        return false;

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    std::random_device rd;
    std::shuffle(img_files.begin(), img_files.end(), std::default_random_engine(rd()));

    for (const auto& path : img_files) {
        cv::Mat img = cv::imread(path);
        if (img.empty()) continue;

        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detector.detectCorners(img, corners, ids))
            continue;

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
        }
    }

    if (object_points.size() < 5) {
        std::cerr << "[autoCalib] 帧数不足，无法标定相机内参 (need >=5, got "
                  << object_points.size() << ")" << std::endl;
        return false;
    }

    // 保存全部有效帧，过滤 + 精化用
    auto all_obj = object_points;
    auto all_img = image_points;

    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    if (!calibrateCameraFiltered(object_points, image_points,
                                 image_size, camera_matrix, dist_coeffs))
        return false;

    // 以过滤后的 K,D 为初值，用全部有效帧做离群过滤 + 重标定
    refineCameraIntrinsics(camera_matrix, dist_coeffs,
                           all_obj, all_img, image_size);
    return true;
}

// ============================================================
// 公开 API
// ============================================================

bool calibrateHandEyeFromDisk(
    const std::string& save_dir,
    cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const CharucoBoardConfig& board_config,
    CalibrationResult& result,
    const std::string& output_yaml) {

    CharucoDetector detector(board_config);
    detector.setCameraParams(camera_matrix, dist_coeffs);

    // 读取法兰位姿
    std::string pos_file = save_dir + "/rob_pos.csv";
    std::ifstream pf(pos_file);
    if (!pf.is_open()) {
        result.message = "无法打开 " + pos_file;
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }
    std::vector<cv::Mat> flange_poses;
    std::string line;
    while (std::getline(pf, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        double pose[6];
        for (int i = 0; i < 6; i++) {
            std::string val;
            if (!std::getline(ss, val, ',')) break;
            pose[i] = std::stod(val);
        }
        flange_poses.push_back(axisAnglePoseToTransform(pose));
    }

    // 扫描图像文件
    std::vector<std::string> img_files;
    cv::Size image_size;
    if (!loadImagesFromDisk(save_dir, img_files, image_size))
        return false;

    size_t n = std::min(img_files.size(), flange_poses.size());
    const auto& board_points = detector.getChessboardCorners3D();

    // 逐帧检测 + 计算重投影误差
    struct FrameRecord {
        cv::Mat robot_pose;
        cv::Mat camera_pose;
        double error;
    };
    std::vector<FrameRecord> frames;
    frames.reserve(n);

    for (size_t i = 0; i < n; i++) {
        cv::Mat img = cv::imread(img_files[i]);
        if (img.empty()) continue;
        cv::Mat rvec, tvec;
        std::vector<cv::Point2f> corners;
        std::vector<int> ids;

        if (!detector.detectPose(img, rvec, tvec, corners, ids, 16))
            continue;

        // 计算该帧重投影误差
        std::vector<cv::Point3f> pts3;
        std::vector<cv::Point2f> pts2;
        for (size_t j = 0; j < ids.size(); j++) {
            int id = ids[j];
            if (id >= 0 && (size_t)id < board_points.size()) {
                pts3.push_back(board_points[id]);
                pts2.push_back(corners[j]);
            }
        }
        if (pts3.size() < 4) continue;

        std::vector<cv::Point2f> projected;
        cv::projectPoints(pts3, rvec, tvec, camera_matrix, dist_coeffs, projected);
        double sum_err = 0.0;
        for (size_t j = 0; j < projected.size(); j++)
            sum_err += cv::norm(projected[j] - pts2[j]);

        double err = sum_err / projected.size();
        frames.push_back({flange_poses[i], rvecTvecToTransform(rvec, tvec), err});
    }

    // 按误差排序，保留误差最小的帧
    std::sort(frames.begin(), frames.end(),
              [](const FrameRecord& a, const FrameRecord& b) { return a.error < b.error; });

    size_t n_orig = frames.size();
    size_t n_keep = std::min(n_orig, std::max(size_t(300), size_t(n_orig * 0.7)));
    std::cout << "[autoCalib] 手眼标定: " << n_orig << " 帧, 保留误差最小的 "
              << n_keep << " 帧" << std::endl;

    std::vector<cv::Mat> robot_poses, camera_poses;
    robot_poses.reserve(n_keep);
    camera_poses.reserve(n_keep);
    for (size_t i = 0; i < n_keep; i++) {
        robot_poses.push_back(frames[i].robot_pose);
        camera_poses.push_back(frames[i].camera_pose);
    }

    estimateHandEyeTrans(camera_matrix, dist_coeffs, robot_poses, camera_poses, result, true);
    estimateHandEyeTrans(camera_matrix, dist_coeffs, robot_poses, camera_poses, result, false);

    // Save result to YAML if output path provided
    if (!output_yaml.empty() && result.success) {
        cv::FileStorage out(output_yaml, cv::FileStorage::WRITE);
        if (out.isOpened()) {
            out << "T_flange_camera" << result.T_flange_camera;
            out << "camera_matrix" << result.camera_matrix;
            out << "distortion_coefficients" << result.dist_coeffs;
            out << "reprojection_error_mm" << result.reproj_error * 1000.0;
            out << "valid_frames" << result.valid_frames;
            out.release();
        }
    }

    if (result.success) {
        std::cout << "\n========== 标定结果 ==========" << std::endl;
        std::cout << "手眼矩阵 T_flange_camera:\n" << result.T_flange_camera << std::endl;
        std::cout << "有效帧数: " << result.valid_frames << std::endl;
        std::cout << "平均误差: " << (result.reproj_error * 1000.0) << " mm" << std::endl;
        std::cout << result.message << std::endl;
        if (!output_yaml.empty())
            std::cout << "结果已保存到: " << output_yaml << std::endl;
    }

    return result.success;
}

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
    AI_det_callback callback) {

    g_save_dir = save_dir;
    g_callback = callback;

    std::cout << "\n========== 自动手眼标定开始 ==========" << std::endl;

    CharucoDetector detector(board_config);

    // ---- Phase 1b: 自动寻找最佳位姿 ----
    double tcp_aa[ROB_AXIS_NUM];   
    rotateFlangeToHorizon(robot_handle, NULL);
    cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
    std::cout << "[autoCalib] 自动寻找最佳位姿..." << std::endl;
    if (!autoFindBestPose(robot_handle, camera, detector,
                            save_dir, callback)) {
        std::cerr << "[autoCalib] 自动寻找最佳位姿失败" << std::endl;
        return false;
    }

    // ---- Phase 2: 计算动态 Z 值 ----
    double z_values[3];
    {
        if (computeZValues(robot_handle, camera, cameraToTcpEnd, detector, z_values)) {
            std::cout << "[autoCalib] 动态 Z 值: "
                      << z_values[0] << ", " << z_values[1] << ", " << z_values[2] << std::endl;
        } else {
            std::cout << "[autoCalib] Z 值计算失败，使用固定偏移 [-300,0,+300]" << std::endl;
        }
    }

    // ---- Phase 3: 生成候选 + IK 过滤 ----
    std::vector<PoseCandidate> candidates;
    if (!generateCandidates(robot_handle, camera, z_values, candidates)) {
        result.message = "无 IK 可达位姿";
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    // ---- Phase 4: 逐点运动 + 采集 ----
    if (!executeCollection(robot_handle, pchRobotIP, camera, detector,
                            candidates, save_dir, callback)) {
        result.message = "未采集到有效数据";
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    // ---- Phase 5: 标定（直接从磁盘读取） ----
    cv::Mat K, D;
    if (true) //camera_matrix.empty() || dist_coeffs.empty()) 
    {
        std::cout << "[autoCalib] 相机内参未知，从磁盘标定..." << std::endl;
        if (!calibrateCameraFromDisk(save_dir, detector, K, D)) {
            result.message = "相机内参标定失败";
            std::cerr << "[autoCalib] " << result.message << std::endl;
            return false;
        }
    } else {
        camera_matrix.getMat().copyTo(K);
        dist_coeffs.getMat().copyTo(D);
        std::cout << "[autoCalib] 使用已有相机内参" << std::endl;
    }

    if (!calibrateHandEyeFromDisk(save_dir, K, D, board_config, result, output_yaml))
        return false;

    K.copyTo(camera_matrix);
    D.copyTo(dist_coeffs);

    std::cout << "========== 自动手眼标定完成 ==========\n" << std::endl;
    return true;
}
