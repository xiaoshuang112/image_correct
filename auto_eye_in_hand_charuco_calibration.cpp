#include "auto_eye_in_hand_charuco_calibration.hpp"

#include "AICamera.hpp"
#ifdef useORB
#include <OrbbecCamera.hpp>
#endif // REL_DBG

#ifdef useOAK
#include "OAKCamera.hpp"
#endif // REL_DBG

#include "robotapi.h"
#include "basestruct.h"

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>
#include <array>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>

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

// 运动到目标关节角（等效 goto_jointpos）
void moveToJointPos(int robot_handle, const double joint[6]) {
    double pose[6] = {0, 0, 0, 0, 0, 0};
    PointControlParaSimple simple;
    for (int i = 0; i < ROB_AXIS_NUM; i++) {
        simple.pose[i] = pose[i];
        simple.jointpos[i] = joint[i];
        simple.tcpOffset[i] = 0;
        simple.coordinatePose[i] = 0;
        simple.speed[i] = 30;
        simple.acc[i] = 30;
    }
    simple.coordinateType = jointCoordinate;
    simple.pointTransType = pointTransStop;
    simple.motiontriggerMode = MovetriggerbyOnlyRpc;

    PointControlPara para;
    CRresult ret = cr_move_pointControlPara_transfer(
        (RobotHandle)robot_handle, simple, &para);
    if (ret != sucess) {
        std::cerr << "[autoCalib] cr_move_pointControlPara_transfer failed: "
                  << ret << std::endl;
        return;
    }
    ret = cr_move_joint((RobotHandle)robot_handle, para, TRUE);
    if (ret != sucess) {
        std::cerr << "[autoCalib] cr_move_joint failed: " << ret << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

namespace {

// 前向声明（visual servoing 函数需要）
cv::Mat axisAnglePoseToTransform(const double pose[6]);
// void moveToJointPos(int robot_handle, const double joint[6]);

// 创建 ChArUco 标定板
bool createCharucoBoard(const CharucoBoardConfig& cfg,
                        cv::Ptr<cv::aruco::Dictionary>& dictionary,
                        cv::Ptr<cv::aruco::CharucoBoard>& board) {
    int dict_id;
    switch (cfg.dictionary_id) {
        case 0:  dict_id = cv::aruco::DICT_4X4_250;  break;
        case 1:  dict_id = cv::aruco::DICT_4X4_1000; break;
        case 2:  dict_id = cv::aruco::DICT_5X5_100;  break;
        case 3:  dict_id = cv::aruco::DICT_5X5_1000; break;
        case 4:  dict_id = cv::aruco::DICT_5X5_250;  break;
        case 5:  dict_id = cv::aruco::DICT_6X6_250;  break;
        default: dict_id = cv::aruco::DICT_5X5_250;  break;
    }

    dictionary = cv::makePtr<cv::aruco::Dictionary>(
        cv::aruco::getPredefinedDictionary(dict_id));

#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
    board = cv::makePtr<cv::aruco::CharucoBoard>(
        cv::Size(cfg.squares_x, cfg.squares_y),
        cfg.square_length, cfg.marker_length, *dictionary);
#else
    board = cv::aruco::CharucoBoard::create(
        cfg.squares_x, cfg.squares_y,
        cfg.square_length, cfg.marker_length, dictionary);
#endif
    return true;
}

// 检测 ChArUco 角点（无需相机内参）
bool detectCharucoCorners(const cv::Mat& image,
                          const cv::Ptr<cv::aruco::Dictionary>& dictionary,
                          const cv::Ptr<cv::aruco::CharucoBoard>& board,
                          std::vector<cv::Point2f>& corners,
                          std::vector<int>& ids) {
    cv::Mat gray;
    if (image.channels() == 3)
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else
        gray = image.clone();

    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;

    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::CharucoParameters charucoParams;
    detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    charucoParams.minMarkers = 1;
    charucoParams.tryRefineMarkers = true;

#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
    cv::aruco::CharucoDetector detector(*board, charucoParams, detectorParams);
    detector.detectBoard(gray, corners, ids, markerCorners, markerIds);
#else
    cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, detectorParams);
    if (!markerIds.empty()) {
        cv::aruco::interpolateCornersCharuco(
            markerCorners, markerIds, gray, board,
            corners, ids, cv::noArray(), cv::noArray(), 1);
    }
#endif

    if (ids.empty() && !markerIds.empty()) {
        cv::aruco::interpolateCornersCharuco(
            markerCorners, markerIds, gray, board,
            corners, ids, cv::noArray(), cv::noArray(), 1);
    }
    if (ids.empty()) {
        cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds);
        if (!markerIds.empty()) {
            cv::aruco::interpolateCornersCharuco(
                markerCorners, markerIds, gray, board,
                corners, ids, cv::noArray(), cv::noArray(), 1);
        }
    }

    return !ids.empty();
}

// 检测 ChArUco 并估算位姿（需要相机内参）
bool detectCharucoPose(const cv::Mat& image,
                       const cv::Ptr<cv::aruco::Dictionary>& dictionary,
                       const cv::Ptr<cv::aruco::CharucoBoard>& board,
                       const cv::Mat& camera_matrix,
                       const cv::Mat& dist_coeffs,
                       cv::Mat& rvec, cv::Mat& tvec,
                       std::vector<cv::Point2f>& corners,
                       std::vector<int>& ids, int minCornersNum ) {
    if (!detectCharucoCorners(image, dictionary, board, corners, ids))
        return false;
    if (ids.size() < minCornersNum) return false;

    return cv::aruco::estimatePoseCharucoBoard(
        corners, ids, board, camera_matrix, dist_coeffs, rvec, tvec);
}

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

// ============================================================
// 评价函数 + XY 搜索（共用）
// ============================================================

// 评价当前位姿检测质量
// use_center=true:  score = ids.size() * 10000 - 中心距像素
// use_center=false: score = ids.size() * 1000000 + 凸包面积
static std::tuple<double, int, cv::Point2f> evaluatePose(
    std::shared_ptr<AICamera>& camera,
    const cv::Ptr<cv::aruco::Dictionary>& dictionary,
    const cv::Ptr<cv::aruco::CharucoBoard>& board,
    bool use_center) {
    cv::Mat f = camera->captureOneFrame(1500);
    if (f.empty()) return {0, 0, cv::Point2f(0, 0)};
    double cx = f.cols / 2.0;
    double cy = f.rows / 2.0;
    std::vector<cv::Point2f> c;
    std::vector<int> i;
    if (!detectCharucoCorners(f, dictionary, board, c, i))
        return {0, (int)i.size(), cv::Point2f(0, 0)};
    cv::Size board_sz;
#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
    board_sz = board->getChessboardSize();
#else
    board_sz = board->checkboradSize;
#endif

    std::vector<cv::Point2f> h;
    cv::convexHull(c, h);
    cv::Point2f bc(0, 0);
    for (auto& pt : c) { bc.x += pt.x; bc.y += pt.y; }
    bc.x /= (float)c.size(); bc.y /= (float)c.size();
    double pd = ((bc.x-cx)*(bc.x-cx) + (bc.y-cy)*(bc.y-cy));
    double score = (double)i.size() * 10000.0;
    // if (use_center) {
    //     score -= pd;
    // } else {
        double hull_area = cv::contourArea(h);
        score = (double)i.size() * 1000000.0 + hull_area - pd/10.0
                - fabs(-computeBoardAngle(c, i, board_sz.width, board_sz.height, f.cols, f.rows) * 180.0 / CV_PI) * 1000;
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
    const cv::Ptr<cv::aruco::Dictionary>& dictionary,
    const cv::Ptr<cv::aruco::CharucoBoard>& board,
    bool use_center,
    const std::vector<std::array<double, 6>>& candidates,
    double best_joint[6]) {
    // 保存原始关节角
    double orig_j[ROB_AXIS_NUM];
    if (cr_get_jointActualPos((RobotHandle)robot_handle, orig_j) != sucess)
        return -1.0;

    double best_j[ROB_AXIS_NUM];
    double best_s = -1e18;
    bool found = false;

    for (const auto& euler : candidates) {
        double ref_j[ROB_AXIS_NUM];
        if (cr_get_jointActualPos((RobotHandle)robot_handle, ref_j) != sucess) 
            continue;
        double euler_arr[ROB_AXIS_NUM];
        std::copy(euler.begin(), euler.end(), euler_arr);
        double aa[ROB_AXIS_NUM];
        if (cr_Eule2AxisAngle(euler_arr, aa) != sucess) 
            continue;
        double joint[ROB_AXIS_NUM];
        if (cr_kineInverse((RobotHandle)robot_handle, aa, ref_j, joint) != sucess) 
            continue;

        moveToJointPos(robot_handle, joint);        

        double s; int n; cv::Point2f bc;
        std::tie(s, n, bc) = evaluatePose(camera, dictionary, board, use_center);
        if (s > best_s) {
            best_s = s;
            std::copy(joint, joint + ROB_AXIS_NUM, best_j);
            found = true;
        }
    }

    // 回到原始位置（由调用方决定是否运动到最佳位姿）   
    moveToJointPos(robot_handle, orig_j);   
    if (!found){          
        return -1.0;}

    std::copy(best_j, best_j + ROB_AXIS_NUM, best_joint);
    return best_s;
}

// 多步长 XY 搜索（Phase 2 / Phase 4 共用）
static void searchXYSteps(int robot_handle,
    std::shared_ptr<AICamera>& camera,
    const cv::Ptr<cv::aruco::Dictionary>& dictionary,
    const cv::Ptr<cv::aruco::CharucoBoard>& board,
    const double* steps, int num_steps,
    double& fixed_x, double& fixed_y, double fixed_z, double current_rz_deg,
    double tcp_aa[6]) {
    cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
    
    for (int i = 0; i < num_steps; i++) {
        double step = steps[i];
        double score; int n; cv::Point2f bc;
        std::tie(score, n, bc) = evaluatePose(camera, dictionary, board, false);        
        std::vector<std::array<double, 6>> xy_candidates;
        for (int sx : {-1, 0, 1}) for (int sy : {-1, 0, 1}) {
            if (sx == 0 && sy == 0) continue;
            xy_candidates.push_back({fixed_x + sx*step, fixed_y + sy*step, fixed_z,
                                      -180.0, 0.0, current_rz_deg});
        }
        double best_j[ROB_AXIS_NUM];
        if (tryCandidatePoses(robot_handle, camera, dictionary, board,
                               false, xy_candidates, best_j) > score) {
            moveToJointPos(robot_handle, best_j);            
            double actual[6];
            cr_get_tcpActualPose((RobotHandle)robot_handle, actual);
            fixed_x = actual[0]; fixed_y = actual[1];
            tcp_aa[0] = fixed_x; tcp_aa[1] = fixed_y;
        }
    }
    moveToJointPos(robot_handle, tcp_aa);    
}

float bound_camera_angle(float angle){
    if(angle > 90)
       return angle - 180;
    if(angle < -90)
       return 180 + angle;
    return angle;
}

// 一次公式反算找最佳位姿：板居中、面积 25-30%、长边平行
bool autoFindBestPose(int robot_handle,
                      std::shared_ptr<AICamera>& camera,
                      const cv::Ptr<cv::aruco::Dictionary>& dictionary,
                      const cv::Ptr<cv::aruco::CharucoBoard>& board,
                      double tcp_aa[6],        // in/out: TCP 轴角 (mm,deg)
                      const double tcp_offset[6]) {

    const double TARGET_AREA_RATIO = 0.275;
    const int MAX_REFINE_ZRZ = 6;
    const double Z_STEP = 100.0;
    const double Z_MAX_OFFSET = 600.0;

    // ---- 固定 rx=-180°, ry=0°（标定板水平），保留 rz ----
    double euler_fixed[ROB_AXIS_NUM];
    double current_rz_deg = 0.0;
    CRresult ret = cr_AxisAngle2Eule(tcp_aa, euler_fixed);
    if (ret == sucess) {
        current_rz_deg = euler_fixed[5];
        euler_fixed[3] = -180.0;
        euler_fixed[4] = 0.0;
        double fixed_aa[ROB_AXIS_NUM];
        ret = cr_Eule2AxisAngle(euler_fixed, fixed_aa);
        if (ret == sucess)
            std::copy(fixed_aa, fixed_aa + ROB_AXIS_NUM, tcp_aa);
    }
    double fixed_x = tcp_aa[0], fixed_y = tcp_aa[1], fixed_z = tcp_aa[2];

    // 标定板总角点数
    cv::Size board_sz =
#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
        board->getChessboardSize();
#else
        board->checkboradSize;
#endif
    int total_corners = (board_sz.width - 1) * (board_sz.height - 1);

    // ================================================================
    // Phase 1: Z 搜索 — 码太少时向上找更好的高度
    // ================================================================

    double cur_score;
    int cur_codes;
    cv::Point2f cur_bc;
    std::tie(cur_score, cur_codes, cur_bc) = evaluatePose(camera, dictionary, board, false);

    //if (cur_codes > 0 && cur_codes < total_corners) 
    {
        std::cout << "[autoFindBestPose] Phase1: Z 搜索 (当前码=" << cur_codes
                  << "/" << total_corners << ")" << std::endl;

        std::vector<std::array<double, 6>> z_candidates;
        for (int idx = 1; idx * Z_STEP <= Z_MAX_OFFSET; idx++)
            z_candidates.push_back({fixed_x, fixed_y, fixed_z + idx * Z_STEP,
                                    -180.0, 0.0, current_rz_deg});

        double best_j[ROB_AXIS_NUM];
        double best_s = tryCandidatePoses(robot_handle, camera, dictionary, board,
                                           false, z_candidates, best_j);
        if (best_s > cur_score) {
            moveToJointPos(robot_handle, best_j);            
            double actual[6];
            cr_get_tcpActualPose((RobotHandle)robot_handle, actual);
            fixed_z = actual[2];
            tcp_aa[2] = fixed_z;
            std::cout << "[autoFindBestPose] Phase1 选定 Z=" << fixed_z << std::endl;
        }
        moveToJointPos(robot_handle, tcp_aa);        
    }

    // ================================================================
    // Phase 2: XY 搜索 — 码不全时找更好的水平位置
    // ================================================================
    
    std::tie(cur_score, cur_codes, cur_bc) = evaluatePose(camera, dictionary, board, false);

    //if (cur_codes > 0 && cur_codes < total_corners)
     {
        std::cout << "[autoFindBestPose] Phase2: XY 搜索 (码=" << cur_codes
                  << "/" << total_corners << ")" << std::endl;
        const double steps[] = {240, 120, 60};
        searchXYSteps(robot_handle, camera, dictionary, board,
                       steps, std::size(steps), fixed_x, fixed_y, fixed_z, current_rz_deg, tcp_aa);
    }

    // ================================================================
    // Phase 3: Z+rz 拟合（不动 XY）
    // ================================================================

    for (int iter = 0; iter < MAX_REFINE_ZRZ; iter++) {
        cv::Mat frame = camera->captureOneFrame(1500);
        if (frame.empty()) {
            std::cerr << "[autoFindBestPose] 采集图像失败" << std::endl;
            return false;
        }

        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detectCharucoCorners(frame, dictionary, board, corners, ids) || ids.size() < 4) {
            if (iter == 0) {
                std::cerr << "[autoFindBestPose] 初始帧检测失败" << std::endl;
                return false;
            }
            std::cout << "[autoFindBestPose] 检测失败，保持当前位置" << std::endl;
            return true;
        }

        // 利用像素面积比估算 Z 调整（area ∝ 1/Z²）
        std::vector<cv::Point2f> hull;
        cv::convexHull(corners, hull);
        double hull_area = cv::contourArea(hull);
        double board_area = hull_area * total_corners / std::max((int)corners.size(), 1);
        double area_ratio = board_area / (double)(frame.cols * frame.rows);
        double Z_desired = fixed_z * std::sqrt(area_ratio / TARGET_AREA_RATIO);
        double dZ =  (Z_desired - fixed_z); 
        double dRz_deg = -computeBoardAngle(corners, ids, board_sz.width, board_sz.height, frame.cols, frame.rows) * 180.0 / CV_PI;

        if (std::abs(dZ) < 30.0 && std::abs(dRz_deg) < 1.0) {
            std::cout << "[autoFindBestPose] Z+rz 收敛, iter=" << iter << std::endl;
            break;
        }

        // 在 0.2~1.0 比例下尝试 Z+rz 组合，选评分最高的
        double cur_s; int n; cv::Point2f bc;
        std::tie(cur_s, n, bc) = evaluatePose(camera, dictionary, board, false);

        std::vector<std::array<double, 6>> zrz_candidates;
        for (double frac = 0.5; frac <= 1.0 + 1e-9; frac += 0.5)
            zrz_candidates.push_back({fixed_x, fixed_y, fixed_z + dZ * frac,
                                      -180.0, 0.0, bound_camera_angle(current_rz_deg + dRz_deg * frac)});

        double best_j[ROB_AXIS_NUM];
        if (tryCandidatePoses(robot_handle, camera, dictionary, board,
                               false, zrz_candidates, best_j) > cur_s) {
            moveToJointPos(robot_handle, best_j);            
            cv::Mat frame = camera->captureOneFrame(1500);
            double actual[6];
            cr_get_tcpActualPose((RobotHandle)robot_handle, actual);
            fixed_z = actual[2];
            tcp_aa[2] = fixed_z;
            double eul[ROB_AXIS_NUM];
            if (cr_AxisAngle2Eule(actual, eul) == sucess)
                current_rz_deg = bound_camera_angle(eul[5]);
        } else {
            std::cout << "[autoFindBestPose] Z+rz 无改善, iter=" << iter << std::endl;
            break;
        }
    }
    moveToJointPos(robot_handle, tcp_aa);    

    // ================================================================
    // Phase 4: XY 精细搜索
    // ================================================================

    std::cout << "[autoFindBestPose] Phase4: XY 精细搜索..." << std::endl;
    const double steps_fine[] = {60, 30, 15};
    
    searchXYSteps(robot_handle, camera, dictionary, board,
                   steps_fine, std::size(steps_fine), fixed_x, fixed_y, fixed_z, current_rz_deg, tcp_aa);

    // 运动到最终最佳位姿（tcp_aa 已包含最优平移+旋转）
    {
        double ref_j[ROB_AXIS_NUM];
        if (cr_get_jointActualPos((RobotHandle)robot_handle, ref_j) == sucess) {
            double joint_best[ROB_AXIS_NUM];
            if (cr_kineInverse((RobotHandle)robot_handle, tcp_aa, ref_j, joint_best) == sucess) {
                moveToJointPos(robot_handle, joint_best);                
                cv::Mat frame = camera->captureOneFrame(1500);
                cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
            }
        }
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

static bool computeZValues(int robot_handle,
    std::shared_ptr<AICamera>& camera,
    const cv::Ptr<cv::aruco::Dictionary>& dictionary,
    const cv::Ptr<cv::aruco::CharucoBoard>& board,
    const double ref_euler[6],   // 当前欧拉位姿（含 Z）
    const double ref_joint[6],   // 当前关节角
    double z_out[3]) {           // 输出 3 个验证后的 Z 值 (mm)

    double orig_j[ROB_AXIS_NUM];
    std::copy(ref_joint, ref_joint + ROB_AXIS_NUM, orig_j);

    // 在当前位姿检测标定板像素尺寸
    cv::Mat frame = camera->captureOneFrame(1500);
    if (frame.empty()) return false;

    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
    if (!detectCharucoCorners(frame, dictionary, board, corners, ids) || ids.size() < 4)
        return false;

    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    for (auto& pt : corners) {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
    }

    float board_w = max_x - min_x;
    float board_h = max_y - min_y;
    float board_dim = std::max(board_w, board_h);
    float img_w = (float)frame.cols;
    float img_h = (float)frame.rows;
    float img_dim = (board_w >= board_h) ? img_w : img_h;

    if (board_dim < 1.0f) { moveToJointPos(robot_handle, orig_j); return false; }

    double current_z = ref_euler[2];
    const double ratios[3] = {0.1, 0.2, 0.49};

    for (int i = 0; i < 3; i++) {
        double z_attempt = current_z * board_dim / (ratios[i] * img_dim);

        std::cout << "[autoCalib]  Z" << i << ": target_ratio=" << ratios[i]
                  << ", 初始Z=" << z_attempt << std::endl;

        bool converged = false;
        for (int iter = 0; iter < 20; iter++) {
            double euler[ROB_AXIS_NUM];
            std::copy(ref_euler, ref_euler + ROB_AXIS_NUM, euler);
            euler[2] = z_attempt;

            double aa[ROB_AXIS_NUM];
            if (cr_Eule2AxisAngle(euler, aa) != sucess) continue;

            double ref_j[ROB_AXIS_NUM];
            double joint[ROB_AXIS_NUM];
            if (cr_get_jointActualPos((RobotHandle)robot_handle, ref_j) != sucess ||
                cr_kineInverse((RobotHandle)robot_handle, aa, ref_j, joint) != sucess)
                continue;

            moveToJointPos(robot_handle, joint);            

            cv::Mat vf = camera->captureOneFrame(1500);
            if (vf.empty()) continue;

            std::vector<cv::Point2f> vc;
            std::vector<int> vi;
            if (!detectCharucoCorners(vf, dictionary, board, vc, vi) || vi.size() < 4)
                continue;

            float vx1 = std::numeric_limits<float>::max(), vx2 = -std::numeric_limits<float>::max();
            float vy1 = std::numeric_limits<float>::max(), vy2 = -std::numeric_limits<float>::max();
            for (auto& pt : vc) {
                vx1 = std::min(vx1, pt.x); vx2 = std::max(vx2, pt.x);
                vy1 = std::min(vy1, pt.y); vy2 = std::max(vy2, pt.y);
            }
            float actual_w = vx2 - vx1;
            float actual_h = vy2 - vy1;
            float actual_ratio = std::min(actual_w / img_w, actual_h / img_h);

            std::cout << "[autoCalib]    iter" << iter << ": Z=" << z_attempt
                      << ", ratio=" << actual_ratio
                      << " (w=" << actual_w/img_w << ", h=" << actual_h/img_h << ")"
                      << std::endl;

            if (std::abs(actual_ratio - ratios[i]) <= 0.03f) {
                double pose[ROB_AXIS_NUM];
                double eul[ROB_AXIS_NUM];
                if (cr_get_tcpActualPose((RobotHandle)robot_handle, pose) == sucess &&
                    cr_AxisAngle2Eule(pose, eul) == sucess) {
                    z_out[i] = eul[2];
                    std::cout << "[autoCalib]    → 收敛, Z=" << z_out[i] << std::endl;
                } else {
                    z_out[i] = z_attempt;
                    std::cout << "[autoCalib]    → 收敛, 读位姿失败用 Z=" << z_out[i] << std::endl;
                }
                converged = true;
                break;
            }

            // 自适应步长：error > 0 → 板太大 → Z↑；error < 0 → 板太小 → Z↓
            // 步长取 Z 的 30% × 误差比例，避免固定步长在近距区振荡
            double error = actual_ratio / ratios[i] - 1.0;
            double step = z_attempt * 0.3 * fabs(error);
            step = std::max(step, 10.0);   // 最小步长 10mm
            step = std::min(step, 100.0);  // 最大步长 100mm
            z_attempt += (error > 0) ? step : -step;
            z_attempt = std::max(z_attempt, -300.0); // 最小 Z
        }

        if (!converged) {
            z_out[i] = z_attempt;
            std::cout << "[autoCalib]    → 未收敛, 使用 Z=" << z_out[i] << std::endl;
        }
    }

    // 回到原始位置
    moveToJointPos(robot_handle, orig_j);    

    return true;
}

// ============================================================
// Phase 3: 生成 729 候选 + IK 过滤
// ============================================================

bool generateCandidates(int robot_handle,
                        const double ref_euler[6],
                        const double ref_joint[6],
                        const double z_values[3],
                        std::vector<PoseCandidate>& candidates) {
    double euler[6], aa[6], joint[6];
    int total = 0, ik_ok = 0;
    
    for (int zi = 0; zi < 3; zi++) { 
        double z = z_values[zi], dlt = 360 * (3-zi)/3, ddeg[]={30, 20, 10};
        const double ang_offsets[] = {-ddeg[zi], 0.0, ddeg[zi]};
        const double xy_offsets[] = {-dlt, 0.0, dlt};
    for (double dx : xy_offsets) {
    for (double dy : xy_offsets) {
    
    for (double drx : ang_offsets) {
    for (double dry : ang_offsets) {
    for (double drz : ang_offsets) {
        total++;
        euler[0] = ref_euler[0] + dx;
        euler[1] = ref_euler[1] + dy;
        euler[2] = z;  // 绝对 Z 值（非偏移量）
        euler[3] = ref_euler[3] + drx;
        euler[4] = ref_euler[4] + dry;
        euler[5] = ref_euler[5] + drz;

        CRresult ret = cr_Eule2AxisAngle(euler, aa);
        if (ret != sucess) continue;

        ret = cr_kineInverse((RobotHandle)robot_handle, aa,
                              const_cast<double*>(ref_joint), joint);
        if (ret == sucess) {
            PoseCandidate cand;
            std::copy(aa, aa + 6, cand.axis_angle);
            std::copy(joint, joint + 6, cand.joint);
            candidates.push_back(cand);
            ik_ok++;
        }
    }}}}}}

    std::cout << "[autoCalib] 729 candidates generated, IK reachable: "
              << ik_ok << "/" << total << std::endl;
    return !candidates.empty();
}

// ============================================================
// Phase 4: 逐点运动 + 采集
// ============================================================

bool executeCollection(int robot_handle,
                       std::shared_ptr<AICamera>& camera,
                       const double tcp_offset[6],
                       const cv::Ptr<cv::aruco::Dictionary>& dictionary,
                       const cv::Ptr<cv::aruco::CharucoBoard>& board,
                       const std::vector<PoseCandidate>& candidates,
                       const std::string& save_dir) {
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

        moveToJointPos(robot_handle, candidates[i].joint);       

        cv::Mat frame = camera->captureOneFrame(1500);
        if (frame.empty()) {
            std::cerr << "\n[autoCalib] 采集图像失败 (pose " << i << ")" << std::endl;
            continue;
        }

        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detectCharucoCorners(frame, dictionary, board, corners, ids))
            continue;
        if(ids.size() < 8)
           continue;
        double tcp_pose[ROB_AXIS_NUM];
        CRresult ret = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_pose);
        if (ret != sucess) continue;

        double flange_pose[ROB_AXIS_NUM];
        ret = cr_TcpToFlangePose(tcp_pose,
              const_cast<double*>(tcp_offset), flange_pose);
        if (ret != sucess) continue;

        // 获取实际关节角
        double actual_joint[ROB_AXIS_NUM];
        cr_get_jointActualPos((RobotHandle)robot_handle, actual_joint);

        // 保存到磁盘
        {
            std::string img_name = save_dir + "/" +
                (captured < 10 ? "00" : captured < 100 ? "0" : "") +
                std::to_string(captured) + ".jpg";
            cv::imwrite(img_name, frame);
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

// ============================================================
// Phase 5a: 相机内参标定（从目录读取图像）
// ============================================================

static bool loadImagesFromDisk(const std::string& save_dir,
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
    // 从第一张图获取尺寸
    cv::Mat first = cv::imread(img_files[0]);
    if (first.empty()) return false;
    image_size = first.size();
    return true;
}

static bool calibrateCameraFromDisk(const std::string& save_dir,
                                     const cv::Ptr<cv::aruco::Dictionary>& dictionary,
                                     const cv::Ptr<cv::aruco::CharucoBoard>& board,
                                     cv::Mat& camera_matrix,
                                     cv::Mat& dist_coeffs) {

    std::vector<cv::Point3f> board_points;
#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
    board_points = board->getChessboardCorners();
#else
    board_points = board->chessboardCorners;
#endif

    std::vector<std::string> img_files;
    cv::Size image_size;
    if (!loadImagesFromDisk(save_dir, img_files, image_size))
        return false;

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;

    for (const auto& path : img_files) {
        cv::Mat img = cv::imread(path);
        if (img.empty()) continue;

        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detectCharucoCorners(img, dictionary, board, corners, ids))
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
        if (pts3.size() >= 6) {
            object_points.push_back(pts3);
            image_points.push_back(pts2);
        }
    }

    if (object_points.size() < 5) {
        std::cerr << "[autoCalib] 帧数不足，无法标定相机内参 (need >=5, got "
                  << object_points.size() << ")" << std::endl;
        return false;
    }

    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    std::vector<cv::Mat> rvecs, tvecs;
    double err = cv::calibrateCamera(object_points, image_points,
                                      image_size, camera_matrix, dist_coeffs,
                                      rvecs, tvecs);
    std::cout << "[autoCalib] 相机内参标定完成, 重投影误差: " << err << " px"
              << std::endl;
    std::cout << "[autoCalib] 相机矩阵:\n" << camera_matrix << std::endl;
    return true;
}

// ============================================================
// Phase 5b: 手眼标定（从目录读取图像 + 位姿）
// ============================================================

static bool calibrateHandEyeFromDisk(const std::string& save_dir,
                                      const cv::Mat& camera_matrix,
                                      const cv::Mat& dist_coeffs,
                                      const cv::Ptr<cv::aruco::CharucoBoard>& board,
                                      const cv::Ptr<cv::aruco::Dictionary>& dictionary,
                                      CalibrationResult& result) {
    // 读取法兰位姿
    std::string pos_file = save_dir + "/rob_pos.csv";
    std::ifstream pf(pos_file);
    if (!pf.is_open()) {
        std::cerr << "[autoCalib] 无法打开 " << pos_file << std::endl;
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
    std::vector<cv::Mat> robot_poses, camera_poses;

    for (size_t i = 0; i < n; i++) {
        cv::Mat img = cv::imread(img_files[i]);
        if (img.empty()) continue;

        cv::Mat rvec, tvec;
        std::vector<cv::Point2f> corners;
        std::vector<int> ids;

        if (!detectCharucoPose(img, dictionary, board,
                                camera_matrix, dist_coeffs,
                                rvec, tvec, corners, ids, 12))
            continue;

        robot_poses.push_back(flange_poses[i]);
        camera_poses.push_back(rvecTvecToTransform(rvec, tvec));
    }

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

    cv::Mat R_cam_gripper, t_cam_gripper;
    try {
        cv::calibrateHandEye(R_gripper2base, t_gripper2base,
                              R_target2cam, t_target2cam,
                              R_cam_gripper, t_cam_gripper,
                              cv::CALIB_HAND_EYE_TSAI);
    } catch (const cv::Exception& e) {
        result.message = "手眼标定异常: " + std::string(e.what());
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    // 合成 4x4 矩阵
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R_cam_gripper.copyTo(T(cv::Rect(0, 0, 3, 3)));
    t_cam_gripper.copyTo(T(cv::Rect(3, 0, 1, 3)));

    result.T_flange_camera = T.clone();
    result.camera_matrix = camera_matrix.clone();
    result.dist_coeffs = dist_coeffs.clone();

    // 评估：T_base_calib = T_base_flange * X * T_camera_calib 的平移一致性
    // 标定板相对基坐标系固定，各位姿下的 T_base_calib 应相同
    std::vector<cv::Mat> T_base_calibs(valid);
    for (int i = 0; i < valid; i++)
        T_base_calibs[i] = robot_poses[i] * T * camera_poses[i];

    cv::Mat mean_t(3, 1, CV_64F, cv::Scalar(0));
    for (int i = 0; i < valid; i++)
        mean_t += T_base_calibs[i](cv::Rect(3, 0, 1, 3));
    mean_t /= valid;

    double sum_err = 0.0;
    for (int i = 0; i < valid; i++) {
        cv::Mat d = T_base_calibs[i](cv::Rect(3, 0, 1, 3)) - mean_t;
        sum_err += d.dot(d);
    }
    result.reproj_error = std::sqrt(sum_err / valid);
    result.success = true;
    result.message = "标定成功, 有效帧: " + std::to_string(valid) +
                     ", 平均误差: " + std::to_string(result.reproj_error * 1000.0) + " mm";

    std::cout << "[autoCalib] " << result.message << std::endl;
    std::cout << "[autoCalib] T_flange_camera:\n" << T << std::endl;
    return true;
}

} // anonymous namespace

// ============================================================
// 公开 API
// ============================================================

bool autoEyeInHandCharucoCalibration(
    int robot_handle,
    const double init_tcp_pose[6],
    const double tcp_offset[6],
    std::shared_ptr<AICamera>& camera,
    const CharucoBoardConfig& board_config,
    cv::InputOutputArray camera_matrix,
    cv::InputOutputArray dist_coeffs,
    CalibrationResult& result,
    const std::string& save_dir) {

    std::cout << "\n========== 自动手眼标定开始 ==========" << std::endl;

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::CharucoBoard> board;
    if (!createCharucoBoard(board_config, dictionary, board)) {
        result.message = "创建 ChArUco 标定板失败";
        return false;
    }

    // ---- Phase 1b: 自动寻找最佳位姿 ----
    double tcp_aa[ROB_AXIS_NUM];
    std::copy(init_tcp_pose, init_tcp_pose + ROB_AXIS_NUM, tcp_aa);

    std::cout << "[autoCalib] 自动寻找最佳位姿..." << std::endl;
    if (!autoFindBestPose(robot_handle, camera, dictionary, board,
                           tcp_aa, tcp_offset)) {
        std::cerr << "[autoCalib] 自动寻找最佳位姿失败，使用初始位姿" << std::endl;
    }    

    // ---- Phase 2: IK 参考关节角 ----
    double ref_joint[ROB_AXIS_NUM] = {0};
    CRresult ret = cr_get_jointActualPos((RobotHandle)robot_handle, ref_joint);
    if (ret != sucess) {
        result.message = "获取关节角失败";
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    // 轴角 → 欧拉参考位姿
    double ref_euler[ROB_AXIS_NUM] = {0};
    ret = cr_AxisAngle2Eule(tcp_aa, ref_euler);
    if (ret != sucess) {
        result.message = "轴角转欧拉角失败";
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    std::cout << "[autoCalib] 参考欧拉位姿: "
              << ref_euler[0] << ", " << ref_euler[1] << ", " << ref_euler[2] << ", "
              << ref_euler[3] << ", " << ref_euler[4] << ", " << ref_euler[5]
              << std::endl;    

    // ---- 计算动态 Z 值（标定板边长占图像 20%, 40%, 60%） ----
    double z_values[3] = {ref_euler[2] - 300.0, ref_euler[2], ref_euler[2] + 300.0};
    {
        double computed_z[3];
        if (computeZValues(robot_handle, camera, dictionary, board,
                           ref_euler, ref_joint, computed_z)) {
            std::copy(computed_z, computed_z + 3, z_values);
            std::cout << "[autoCalib] 动态 Z 值: "
                      << z_values[0] << ", " << z_values[1] << ", " << z_values[2] << std::endl;
        } else {
            std::cout << "[autoCalib] Z 值计算失败，使用固定偏移 [-300,0,+300]" << std::endl;
        }
    }

    // ---- Phase 3: 生成候选 + IK 过滤 ----
    std::vector<PoseCandidate> candidates;
    if (!generateCandidates(robot_handle, ref_euler, ref_joint, z_values, candidates)) {
        result.message = "无 IK 可达位姿";
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    // ---- Phase 4: 逐点运动 + 采集（保存到磁盘） ----
    if (!executeCollection(robot_handle, camera, tcp_offset,
                            dictionary, board, candidates, save_dir)) {
        result.message = "未采集到有效数据";
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    // ---- Phase 5: 标定（直接从磁盘读取） ----
    cv::Mat K, D;
    if (camera_matrix.empty() || dist_coeffs.empty()) {
        std::cout << "[autoCalib] 相机内参未知，从磁盘标定..." << std::endl;
        if (!calibrateCameraFromDisk(save_dir, dictionary, board, K, D)) {
            result.message = "相机内参标定失败";
            std::cerr << "[autoCalib] " << result.message << std::endl;
            return false;
        }
    } else {
        camera_matrix.getMat().copyTo(K);
        dist_coeffs.getMat().copyTo(D);
        std::cout << "[autoCalib] 使用已有相机内参" << std::endl;
    }

    if (!calibrateHandEyeFromDisk(save_dir, K, D, board, dictionary, result))
        return false;

    K.copyTo(camera_matrix);
    D.copyTo(dist_coeffs);

    std::cout << "========== 自动手眼标定完成 ==========\n" << std::endl;
    return true;
}
