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
#include "utils/calib_transform.hpp"
#include "calibration_utils.hpp"
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
            int call_ret = g_callback(nullptr, 0, 1);
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
            int call_ret = g_callback(nullptr, 0, 1);
            if (-1==call_ret)
            {
                std::cerr << "\r[autoCalib] 用户取消了  采集进度"  << std::endl;

                return;
            }            
        }           

    }
    moveToTcpAxisPos(robot_handle, tcp_aa);    
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

// ============================================================
// 计算并验证 3 个候选 Z 值：迭代逼近目标边长比
// ============================================================

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
            return false;
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
