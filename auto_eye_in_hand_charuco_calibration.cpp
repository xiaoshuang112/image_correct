#include "auto_eye_in_hand_charuco_calibration.hpp"

#include "AICamera.hpp"
#ifdef useORB
#include <OrbbecCamera.hpp>
#endif // useORB

#ifdef useOAK
#include "OAKCamera.hpp"
#endif // useOAK

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

// 运动到目标关节角（等效 goto_jointpos）
bool moveToJointPos(int robot_handle, const double joint[6]) {
    double pose[6] = {0, 0, 0, 0, 0, 0};
    cr_get_jointActualPos(robot_handle, pose);
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
        return false;
    }
    ret = cr_move_joint((RobotHandle)robot_handle, para, TRUE);
    if (ret != sucess) {
        std::cerr << "[autoCalib] cr_move_joint failed: " << ret << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return true;
}

// 这些函数定义在匿名命名空间之后（全局作用域），此处前向声明
bool moveToTcpAxisPos(int robot_handle, double* anglePos);
bool convertFlangeOulePos2Joint(int robot_handle, double* flangeEulePos, double *joint);
bool moveToFlangeEulePos(int robotHandle, double *flangeEulePos);
bool convertTcpAxisPos2FlangPos(int robotHandle, double *tcpAnglePos, double *flangeAnglePos);
bool convertTcpAxisPos2FlangeEule(int robot_handle, double *tcpAxisPos, double *flangeEulePos);

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


void filterByCharucoReprojectionError(
     const cv::Ptr<cv::aruco::CharucoBoard>& board,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    std::vector<cv::Point2f>& charuco_corners,
    std::vector<int>& charuco_ids,
    const cv::Mat& rvec,
    const cv::Mat& tvec, float thresh)  {

    std::vector<cv::Point3f> object_points;
    object_points.reserve(charuco_ids.size());

    
#if CV_MAJOR_VERSION >= 4 && CV_VERSION_MINOR >= 7
    // OpenCV 4.7.0+：使用 getter 方法
    const auto& board_object_points = board->getChessboardCorners();
#else
    // OpenCV 4.6.x 及以下：直接访问成员变量
    const auto& board_object_points = board->chessboardCorners;
#endif


    for (size_t i = 0; i < charuco_ids.size(); ++i) {
        const int id = charuco_ids[i];
        if (id < 0 || static_cast<size_t>(id) >= board_object_points.size()) {
            continue;
        }
        object_points.push_back(board_object_points[static_cast<size_t>(id)]);
    }

    if (object_points.size() != charuco_corners.size() || object_points.empty()) {
        return;
    }

    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

    double sum_err = 0.0;
    double sum_sq_err = 0.0;
    double max_err = 0.0;
    std::vector<float> vError;
    for (size_t i = 0; i < projected_points.size(); ++i) {
        const cv::Point2f& p_det = charuco_corners[i];
        const cv::Point2f& p_proj = projected_points[i];
        const double err = cv::norm(p_det - p_proj);

        sum_err += err;
        sum_sq_err += err * err;
        max_err = std::max(max_err, err);
        vError.push_back(err);
        // cv::circle(image_with_detection, p_proj, 3, cv::Scalar(0, 0, 255), -1);
        // cv::line(image_with_detection, p_det, p_proj, cv::Scalar(0, 255, 255), 1);
    }

    const double mean_err = sum_err / static_cast<double>(projected_points.size());
    const double rms_err = std::sqrt(sum_sq_err / static_cast<double>(projected_points.size()));

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "Reproj(px) mean=" << mean_err << " rms=" << rms_err << " max=" << max_err;
    // cv::putText(image_with_detection, oss.str(), cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
     // filter outer
    if (thresh > 0.01f && !charuco_ids.empty()){
        for (int i = static_cast<int>(charuco_ids.size()) - 1; i >= 0; i--) {
            if (vError[i] >= thresh) {
                charuco_ids.erase(charuco_ids.begin() + i);
                charuco_corners.erase(charuco_corners.begin() + i);                
            }
        }
    }
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
    if (ids.size() < minCornersNum) 
        return false;
     auto threshes = (image.rows == 1280) ? std::array{0.3f, 0.2f} : std::array{0.6f, 0.4f}; 
    

    if(cv::aruco::estimatePoseCharucoBoard(
        corners, ids, board, camera_matrix, dist_coeffs, rvec, tvec)){
        filterByCharucoReprojectionError(
        board, camera_matrix, dist_coeffs, corners, ids,  rvec, tvec, threshes[0]); // 0.3 = 1280
        if (ids.size() < minCornersNum) 
           return false;
        }
    
    if(cv::aruco::estimatePoseCharucoBoard(
        corners, ids, board, camera_matrix, dist_coeffs, rvec, tvec)){
        filterByCharucoReprojectionError(
        board, camera_matrix, dist_coeffs, corners, ids,  rvec, tvec, threshes[1]); // 0.2 = 1280
        if (ids.size() < minCornersNum) 
           return false;
        }

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

        if (nullptr!=g_callback) {
            // 获取当前时间点
            auto now = std::chrono::high_resolution_clock::now();

            // 将时间转换为自纪元以来的纳秒数
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
    const cv::Ptr<cv::aruco::Dictionary>& dictionary,
    const cv::Ptr<cv::aruco::CharucoBoard>& board,
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
        std::tie(s, n, bc) = evaluatePose(camera, dictionary, board, use_center);
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
    const cv::Ptr<cv::aruco::Dictionary>& dictionary,
    const cv::Ptr<cv::aruco::CharucoBoard>& board,
    const double* steps, int num_steps,    
    double tcp_aa[6]) {
    cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);

    double flangeEulePos[ROB_AXIS_NUM];    
    convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEulePos);
    // CRresult ret = cr_AxisAngle2Eule(tcp_aa, flangeEulePos);
    
    for (int i = 0; i < num_steps; i++) {
        double step = steps[i];
        double score; int n; cv::Point2f bc;
        std::tie(score, n, bc) = evaluatePose(camera, dictionary, board, false);        
        std::vector<std::array<double, 6>> xy_candidates;
        for (int sx : {-1, 0, 1}) for (int sy : {-1, 0, 1}) {
            if (sx == 0 && sy == 0) 
               continue;
            xy_candidates.push_back({flangeEulePos[0] + sx*step, flangeEulePos[1] + sy*step, flangeEulePos[2],
                                      flangeEulePos[3], flangeEulePos[4], flangeEulePos[5]});
        }
        double bestFlangeEule[ROB_AXIS_NUM];
        if (tryCandidatePoses(robot_handle, camera, dictionary, board,
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
                      const cv::Ptr<cv::aruco::Dictionary>& dictionary,
                      const cv::Ptr<cv::aruco::CharucoBoard>& board,
                      double tcp_aa[6],        // in/out: TCP 轴角 (mm,deg)
                      const double tcp_offset[6],
                      const std::string& save_dir,
                      AI_det_callback callback) {

    const double TARGET_AREA_RATIO = 0.275;
    const int MAX_REFINE_ZRZ = 6;
    const double Z_STEP = 100.0;
    const double Z_MAX_OFFSET = 600.0;

    // ---- 固定 rx=-180°, ry=0°（标定板水平），保留 rz ----
    double flangeEulePos[ROB_AXIS_NUM];   
    rotateFlangeToHorizon(robot_handle, flangeEulePos);    
    cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);          

    // 标定板总角点数
    cv::Size board_sz =
#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
        board->getChessboardSize();
#else
        board->checkboradSize;
#endif
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
    std::tie(cur_score, cur_codes, cur_bc) = evaluatePose(camera, dictionary, board, false);

    //if (cur_codes > 0 && cur_codes < total_corners) 
    {
        std::cout << "[autoFindBestPose] Phase1: Z 搜索 (当前码=" << cur_codes
                  << "/" << total_corners << ")" << std::endl;

        std::vector<std::array<double, 6>> z_candidates;
        for (int idx = 1; idx * Z_STEP <= Z_MAX_OFFSET; idx++)
            z_candidates.push_back({flangeEulePos[0], flangeEulePos[1], flangeEulePos[2] + idx * Z_STEP,
                                    flangeEulePos[3], flangeEulePos[4], flangeEulePos[5]});

        double bestFlangeEule[ROB_AXIS_NUM];
        double best_s = tryCandidatePoses(robot_handle, camera, dictionary, board,
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
    
    std::tie(cur_score, cur_codes, cur_bc) = evaluatePose(camera, dictionary, board, false);

    //if (cur_codes > 0 && cur_codes < total_corners)
     {
        std::cout << "[autoFindBestPose] Phase2: XY 搜索 (码=" << cur_codes
                  << "/" << total_corners << ")" << std::endl;
        const double steps[] = {240, 120, 60};
        searchXYSteps(robot_handle, camera, dictionary, board,
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
        if (!detectCharucoCorners(frame, dictionary, board, corners, ids) || ids.size() < 4) {
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
        std::tie(cur_s, n, bc) = evaluatePose(camera, dictionary, board, false);

        std::vector<std::array<double, 6>> zrz_candidates;
        for (double frac = 0.5; frac <= 1.0 + 1e-9; frac += 0.5)
            zrz_candidates.push_back({flangeEulePos[0], flangeEulePos[1], flangeEulePos[2] + dZ * frac,
                                      flangeEulePos[3], flangeEulePos[4], bound_camera_angle(flangeEulePos[5] + dRz_deg * frac)});

        double bestFlangeEulePos[ROB_AXIS_NUM];
        if (tryCandidatePoses(robot_handle, camera, dictionary, board,
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
    searchXYSteps(robot_handle, camera, dictionary, board, steps_fine, std::size(steps_fine), tcp_aa);

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
    const cv::Ptr<cv::aruco::Dictionary>& dictionary,
    const cv::Ptr<cv::aruco::CharucoBoard>& board,    
    double z_out_[3]) {           // 输出 3 个验证后的 Z 值 (mm)
    double tcp_aa[ROB_AXIS_NUM], tcpEulePos[ROB_AXIS_NUM], oriFlangeEulePos[ROB_AXIS_NUM], outZ[3];
    //double orig_joint[ROB_AXIS_NUM];
    cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);
    convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, oriFlangeEulePos);
    cr_AxisAngle2Eule(tcp_aa, tcpEulePos);
    z_out_[0] = oriFlangeEulePos[2] - 100.0;
    z_out_[1] = oriFlangeEulePos[2] + 100.0;
    z_out_[2] = oriFlangeEulePos[2] + 300.0;
    memcpy(outZ, z_out_, sizeof(outZ));
    

    // 在当前位姿检测标定板像素尺寸
    cv::Mat frame, depth; 
    for(int i = 0; i < 3; i++){
      camera->captureOneFrameWithDepth(frame, depth);
      if(depth.cols > 0 ) break;
    }
    if (frame.empty() || depth.empty()){
       std::cout << "[computeZValues] ChArUco 获取深度图像失败" << std::endl;  
       return false;
    } 

    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
    if (!detectCharucoCorners(frame, dictionary, board, corners, ids) || ids.size() < 4){
        std::cout << "[computeZValues] ChArUco 初始图的角点过少" << std::endl;  
        return false;
    }

    //{
        double heightCamera = computeAvgDepth(depth, corners);
        if (heightCamera > 0.0)
            std::cout << "[autoCalib] ChArUco 区域平均深度: " << heightCamera << " mm" << std::endl;
    //}

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

    if (board_dim < 1.0f) { 
         moveToFlangeEulePos(robot_handle, oriFlangeEulePos); 
         return false; }

    double current_z = oriFlangeEulePos[2];
    const double ratios[3] = {0.08, 0.16, 0.30};
    double z_board = current_z - heightCamera;//, heightTcp = heightCamera - cameraToTcpEnd;
    double minH = 1.5 * cameraToTcpEnd; // Avoid TCP collision with calibration plate 

    for (int i = 0; i < 3; i++) {
        
        double z_attempt = std::max(minH, heightCamera/2 * board_dim / (ratios[i] * img_dim)) + z_board;

        std::cout << "[autoCalib]  Z" << i << ": target_ratio=" << ratios[i]
                  << ", 初始Z=" << z_attempt << std::endl;

        bool converged = false;
        double best_z = z_attempt;
        double best_ratio_err = 1e18;
        for (int iter = 0; iter < 20; iter++) {
            double euler[ROB_AXIS_NUM];
            //double joint[ROB_AXIS_NUM];
            std::copy(oriFlangeEulePos, oriFlangeEulePos + ROB_AXIS_NUM, euler);
            euler[2] = z_attempt;
            if(!moveToFlangeEulePos(robot_handle, euler))
               continue;           

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
            float actual_ratio = std::max(actual_w / img_w, actual_h / img_h);

            double ratio_err = fabs(actual_ratio - ratios[i]);
            if (ratio_err < best_ratio_err) {
                best_ratio_err = ratio_err;
                best_z = z_attempt;
            }

            std::cout << "[autoCalib]    iter" << iter << ": Z=" << z_attempt
                      << ", ratio=" << actual_ratio
                      << " (w=" << actual_w/img_w << ", h=" << actual_h/img_h << ")"
                      << std::endl;

            if (ratio_err <= 0.03f) {
                double pose[ROB_AXIS_NUM];
                double flangeEule[ROB_AXIS_NUM];
                if (cr_get_tcpActualPose((RobotHandle)robot_handle, pose) == sucess)
                {
                    convertTcpAxisPos2FlangeEule(robot_handle, pose, flangeEule);
                    outZ[i] = flangeEule[2];
                    std::cout << "[autoCalib]    → 收敛, Z=" << outZ[i] << std::endl;
                } else {
                    outZ[i] = z_attempt;
                    std::cout << "[autoCalib]    → 收敛, 读位姿失败用 Z=" << outZ[i] << std::endl;
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
            z_attempt = z_board + std::max(z_attempt- z_board, minH); // 最小 Z
        }

        if (!converged) {
            outZ[i] = best_z;
            std::cout << "[autoCalib]    → 未收敛, 使用最佳 Z=" << best_z
                      << " (ratio_err=" << best_ratio_err << ")" << std::endl;
        }
    }

    // 回到原始位置
    moveToFlangeEulePos(robot_handle, oriFlangeEulePos);    
    memcpy(z_out_, outZ, sizeof(outZ));

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
        double z = z_values[zi], dlt = 360 * (3-zi)/3, ddeg[]={25, 20, 15};
        const double ang_offsets[] = {-ddeg[zi], 0.0, ddeg[zi]};
        const double xy_offsets[] = {-dlt, 0.0, dlt};
    for (double dx : xy_offsets) {
    for (double dy : xy_offsets) {
    
    for (double drx : ang_offsets) {
    for (double dry : ang_offsets) {
    for (double drz : ang_offsets) {
        total++;
        euler[0] = flangeEulePos[0] + dx;
        euler[1] = flangeEulePos[1] + dy;
        euler[2] = z;  // 绝对 Z 值（非偏移量）
        euler[3] = flangeEulePos[3] + drx;
        euler[4] = flangeEulePos[4] + dry;
        euler[5] = flangeEulePos[5] + drz;

        if(convertFlangeOulePos2Joint(robot_handle, euler, joint)){        
            // moveToFlangeEulePos(robot_handle, euler);
            // frame = camera->captureOneFrame(1500);
            // moveToJointPos(robot_handle, joint);
            // frame = camera->captureOneFrame(1500);
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
        
       std::cout << "\n[reconnectRobotHandle] wait to re-connect robot handle: " << ", waiting " << waitingTime << " milliseconds for " << try_time <<" connect\n" << std::flush;
       std::this_thread::sleep_for(std::chrono::milliseconds(waitingTime));          
       result = cr_create_robot(&old_handle, pchRobotIP, 2323, "123");  // 机械臂连接        
       try_time ++;
    }
    return old_handle;
}

// ============================================================
// Phase 4: 逐点运动 + 采集
// ============================================================

bool executeCollection(int robot_handle,
                       char* pchRobotIP,
                       std::shared_ptr<AICamera>& camera,
                       const double tcp_offset[6],
                       const cv::Ptr<cv::aruco::Dictionary>& dictionary,
                       const cv::Ptr<cv::aruco::CharucoBoard>& board,
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

        if(false == moveToJointPos(robot_handle, candidates[i].joint)){
        //    double init_tcp_pose[6] = {-753.377,113.607,178.971, 6.5796, -4.09077, -78.0137}, euler_fixed[6];  
        //     moveToTcpAxisPos(robot_handle, init_tcp_pose);
        //     cv::Mat frame = camera->captureOneFrame(1000);
            //int waitingTime = 60000;
            std::cout << "\n[autoCalib] failure to move: " << i << "/" << total << ", waiting for reconnect\n" << std::flush;
            //std::this_thread::sleep_for(std::chrono::milliseconds(waitingTime));           
            robot_handle = reconnectRobotHandle(robot_handle, pchRobotIP);
            freeRobotHandle = 1;
            continue;
        }       

        cv::Mat frame = camera->captureOneFrame(1500);
        if (frame.empty()) {
            std::cerr << "\n[autoCalib] 采集图像失败 (pose " << i << ")" << std::endl;
            continue;
        }

        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detectCharucoCorners(frame, dictionary, board, corners, ids))
            continue;
        if(ids.size() < 12)
           continue;
        double tcp_pose[ROB_AXIS_NUM];
        CRresult ret = cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_pose);
        if (ret != sucess) continue;

        double flange_pose[ROB_AXIS_NUM];
        ret = cr_TcpToFlangePose(tcp_pose, const_cast<double*>(tcp_offset), flange_pose);
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
    std::cout << "[autoCalib] T_flange_camera:\n" << T << std::endl;

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
        for(int i=valid-1; i>=0; i--){
            if(vErr[i] > 0.025){
                robot_poses.erase(robot_poses.begin() + i);
                camera_poses.erase(camera_poses.begin() + i);
                removed++;
            }
       }
        if (removed > 0) {
            std::cout << "[autoCalib] 离群剔除: " << removed
                      << " 帧 (阈值 " << 20 << "mm), 剩余 " << robot_poses.size() << " 帧" << std::endl;
        }
    }

    return true;
}

// ============================================================
// Phase 5b: 手眼标定（从目录读取图像 + 位姿）
// ============================================================

static bool calibrateHandEyeFromDisk(const std::string& save_dir,
                                      cv::Mat& camera_matrix,
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
    // camera_matrix = camera_matrix * 2; // scale for fine-resolution
    // camera_matrix.at<double>(2, 2) = 1;

    for (size_t i = 0; i < n; i++) {
        cv::Mat img = cv::imread(img_files[i]);
        if (img.empty()) continue;
        // cv::resize(img, img, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR); // scale for fine-resolution
        cv::Mat rvec, tvec;
        std::vector<cv::Point2f> corners;
        std::vector<int> ids;

        if (!detectCharucoPose(img, dictionary, board,
                                camera_matrix, dist_coeffs,
                                rvec, tvec, corners, ids, 16))
            continue;

        robot_poses.push_back(flange_poses[i]);
        camera_poses.push_back(rvecTvecToTransform(rvec, tvec));
    }
    bool ret = estimateHandEyeTrans(camera_matrix, dist_coeffs, robot_poses, camera_poses, result, true);
    
    ret = estimateHandEyeTrans(camera_matrix, dist_coeffs, robot_poses, camera_poses, result, false);    
    return ret;
}

} // anonymous namespace

// ============================================================
// 辅助函数（全局可见）
// ============================================================

bool convertFlangeAxisPos2Joint(int robot_handle, double* flangeAnglePos, double *joint){
    double ref_j[ROB_AXIS_NUM];
    if (cr_get_jointActualPos((RobotHandle)robot_handle, ref_j) != sucess)
            return false;
    if (cr_kineInverse((RobotHandle)robot_handle, flangeAnglePos, ref_j, joint) != sucess)
            return false;
    return true;
}

bool convertTcpAxisPos2FlangPos(int robotHandle, double *tcpAnglePos, double *flangeAnglePos){
    TCPMsg tcpMsgList;
    if( cr_get_currentTCPmsg(robotHandle, &tcpMsgList) == sucess )   {
      if(cr_TcpToFlangePose(tcpAnglePos, tcpMsgList.tcpOffset, flangeAnglePos) == sucess)
        return true;
    }
    return false;
}

bool convertFlangeOulePos2Joint(int robot_handle, double* flangeEulePos, double *joint){
     double flangeAxisPos[ROB_AXIS_NUM];
     if (cr_Eule2AxisAngle(flangeEulePos, flangeAxisPos) != sucess)
            return false;    
     return convertFlangeAxisPos2Joint(robot_handle, flangeAxisPos, joint);
}

bool convertTcpAxisPos2FlangeEule(int robot_handle, double *tcpAxisPos, double *flangeEulePos){
    double flangeAxisPos[ROB_AXIS_NUM];
    if( convertTcpAxisPos2FlangPos(robot_handle, tcpAxisPos, flangeAxisPos) ){
      cr_AxisAngle2Eule(flangeAxisPos, flangeEulePos);
      return true;
    }
    return false;
}

bool moveToFlangeAxisPos(int robot_handle, double* axisAnglePos){
   double joint[6];
   if(convertFlangeAxisPos2Joint(robot_handle, axisAnglePos, joint)){
      moveToJointPos(robot_handle, (double*)joint);
      return true;
   }
   return false;
}

bool moveToTcpAxisPos(int robotHandle, double *tcpAnglePos){    
    double flangeAxisAnglePos[6];
    if(convertTcpAxisPos2FlangPos(robotHandle, tcpAnglePos, flangeAxisAnglePos) )
       return moveToFlangeAxisPos(robotHandle, flangeAxisAnglePos);
    return false;
}

bool moveToFlangeEulePos(int robotHandle, double *flangeEulePos){
   double flangeAxisAnglePos[6];
   cr_Eule2AxisAngle(flangeEulePos, flangeAxisAnglePos);
   return moveToFlangeAxisPos(robotHandle, flangeAxisAnglePos);
   //return true;
}


// ============================================================
// 公开 API
// ============================================================

bool calibrateHandEyeFromDisk(
    const std::string& save_dir,
    cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const CharucoBoardConfig& board_config,
    CalibrationResult& result) {

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::CharucoBoard> board;
    if (!createCharucoBoard(board_config, dictionary, board)) {
        result.message = "创建 ChArUco 标定板失败";
        return false;
    }

    return calibrateHandEyeFromDisk(save_dir, camera_matrix, dist_coeffs,
                                     board, dictionary, result);
}

bool autoEyeInHandCharucoCalibration(
    int robot_handle,    
    char* pchRobotIP,
    const double tcp_offset[6],
    std::shared_ptr<AICamera>& camera,
    const CharucoBoardConfig& board_config,
    cv::InputOutputArray camera_matrix,
    cv::InputOutputArray dist_coeffs,
    int cameraToTcpEnd,
    CalibrationResult& result,
    const std::string& save_dir,
    AI_det_callback callback) {

    g_save_dir = save_dir;
    g_callback = callback;

    std::cout << "\n========== 自动手眼标定开始 ==========" << std::endl;

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::CharucoBoard> board;
    if (!createCharucoBoard(board_config, dictionary, board)) {
        result.message = "创建 ChArUco 标定板失败";
        return false;
    }

    // ---- Phase 1b: 自动寻找最佳位姿 ----
    double tcp_aa[ROB_AXIS_NUM];
    //std::copy(init_tcp_pose, init_tcp_pose + ROB_AXIS_NUM, tcp_aa);
    rotateFlangeToHorizon(robot_handle, NULL);    
    cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa);          
    std::cout << "[autoCalib] 自动寻找最佳位姿..." << std::endl;
    if (!autoFindBestPose(robot_handle, camera, dictionary, board,
                           tcp_aa, tcp_offset,save_dir,callback)) {
        std::cerr << "[autoCalib] 自动寻找最佳位姿失败，使用初始位姿" << std::endl;
        return false;
    }    

    // ---- Phase 2: IK 参考关节角 ----    
    
    // ---- 计算动态 Z 值（标定板边长占图像 20%, 40%, 60%） ----
    double z_values[3];// = {flangeEulePos[2] - 100.0, flangeEulePos[2]+100, flangeEulePos[2] + 300.0};
    {
        if (computeZValues(robot_handle, camera, cameraToTcpEnd, dictionary, board, z_values)) {    
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

    // ---- Phase 4: 逐点运动 + 采集（保存到磁盘） ----
    if (!executeCollection(robot_handle, pchRobotIP, camera, tcp_offset,
                            dictionary, board, candidates, save_dir,callback)) {
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
