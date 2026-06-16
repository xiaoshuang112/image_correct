#include "eye_to_hand_calibration.hpp"
#include "auto_eye_in_hand_charuco_calibration.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <array>
#include <cmath>
#include <filesystem>
#include <thread>
#include <chrono>

#include "utils/robot_util.hpp"
#include "utils/calib_transform.hpp"

using namespace calib_transform;

// ============================================================
// 眼在手外手眼矩阵精化
// ============================================================
// 对于眼在手外，T_base_camera * T_camera_calib_i = T_base_flange_i * T_flange_calib
// 精化目标：Z_i = inv(T_base_flange_i) * X * T_camera_calib_i 的平移方差最小
// 其中 X = T_base_camera, Z_i = T_flange_calib（应为常量）
static cv::Mat refineEyeToHandConsistency(
    const cv::Mat& X_init,
    const std::vector<cv::Mat>& robot_poses,   // T_base_flange
    const std::vector<cv::Mat>& camera_poses) { // T_camera_calib

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

    // Z_i = inv(T_base_flange_i) * X * T_camera_calib_i
    auto eval = [&](const cv::Mat& X,
                    std::vector<cv::Mat>& residuals,
                    cv::Mat& mean_t) -> double {
        std::vector<cv::Mat> tvecs(n);
        mean_t = cv::Mat::zeros(3, 1, CV_64F);
        for (int i = 0; i < n; i++) {
            cv::Mat inv_robot = invertTransform(robot_poses[i]);
            cv::Mat Z = inv_robot * X * camera_poses[i];
            tvecs[i] = Z(cv::Rect(3, 0, 1, 3)).clone();
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
    std::cout << "[eyeToHand/refine] 初始方差: " << (prev_cost * 1e6) << " mm²" << std::endl;

    for (int iter = 0; iter < max_iters; iter++) {
        cv::Mat R_full(3 * n, 1, CV_64F);
        for (int i = 0; i < n; i++) {
            R_full.at<double>(3 * i,     0) = res[i].at<double>(0);
            R_full.at<double>(3 * i + 1, 0) = res[i].at<double>(1);
            R_full.at<double>(3 * i + 2, 0) = res[i].at<double>(2);
        }

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

        cv::Mat H = J_full.t() * J_full;
        cv::Mat g = J_full.t() * R_full;
        for (int j = 0; j < 6; j++)
            H.at<double>(j, j) += lambda;

        cv::Mat dp;
        if (!cv::solve(H, -g, dp, cv::DECOMP_SVD))
            break;

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
                std::cout << "[eyeToHand/refine]   iter " << iter << ": "
                          << (cost * 1e6) << " mm²" << std::endl;
            if (std::abs(prev_cost - cost) < 1e-14 || prev_cost == 0.0)
                break;
            prev_cost = cost;
        } else {
            lambda *= 3.0;
        }
    }

    std::cout << "[eyeToHand/refine] 精化后方差: " << (prev_cost * 1e6) << " mm²" << std::endl;
    return X;
}

// ============================================================
// Phase 2: 位姿生成 + IK 过滤
// ============================================================
static bool generateEyeToHandCandidates(
    int robot_handle,
    const double flangeEuleInit[6],   // 示教原点的法兰欧拉位姿
    const PoseGenerationConfig& config,
    std::vector<std::array<double, 6>>& candidates) {

    candidates.clear();

    // 按 config 循环生成偏移
    double euler[6], joint[6];
    for (double dx : config.offset_x) {
    for (double dy : config.offset_y) {
    for (double dz : config.offset_z) {
    for (double drx : config.offset_rx) {
    for (double dry : config.offset_ry) {
    for (double drz : config.offset_rz) {
        euler[0] = flangeEuleInit[0] + dx;
        euler[1] = flangeEuleInit[1] + dy;
        euler[2] = flangeEuleInit[2] + dz;
        euler[3] = flangeEuleInit[3] + drx;
        euler[4] = flangeEuleInit[4] + dry;
        euler[5] = flangeEuleInit[5] + drz;

        if (convertFlangeOulePos2Joint(robot_handle, euler, joint)) {
            std::array<double, 6> cand;
            std::copy(joint, joint + 6, cand.data());
            candidates.push_back(cand);
        }
    }}}}}}

    std::cout << "[eyeToHand] 位姿生成: " << candidates.size() << "/"
              << (config.offset_x.size() * config.offset_y.size() *
                  config.offset_z.size() * config.offset_rx.size() *
                  config.offset_ry.size() * config.offset_rz.size())
              << " IK 可达" << std::endl;

    return !candidates.empty();
}

// ============================================================
// Phase 3: 逐点采集
// ============================================================
static int executeEyeToHandCollection(
    int robot_handle,
    char* pchRobotIP,
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    const std::vector<std::array<double, 6>>& candidates,
    const std::string& save_dir,
    std::vector<cv::Mat>& robot_poses,    // T_base_flange
    int min_corners) {

    robot_poses.clear();

    if (save_dir.empty()) {
        std::cerr << "[eyeToHand] 保存目录为空" << std::endl;
        return 0;
    }

    // 准备保存目录
    {
        namespace fs = std::filesystem;
        if (fs::exists(save_dir))
            fs::remove_all(save_dir);
        fs::create_directories(save_dir);
    }
    std::string pos_file = save_dir + "/rob_pos.csv";
    std::string joint_file = save_dir + "/rob_joint.csv";

    int total = (int)candidates.size();
    int captured = 0;

    for (int i = 0; i < total; i++) {
        std::cout << "\r[eyeToHand] 采集进度: " << (i + 1) << "/" << total
                  << " (有效: " << captured << ")" << std::flush;

        CRresult ret = moveToJointPos(robot_handle, candidates[i].data());
        if (ret != sucess) {
            continue;
        }

        // 采集新帧（失败时重试一次，避免偶发坏帧）
        cv::Mat frame = camera->captureOneFrame(1500);
        if (frame.empty()) continue;

        // 仅检测角点（不需要相机内参），验证标定板可见
        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detector.detectCorners(frame, corners, ids)) {
            frame = camera->captureOneFrame(1500);
            if (frame.empty()) continue;
            if (!detector.detectCorners(frame, corners, ids))
                continue;
        }
        if ((int)ids.size() < min_corners)
            continue;

        // 获取实际 TCP 位姿
        double tcp_aa[6];
        if (cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa) != sucess)
            continue;

        // TCP 轴角 → 法兰轴角（与 axisAnglePoseToTransform 匹配）
        double flange_aa[6];
        if (!convertTcpAxisPos2FlangPos(robot_handle, tcp_aa, flange_aa))
            continue;

        // 保存图像
        {
            std::string img_name = save_dir + "/" +
                (captured < 10 ? "00" : captured < 100 ? "0" : "") +
                std::to_string(captured) + ".jpg";
            cv::imwrite(img_name, frame);
        }

        // 保存 pos（法兰轴角 mm,deg）
        {
            std::ofstream f(pos_file, std::ios::app);
            f << std::fixed << std::setprecision(3);
            for (int j = 0; j < 6; j++)
                f << flange_aa[j] << (j < 5 ? "," : "\n");
        }
        {
            std::ofstream f(joint_file, std::ios::app);
            f << std::fixed << std::setprecision(3);
            for (int j = 0; j < 6; j++)
                f << candidates[i][j] << (j < 5 ? "," : "\n");
        }

        // 保存内存中的位姿用于标定
        robot_poses.push_back(axisAnglePoseToTransform(flange_aa));
        captured++;
    }

    std::cout << "\n[eyeToHand] 采集完成，有效帧: " << captured << "/" << total << std::endl;
    return captured;
}

// ============================================================
// Phase 3.5: 从已采集的 ChArUco 图像标定相机内参
// ============================================================
// ============================================================
// Phase 3.75: 用标定后的相机内参计算每帧标定板位姿
// ============================================================
static bool computeBoardPosesFromImages(
    const std::string& save_dir,
    CharucoDetector& detector,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    std::vector<cv::Mat>& camera_poses) {

    camera_poses.clear();
    std::cout << "[eyeToHand] 计算标定板位姿..." << std::endl;

    // 临时设置相机内参用于 detectPose
    detector.setCameraParams(camera_matrix, dist_coeffs);

    for (int i = 0; ; i++) {
        std::ostringstream ss;
        ss << save_dir << "/" << (i < 10 ? "00" : i < 100 ? "0" : "") << i << ".jpg";
        std::string path = ss.str();

        cv::Mat img = cv::imread(path);
        if (img.empty()) break;

        cv::Mat rvec, tvec;
        std::vector<cv::Point2f> corners;
        std::vector<int> ids;
        if (!detector.detectPose(img, rvec, tvec, corners, ids, 4)) {
            std::cout << "[eyeToHand]   " << path << " 位姿估计失败，跳过" << std::endl;
            continue;
        }

        camera_poses.push_back(rvecTvecToTransform(rvec, tvec));
    }

    std::cout << "[eyeToHand] 成功计算 " << camera_poses.size() << " 帧位姿" << std::endl;
    return !camera_poses.empty();
}

// 手眼标定 + 一致性精化（返回 4x4 T_camera_base，失败返回空 Mat）
static cv::Mat runHandEyeCalibration(
    const std::vector<cv::Mat>& R_gripper2base,
    const std::vector<cv::Mat>& t_gripper2base,
    const std::vector<cv::Mat>& R_target2cam,
    const std::vector<cv::Mat>& t_target2cam,
    const std::vector<cv::Mat>& robot_poses,   // T_base_flange
    const std::vector<cv::Mat>& camera_poses,  // T_camera_calib
    cv::HandEyeCalibrationMethod method) {
    cv::Mat R_cam_gripper, t_cam_gripper;
    try {
        cv::calibrateHandEye(R_gripper2base, t_gripper2base,
                              R_target2cam, t_target2cam,
                              R_cam_gripper, t_cam_gripper,
                              method);
    } catch (const cv::Exception&) {
        return cv::Mat();
    }
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R_cam_gripper.copyTo(T(cv::Rect(0, 0, 3, 3)));
    t_cam_gripper.copyTo(T(cv::Rect(3, 0, 1, 3)));
    // return T;
    return refineEyeToHandConsistency(T, robot_poses, camera_poses);
}

// 评估眼在手外平移一致性
// Z_i = inv(T_base_flange_i) * X * T_camera_calib_i, 返回 RMS 平移偏差 (m)
static double evaluateTranslationConsistency(
    const std::vector<cv::Mat>& robot_poses,   // T_base_flange
    const std::vector<cv::Mat>& camera_poses,  // T_camera_calib
    const cv::Mat& X,                          // T_camera_base
    std::vector<double>& per_frame_errors) {   // output
    int n = (int)robot_poses.size();
    double sum_err = 0.0;
    cv::Mat mean_t(3, 1, CV_64F, cv::Scalar(0));
    for (int i = 0; i < n; i++) {
        cv::Mat inv_robot = invertTransform(robot_poses[i]);
        cv::Mat Z = inv_robot * X * camera_poses[i];
        mean_t += Z(cv::Rect(3, 0, 1, 3));
    }
    mean_t /= n;
    per_frame_errors.resize(n);
    for (int i = 0; i < n; i++) {
        cv::Mat inv_robot = invertTransform(robot_poses[i]);
        cv::Mat Z = inv_robot * X * camera_poses[i];
        cv::Mat d = Z(cv::Rect(3, 0, 1, 3)) - mean_t;
        per_frame_errors[i] = std::sqrt(d.dot(d));
        sum_err += d.dot(d);
    }
    return std::sqrt(sum_err / n);
}

// ============================================================
// Phase 4: 眼在手外标定
// ============================================================
static bool eyeToHandCalibrate(
    std::vector<cv::Mat>& robot_poses,   // T_base_flange (含离群过滤)
    std::vector<cv::Mat>& camera_poses,  // T_camera_calib (含离群过滤)
    EyeToHandResult& result) {

    int valid = (int)robot_poses.size();
    if (valid < 5) {
        result.message = "有效帧数不足: " + std::to_string(valid) + " (need >=5)";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }

    result.valid_frames = valid;

    // 眼在手外：传入 inv(T_base_flange) 作为 gripper2base
    // T_camera_calib 作为 target2cam
    // 输出 = T_camera_base (base 在 camera 下的位姿)
    std::vector<cv::Mat> R_gripper2base, t_gripper2base;
    std::vector<cv::Mat> R_target2cam, t_target2cam;
    for (int i = 0; i < valid; i++) {
        cv::Mat inv_robot = invertTransform(robot_poses[i]);   // T_flange_base
        R_gripper2base.push_back(inv_robot(cv::Rect(0, 0, 3, 3)).clone());
        t_gripper2base.push_back(inv_robot(cv::Rect(3, 0, 1, 3)).clone());
        R_target2cam.push_back(camera_poses[i](cv::Rect(0, 0, 3, 3)).clone());
        t_target2cam.push_back(camera_poses[i](cv::Rect(3, 0, 1, 3)).clone());
    }

    struct CalibMethodResult {
        std::string name;
        cv::Mat T;
        double reproj_error;
        std::vector<double> per_frame_errors;
    };
    std::vector<CalibMethodResult> calib_results;
    std::vector<cv::HandEyeCalibrationMethod> calib_methods = {
        cv::CALIB_HAND_EYE_TSAI, cv::CALIB_HAND_EYE_PARK, cv::CALIB_HAND_EYE_HORAUD};
    std::vector<std::string> calib_method_names = {"TSAI", "PARK", "HORAUD"};

    for (size_t m = 0; m < calib_methods.size(); m++) {
        cv::Mat T_refined = runHandEyeCalibration(
            R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
            robot_poses, camera_poses, calib_methods[m]);
        if (T_refined.empty()) {
            std::cerr << "[eyeToHand] " << calib_method_names[m] << " 标定失败" << std::endl;
            continue;
        }

        // 评估平移一致性
        std::vector<double> vErr;
        double rms = evaluateTranslationConsistency(robot_poses, camera_poses, T_refined, vErr);
        calib_results.push_back({calib_method_names[m], T_refined, rms, vErr});

        std::cout << "[eyeToHand] " << calib_method_names[m]
                  << " -> 平移一致性误差: " << (std::sqrt(rms) * 1000.0)
                  << " mm" << std::endl;
    }

    if (calib_results.empty()) {
        result.message = "所有手眼标定方法均失败";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }

    // 选误差最小的结果
    size_t best_idx = 0;
    for (size_t i = 1; i < calib_results.size(); i++) {
        if (calib_results[i].reproj_error < calib_results[best_idx].reproj_error)
            best_idx = i;
    }

    // ---- 离群过滤 ----
    {
        std::vector<double>& vErr = calib_results[best_idx].per_frame_errors;
        double thresh = 0.003;  // 2mm
        int removed = 0;
        for (int i = valid - 1; i >= 0; i--) {
            if (vErr[i] > thresh) {
                robot_poses.erase(robot_poses.begin() + i);
                camera_poses.erase(camera_poses.begin() + i);
                R_gripper2base.erase(R_gripper2base.begin() + i);
                t_gripper2base.erase(t_gripper2base.begin() + i);
                R_target2cam.erase(R_target2cam.begin() + i);
                t_target2cam.erase(t_target2cam.begin() + i);
                removed++;
            }
        }
        if (removed > 0) {
            std::cout << "[eyeToHand] 离群剔除: " << removed
                      << " 帧 (阈值 " << thresh * 1000.0 << "mm), 剩余 "
                      << robot_poses.size() << " 帧" << std::endl;
            valid = (int)robot_poses.size();
            if (valid < 5) {
                result.message = "离群过滤后有效帧数不足: " + std::to_string(valid);
                std::cerr << "[eyeToHand] " << result.message << std::endl;
                return false;
            }

            // 用过滤后的数据，以最优方法重新标定
            cv::Mat T_new = runHandEyeCalibration(
                R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
                robot_poses, camera_poses, calib_methods[best_idx]);
            if (!T_new.empty())
                calib_results[best_idx].T = T_new;
        }
    }

    valid = (int)robot_poses.size();
    cv::Mat T_camera_base = calib_results[best_idx].T.clone();

    // 重新评估剔除后的误差
    {
        std::vector<double> vErr;
        double rms_err = evaluateTranslationConsistency(robot_poses, camera_poses, T_camera_base, vErr);
        std::cout << "[eyeToHand] 剔除后误差: " << (rms_err * 1000.0)
                  << " mm (" << valid << " 帧)" << std::endl;

        // T_base_camera = inv(T_camera_base)
        cv::Mat T = invertTransform(T_camera_base);

        result.T_base_camera = T.clone();
        result.reproj_error = rms_err;
        result.valid_frames = valid;
        result.success = true;
        result.message = "标定成功, 有效帧: " + std::to_string(valid) +
                         ", 平均误差: " + std::to_string(rms_err * 1000.0) + " mm"
                         " (最优: " + calib_results[best_idx].name + ")";

        std::cout << "[eyeToHand] " << result.message << std::endl;
        std::cout << "[eyeToHand] T_base_camera:\n" << result.T_base_camera << std::endl;

        // 位姿分布分析
        {
            cv::Mat inv0 = invertTransform(robot_poses[0]);
            cv::Mat Z0 = inv0 * T_camera_base * camera_poses[0];
            double x_min = Z0.at<double>(0, 3) * 1000.0, x_max = x_min;
            double y_min = Z0.at<double>(1, 3) * 1000.0, y_max = y_min;
            double z_min = Z0.at<double>(2, 3) * 1000.0, z_max = z_min;
            for (int i = 1; i < valid; i++) {
                cv::Mat inv_i = invertTransform(robot_poses[i]);
                cv::Mat Z_i = inv_i * T_camera_base * camera_poses[i];
                double x = Z_i.at<double>(0, 3) * 1000.0;
                double y = Z_i.at<double>(1, 3) * 1000.0;
                double z = Z_i.at<double>(2, 3) * 1000.0;
                if (x < x_min) x_min = x; if (x > x_max) x_max = x;
                if (y < y_min) y_min = y; if (y > y_max) y_max = y;
                if (z < z_min) z_min = z; if (z > z_max) z_max = z;
            }
            std::cout << "[eyeToHand] 标定板在法兰坐标系下的空间分布:" << std::endl;
            std::cout << "[eyeToHand]   X: " << x_min << " ~ " << x_max << " mm, 跨度 " << (x_max - x_min) << " mm" << std::endl;
            std::cout << "[eyeToHand]   Y: " << y_min << " ~ " << y_max << " mm, 跨度 " << (y_max - y_min) << " mm" << std::endl;
            std::cout << "[eyeToHand]   Z: " << z_min << " ~ " << z_max << " mm, 跨度 " << (z_max - z_min) << " mm" << std::endl;
        }
    }

    return true;
}

// ============================================================
// 主入口
// ============================================================

bool autoEyeToHandCharucoCalibration(
    int robot_handle,
    char* pchRobotIP,
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    const PoseGenerationConfig& pose_config,
    EyeToHandResult& result,
    const std::string& save_dir) {

    std::cout << "\n========== 眼在手外 ChArUco 标定 ==========" << std::endl;

    // ---- Phase 1: 获取示教原点 ----
    double tcp_aa[6], flangeEuleInit[6];
    if (cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa) != sucess) {
        result.message = "获取机器人位姿失败";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }
    if (!convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEuleInit)) {
        result.message = "轴角转法兰欧拉失败";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }
    std::cout << "[eyeToHand] 示教原点 (法兰欧拉): "
              << flangeEuleInit[0] << ", " << flangeEuleInit[1] << ", " << flangeEuleInit[2] << " mm, "
              << flangeEuleInit[3] << ", " << flangeEuleInit[4] << ", " << flangeEuleInit[5] << " deg"
              << std::endl;

    // ---- Phase 2: 生成候选位姿 ----
    std::vector<std::array<double, 6>> candidates;
    if (!generateEyeToHandCandidates(robot_handle, flangeEuleInit, pose_config, candidates)) {
        result.message = "无 IK 可达位姿";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }

    // ---- Phase 3: 逐点采集（仅检测角点，不需要相机内参） ----
    std::vector<cv::Mat> robot_poses;
    int captured = executeEyeToHandCollection(
        robot_handle, pchRobotIP, camera, detector, candidates,
        save_dir, robot_poses, pose_config.min_corners);

    if (captured < pose_config.min_valid_frames) {
        result.message = "有效帧数不足: " + std::to_string(captured) +
                         " (need >= " + std::to_string(pose_config.min_valid_frames) + ")";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }

    // ---- Phase 3.5: 从采集图像标定相机内参（复用眼在手上的逐层筛选方法） ----
    cv::Mat camera_matrix, dist_coeffs;
    if (!calibrateCameraFromDisk(save_dir, detector, camera_matrix, dist_coeffs)) {
        result.message = "相机标定失败";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }
    result.camera_matrix = camera_matrix;
    result.dist_coeffs = dist_coeffs;

    // 保存相机内参供后续使用
    {
        std::string cam_file = save_dir + "/camera_params.yaml";
        cv::FileStorage fs(cam_file, cv::FileStorage::WRITE);
        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_coefficients" << dist_coeffs;
        std::cout << "[eyeToHand] 相机内参已保存到: " << cam_file << std::endl;
    }

    // ---- Phase 3.75: 用标定后的内参计算每帧标定板位姿 ----
    std::vector<cv::Mat> camera_poses;
    if (!computeBoardPosesFromImages(save_dir, detector, camera_matrix, dist_coeffs, camera_poses)) {
        result.message = "计算标定板位姿失败";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }

    // robot_poses 与 camera_poses 必须一一对应
    if (robot_poses.size() != camera_poses.size()) {
        result.message = "机器人位姿数 (" + std::to_string(robot_poses.size()) +
                         ") 与标定板位姿数 (" + std::to_string(camera_poses.size()) + ") 不匹配";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }

    // ---- Phase 4: 标定 ----
    return eyeToHandCalibrate(robot_poses, camera_poses, result);
}

// ============================================================
// 从 CSV 加载机器人位姿（法兰欧拉 → 4x4，与采集时保持一致的转换方式）
// ============================================================
static bool loadRobotPosesFromCSV(const std::string& csv_file,
                                   std::vector<cv::Mat>& robot_poses) {
    std::ifstream f(csv_file);
    if (!f.is_open()) {
        std::cerr << "[eyeToHand] 无法打开 " << csv_file << std::endl;
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
        robot_poses.push_back(axisAnglePoseToTransform(vals));
        count++;
    }

    std::cout << "[eyeToHand] 从 " << csv_file << " 加载 " << count << " 帧机器人位姿" << std::endl;
    return count > 0;
}

// ============================================================
// 将 rob_pos.csv（欧拉角）转换为 rob_axis.csv（轴角）
// ============================================================
bool convertEulerCsvToAxisCsv(const std::string& save_dir) {
    std::string euler_file = save_dir + "/rob_eule.csv";
    std::string axis_file = save_dir + "/rob_pos.csv";

    std::ifstream fin(euler_file);
    if (!fin.is_open()) {
        std::cerr << "[eyeToHand] 无法打开 " << euler_file << std::endl;
        return false;
    }

    std::ofstream fout(axis_file);
    if (!fout.is_open()) {
        std::cerr << "[eyeToHand] 无法创建 " << axis_file << std::endl;
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
            std::cerr << "[eyeToHand] 欧拉→轴角转换失败，第 " << (count + 1) << " 行" << std::endl;
            return false;
        }
        fout << std::fixed << std::setprecision(3);
        for (int j = 0; j < ROB_AXIS_NUM; j++)
            fout << axis[j] << (j < ROB_AXIS_NUM - 1 ? "," : "\n");
        count++;
    }

    std::cout << "[eyeToHand] 转换完成: " << euler_file << " → " << axis_file
              << " (" << count << " 帧)" << std::endl;
    return count > 0;
}

// ============================================================
// 离线标定入口：读磁盘数据 → 标定相机 → 标定手眼
// ============================================================
bool calibrateEyeToHandFromDisk(const std::string& save_dir,
                                 EyeToHandResult& result) {

    std::cout << "\n========== 离线眼在手外标定 ==========" << std::endl;
    std::cout << "[eyeToHand] 数据目录: " << save_dir << std::endl;

    // ---- 创建检测器（默认标定板参数，与采集时一致） ----
    CharucoBoardConfig board_cfg;
    CharucoDetector detector(board_cfg);

    // ---- 从图像标定相机内参（复用眼在手上的逐层筛选方法） ----
    if (!calibrateCameraFromDisk(save_dir, detector,
                                  result.camera_matrix, result.dist_coeffs)) {
        result.message = "相机标定失败";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }

    // ---- 计算每帧标定板位姿 ----
    std::vector<cv::Mat> camera_poses;
    if (!computeBoardPosesFromImages(save_dir, detector,
                                      result.camera_matrix, result.dist_coeffs,
                                      camera_poses)) {
        result.message = "计算标定板位姿失败";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }

    // ---- 加载机器人位姿（优先读轴角文件，否则尝试转换欧拉文件） ----
    std::vector<cv::Mat> robot_poses;
    std::string axis_csv = save_dir + "/rob_pos.csv";
    std::string euler_csv = save_dir + "/rob_eule.csv";
    if (loadRobotPosesFromCSV(axis_csv, robot_poses)) {
        // 已有轴角文件
    } else if (convertEulerCsvToAxisCsv(save_dir) &&
               loadRobotPosesFromCSV(axis_csv, robot_poses)) {
        // 转换成功后再加载
    } else {
        result.message = "加载机器人位姿失败";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }

    // robot_poses 与 camera_poses 必须一一对应
    if (robot_poses.size() != camera_poses.size()) {
        result.message = "机器人位姿数 (" + std::to_string(robot_poses.size()) +
                         ") 与标定板位姿数 (" + std::to_string(camera_poses.size()) + ") 不匹配";
        std::cerr << "[eyeToHand] " << result.message << std::endl;
        return false;
    }

    // ---- 手眼标定 ----
    if (!eyeToHandCalibrate(robot_poses, camera_poses, result))
        return false;

    // ---- 保存结果 ----
    {
        std::string out_file = save_dir + "/eye_to_hand_result.yaml";
        cv::FileStorage out(out_file, cv::FileStorage::WRITE);
        out << "T_base_camera" << result.T_base_camera;
        out << "camera_matrix" << result.camera_matrix;
        out << "distortion_coefficients" << result.dist_coeffs;
        out << "reprojection_error_mm" << result.reproj_error * 1000.0;
        out << "valid_frames" << result.valid_frames;
        std::cout << "[eyeToHand] 结果已保存到: " << out_file << std::endl;
    }

    return true;
}
