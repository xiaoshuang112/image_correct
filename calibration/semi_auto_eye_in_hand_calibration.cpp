#include "semi_auto_eye_in_hand_calibration.hpp"

#include "camera_calibration.hpp"           // calibrateCameraFromDisk, loadImagesFromDisk
#include "calibration_utils.hpp"            // executeCollection, PoseCandidate

#include <opencv2/calib3d.hpp>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <filesystem>

#include "utils/robot_util.hpp"
#include "utils/calib_transform.hpp"

// ============================================================
// 内部函数
// ============================================================
namespace {

// Phase 2: 位姿生成 + IK 过滤
// 与 generateEyeToHandCandidates 结构相同，输出 std::vector<PoseCandidate>
static bool generateEyeInHandCandidates(
    int robot_handle,
    const double flangeEuleInit[6],
    const PoseGenerationConfig& config,
    std::vector<PoseCandidate>& candidates) {

    candidates.clear();
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
            PoseCandidate cand;
            std::copy(joint, joint + 6, cand.joint);
            std::fill(cand.axis_angle, cand.axis_angle + 6, 0.0);
            candidates.push_back(cand);
        }
    }}}}}}

    std::cout << "[semiAutoEyeInHand] 位姿生成: " << candidates.size() << "/"
              << (config.offset_x.size() * config.offset_y.size() *
                  config.offset_z.size() * config.offset_rx.size() *
                  config.offset_ry.size() * config.offset_rz.size())
              << " IK 可达" << std::endl;

    return !candidates.empty();
}

// 统计 save_dir 下 .jpg 文件数
static int countCapturedFrames(const std::string& dir) {
    namespace fs = std::filesystem;
    int count = 0;
    if (!fs::exists(dir)) return 0;
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() == ".jpg") count++;
    }
    return count;
}

} // anonymous namespace

// ============================================================
// 在线标定入口
// ============================================================
bool semiAutoEyeInHandCalibration(
    int robot_handle,
    char* pchRobotIP,
    std::shared_ptr<AICamera>& camera,
    CharucoDetector& detector,
    const PoseGenerationConfig& pose_config,
    const CharucoBoardConfig& board_config,
    cv::InputOutputArray camera_matrix,
    cv::InputOutputArray dist_coeffs,
    CalibrationResult& result,
    const std::string& save_dir,
    const std::string& output_yaml,
    AI_det_callback callback) {

    std::cout << "\n========== 半自动眼在手内 ChArUco 标定 ==========" << std::endl;

    // ---- Phase 1: 获取示教原点 ----
    double tcp_aa[6], flangeEuleInit[6];
    if (cr_get_tcpActualPose((RobotHandle)robot_handle, tcp_aa) != sucess) {
        result.message = "获取机器人位姿失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return false;
    }
    if (!convertTcpAxisPos2FlangeEule(robot_handle, tcp_aa, flangeEuleInit)) {
        result.message = "轴角转法兰欧拉失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return false;
    }
    std::cout << "[semiAutoEyeInHand] 示教原点 (法兰欧拉): "
              << flangeEuleInit[0] << ", " << flangeEuleInit[1] << ", " << flangeEuleInit[2] << " mm, "
              << flangeEuleInit[3] << ", " << flangeEuleInit[4] << ", " << flangeEuleInit[5] << " deg"
              << std::endl;

    // ---- Phase 2: 生成候选位姿 ----
    std::vector<PoseCandidate> candidates;
    if (!generateEyeInHandCandidates(robot_handle, flangeEuleInit, pose_config, candidates)) {
        result.message = "无 IK 可达位姿";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return false;
    }

    // ---- Phase 3: 逐点采集 ----
    if (!executeCollection(robot_handle, pchRobotIP, camera, detector,
                            candidates, save_dir, callback)) {
        result.message = "采集失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return false;
    }

    {
        int captured = countCapturedFrames(save_dir);
        if (captured < pose_config.min_valid_frames) {
            result.message = "有效帧数不足: " + std::to_string(captured) +
                             " (need >= " + std::to_string(pose_config.min_valid_frames) + ")";
            std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
            return false;
        }
        std::cout << "[semiAutoEyeInHand] 有效帧: " << captured << std::endl;
    }

    // ---- Phase 4: 相机内参标定 ----
    cv::Mat cam_mat, dist;
    if (!camera_matrix.empty()) {
        cam_mat = camera_matrix.getMat().clone();
        dist = dist_coeffs.getMat().clone();
        std::cout << "[semiAutoEyeInHand] 使用已知相机内参" << std::endl;
    } else {
        if (!calibrateCameraFromDisk(save_dir, detector, cam_mat, dist)) {
            result.message = "相机内参标定失败";
            std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
            return false;
        }
        std::cout << "[semiAutoEyeInHand] 相机内参已标定" << std::endl;
    }

    // ---- Phase 5: 手眼标定 ----
    if (!calibrateHandEyeFromDisk(save_dir, cam_mat, dist, board_config,
                                   result, output_yaml)) {
        result.message = "手眼标定失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return false;
    }

    // 输出回传
    cam_mat.copyTo(camera_matrix);
    dist.copyTo(dist_coeffs);

    return true;
}

// ============================================================
// 离线标定入口
// ============================================================
bool calibrateEyeInHandFromDisk(
    const std::string& save_dir,
    const CharucoBoardConfig& board_config,
    cv::InputOutputArray camera_matrix,
    cv::InputOutputArray dist_coeffs,
    CalibrationResult& result,
    const std::string& output_yaml) {

    std::cout << "\n========== 半自动眼在手内离线标定 ==========" << std::endl;
    std::cout << "[semiAutoEyeInHand] 数据目录: " << save_dir << std::endl;

    CharucoDetector detector(board_config);

    // ---- Phase 1: 相机内参标定 ----
    cv::Mat cam_mat, dist;
    if (false ) { // !camera_matrix.empty()
        cam_mat = camera_matrix.getMat().clone();
        dist = dist_coeffs.getMat().clone();
        std::cout << "[semiAutoEyeInHand] 使用已知相机内参" << std::endl;
    } else {
        if (!calibrateCameraFromDisk(save_dir, detector, cam_mat, dist)) {
            result.message = "相机内参标定失败";
            std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
            return false;
        }
        std::cout << "[semiAutoEyeInHand] 相机内参已标定" << std::endl;
    }

    // ---- Phase 2: 手眼标定 ----
    if (!calibrateHandEyeFromDisk(save_dir, cam_mat, dist, board_config, result, output_yaml)) {
        result.message = "手眼标定失败";
        std::cerr << "[semiAutoEyeInHand] " << result.message << std::endl;
        return false;
    }

    // 输出回传
    cam_mat.copyTo(camera_matrix);
    dist.copyTo(dist_coeffs);

    return true;
}

// ============================================================
// 手眼标定精化：Gauss-Newton 最小化基座标系下板位姿平移方差
// ============================================================
namespace {

static cv::Mat refineHandEyePoseConsistency(
    const cv::Mat& X_init,
    const std::vector<cv::Mat>& robot_poses,
    const std::vector<cv::Mat>& camera_poses) {

    int n = (int)robot_poses.size();
    if (n < 3) return X_init.clone();

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

    std::vector<cv::Mat> R_gripper2base, t_gripper2base;
    std::vector<cv::Mat> R_target2cam, t_target2cam;
    for (int i = 0; i < valid; i++) {
        R_gripper2base.push_back(robot_poses[i](cv::Rect(0, 0, 3, 3)).clone());
        t_gripper2base.push_back(robot_poses[i](cv::Rect(3, 0, 1, 3)).clone());
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

        T = refineHandEyePoseConsistency(T, robot_poses, camera_poses);

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
    std::cout << "[autoCalib] T_flange_camera:\n" << result.T_flange_camera << std::endl;

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
        float thresh = 0.001;
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
}}

// ============================================================
// 手眼标定主入口（从磁盘读取数据）
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
    std::vector<cv::Mat> flange_poses;
    if (!loadRobotPosesFromCSV(save_dir + "/rob_pos.csv", flange_poses)) {
        result.message = "无法加载 " + save_dir + "/rob_pos.csv";
        std::cerr << "[autoCalib] " << result.message << std::endl;
        return false;
    }

    // 扫描图像文件
    std::vector<std::string> img_files;
    cv::Size image_size;
    if (!loadImagesFromDisk(save_dir, img_files, image_size))
        return false;

    size_t n = std::min(img_files.size(), flange_poses.size());
    const auto& board_points = detector.getChessboardCorners3D();

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
        frames.push_back({flange_poses[i], calib_transform::rvecTvecToTransform(rvec, tvec), err});
    }

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

    if (!output_yaml.empty() && result.success) {
        saveCalibrationResult(output_yaml, "T_flange_camera",
                               result.T_flange_camera, result.camera_matrix, result.dist_coeffs,
                               result.reproj_error * 1000.0, result.valid_frames);
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
