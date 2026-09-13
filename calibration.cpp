#include "auto_eye_in_hand_charuco_calibration.hpp"
#include "eye_to_hand_calibration.hpp"
#include "calibration_utils.hpp"
#include "calibration_session.hpp"
#include "intrinsic_identity.hpp"
#include "semi_auto_eye_in_hand_calibration.hpp"
#include "charuco_detector.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <set>

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX
    #include <windows.h>
    #include <io.h>
    #include <fcntl.h>

#else
    #include<unistd.h>
#endif

#include "my_base.h"
#include "cameras/AICamera.hpp"

#ifdef useORB
#include "cameras/OrbbecCamera.hpp"
#endif // REL_DBG

#ifdef useOAK
#include "cameras/OAKCamera.hpp"
#endif // REL_DBG

#ifdef useMVS
#include "cameras/MVSCamera.hpp"
#endif // useMVS


#include "robotapi.h"
#include "basestruct.h"
#include "utils/putText.h"
#include "circlesampler/CircleSampler.h"
#include "piecesampler/PieceSampler.h"
#include "piecesampler/Config.h"
#include "testcameradeepth/DepthTestConfig.h"
#include "testcameradeepth/DepthRemedy.h"
#include "testcameradeepth/DepthTester.h"

CONFIG g_cfg;
std::string g_appPath = "";
std::string g_paramPath = "";
std::string g_posPath = "";
std::string g_calibrationPath = "";

std::shared_ptr<AICamera> g_camera = nullptr;
RobotHandle robotHandle;

namespace fs = std::filesystem;


std::string string_format(const char* format, ...) {
    if (!format)
        return {};

    va_list args;
    va_start(args, format);
    va_list countArgs;
    va_copy(countArgs, args);
    const int count = vsnprintf(nullptr, 0, format, countArgs);
    va_end(countArgs);
    if (count < 0) {
        va_end(args);
        return {};
    }

    std::vector<char> buffer(static_cast<size_t>(count) + 1U, '\0');
    vsnprintf(buffer.data(), buffer.size(), format, args);
    va_end(args);
    return std::string(buffer.data(), static_cast<size_t>(count));
}



// 主菜单
void display_menu() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "      机器人标定系统" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "1. 自动眼在手上标定" << std::endl;
    std::cout << "2. 半自动眼在手上标定" << std::endl;
    std::cout << "3. 眼在手外标定" << std::endl;
    std::cout << "4. 螺纹圆形采样" << std::endl;
    std::cout << "5. CircleSample 测试图采集" << std::endl;
    std::cout << "6. 工件检测采样" << std::endl;
    std::cout << "7. 退出程序" << std::endl;
    std::cout << "8. 奇异点测试分析" << std::endl;
    std::cout << "9. 眼在手上本地数据标定" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "请选择操作 (1-9): ";
}


bool GetBasePathInfo() {

    char szPath[2048] = { 0 };
    char* p = szPath;
#ifdef _WIN32
    GetModuleFileNameA(NULL, szPath, sizeof(szPath)); // �õ�ȫ��·��
    char split = '\\';
#else
    const ssize_t pathLength = readlink("/proc/self/exe", szPath, sizeof(szPath) - 1);
    if (pathLength < 0) {
        std::cerr << "读取当前可执行文件路径失败" << std::endl;
        return false;
    }
    szPath[pathLength] = '\0';
    char split = '/';
#endif

    while (strchr(p, split)) // 从后往前找 分隔符
    {
        p = strchr(p, split);
        p++;
    }
    char t = *p;
    *p = 0; // 当前路径放在 szPath 中了
    g_appPath = szPath;

    g_paramPath = g_appPath + "param/";
    g_calibrationPath = g_paramPath + "calibration/";
    g_posPath = g_paramPath + "pos/";
    

    if (!fs::exists(g_paramPath))
        fs::create_directory(g_paramPath); 

    if (!fs::exists(g_calibrationPath))
        fs::create_directory(g_calibrationPath);


    return true;
}

int load_config(std::string cfg_file) {
    cv::FileStorage fs(cfg_file, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        std::cerr << "无法打开配置文件!" << std::endl;
        return -1;
    }

    // Select camera section based on compile flags
    std::string cam_section;
#ifdef useORB
    cam_section = "camera_obb_100";
#elif defined(useMVS)
    int choice = 1;
    std::cout << "\n请选择 MVS 相机型号:" << std::endl;
    std::cout << "  1. camera_mvs_130 (1280x1024)" << std::endl;
    std::cout << "  2. camera_mvs_500 (2448x2048)" << std::endl;
    std::cout << "  请选择 (1-2): " << std::flush;
    std::cin >> choice;
    cam_section = (choice == 2) ? "camera_mvs_500" : "camera_mvs_130";
#else
    cam_section = "camera_obb_100";
#endif

    // Read camera-specific params from the selected section
    cv::FileNode cam_node = fs[cam_section];
    if (!cam_node.empty()) {
        cam_node["cam_ip"] >> g_cfg.cam_ip;
        cam_node["cam_port"] >> g_cfg.cam_port;
        cam_node["fps"] >> g_cfg.fps;
        cam_node["cam_width"] >> g_cfg.cam_width;
        cam_node["cam_height"] >> g_cfg.cam_height;
    } else {
        std::cerr << "[load_config] 配置中未找到 '" << cam_section << "' 节，使用默认参数" << std::endl;
    }

    // Calibration grid params (always at root level)
    fs["grid_num_heng"] >> g_cfg.grid_num_heng;
    fs["grid_num_shu"] >> g_cfg.grid_num_shu;
    fs["grid_size"] >> g_cfg.grid_size;

    // Robot connection params
    fs["robot_ip"] >> g_cfg.robot_ip;
    fs["robot_port"] >> g_cfg.robot_port;
    fs["robot_pwd"] >> g_cfg.robot_pwd;

    // 机械臂运行速度百分比 (1~100)，缺省或越界时回退为 100
    int speed_pct = 100;
    if (!fs["robotSpeedPercent"].empty())
        fs["robotSpeedPercent"] >> speed_pct;
    if (speed_pct < 1 || speed_pct > 100) {
        std::cerr << "[load_config] robotSpeedPercent (" << speed_pct
                  << ") 超出 1~100 范围，回退为 100" << std::endl;
        speed_pct = 100;
    }
    g_cfg.robotSpeedPercent = speed_pct;

    std::cout << "相机型号: " << cam_section << std::endl;
    std::cout << "图像尺寸: " << g_cfg.cam_width << " x " << g_cfg.cam_height << std::endl;
    std::cout << "机械臂速度百分比: " << g_cfg.robotSpeedPercent << "%" << std::endl;

    fs.release();

    return 0;
}

void checkAndRestoreRobotHand(){
    // 检查机械臂是否上电
    enum RobotModes robotMode;
    CRresult res = cr_get_robotMode(robotHandle, &robotMode);
    if (res == sucess)
    {
        if (robotMode == JointPowerOff)
        {
            std::cout << "机械臂未上电，上电使能" << std::endl;
            res = cr_FaultReset(robotHandle);
            if (res == sucess)
            {
                std::cout << "机械臂故障复位" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            else
                std::cout << "故障复位失败" << std::endl;
            
            if (res == sucess)
                res = cr_poweron(robotHandle); // 上电
            if (res == sucess)
            {
                std::cout << "机械臂开始上电" << std::endl;
                while (res == sucess)
                {
                    res = cr_get_robotMode(robotHandle, &robotMode);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    if (robotMode == JointIdle)
                    {
                        break;
                    }
                }

                res = cr_enable(robotHandle); // 使能
                if (res == sucess)
                {
                    std::cout << "机械臂开始使能" << std::endl;
                    while (res == sucess)
                    {
                        res = cr_get_robotMode(robotHandle, &robotMode);
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        if (robotMode == ProgramStop)
                        {
                            break;
                        }
                    }
                }
                else
                    std::cout << "机械臂使能失败 res: " << res << std::endl;
            }
            else
                std::cout << "机械臂上电失败 res: " << res << std::endl;
        }
    }
    else
        std::cout << "获取机械臂模式失败" << std::endl;
}

// ============================================================
// 将配置中的机械臂运行速度百分比 (1~100) 下发到控制器
// 用于自动/半自动眼在手上标定开始前设置机械臂速度
// ============================================================
static void applyRobotSpeedPercent() {
    int pct = g_cfg.robotSpeedPercent;
    if (pct < 1) pct = 1;
    if (pct > 100) pct = 100;
    CRresult res = cr_set_robotSpeedPercent(robotHandle, static_cast<unsigned int>(pct));
    if (res == sucess)
        std::cout << "机械臂速度百分比已设置为 " << pct << "%" << std::endl;
    else
        std::cerr << "设置机械臂速度百分比失败 res: " << res << std::endl;
}

int  open_camera(){
   if (!g_camera->open()) {
                std::cerr << "打开相机失败" << std::endl;
                return -1;
            }

    // 读取相机内参
    auto m = g_camera->getCameraIntrinsics(g_cfg.cam_width, g_cfg.cam_height);        

    if (!m.empty())
    {
        // 保存相机参数到文件
        cv::FileStorage out(g_paramPath + "camera_params.yaml", cv::FileStorage::WRITE);
        cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

        camera_matrix.at<double>(0, 0) = m[0][0];
        camera_matrix.at<double>(1, 1) = m[1][1];
        camera_matrix.at<double>(2, 2) = 1.0;

        camera_matrix.at<double>(0, 2) = m[0][2];
        camera_matrix.at<double>(1, 2) = m[1][2];

        for (size_t i = 0; i < m[3].size(); i++)
        {
            dist_coeffs.at<double>(i, 0) = m[3][i];
        }
        
        out << "camera_matrix" << camera_matrix;
        out << "distortion_coefficients" << dist_coeffs;
    } else {
        std::cerr << "[open_camera] 警告: 相机未提供内参，camera_params.yaml 未更新" << std::endl;
        std::cerr << "[open_camera] 请通过菜单 2 标定生成相机内参文件" << std::endl;
    }

    return 0;
}

// ============================================================
// 螺纹圆形采样入口（菜单项 5）
// ============================================================
static void screw_sampling() {
    
    ScrewSamplingConfig sscfg;
    std::string cfg_path = g_appPath + "circlesampler/screw_sampling_config.yaml";
    bool have_cfg = loadScrewSamplingConfig(cfg_path, sscfg);

    if (have_cfg) {
        std::cout << "已加载配置: " << cfg_path << std::endl;
    } else {
        std::cout << "未找到配置文件，使用交互式输入" << std::endl;
        std::cout << "\n目标点坐标 (x y z mm, 每行一组, 输入 0 0 0 完成):" << std::endl;
        double x, y, z;
        while (std::cin >> x >> y >> z) {
            if (x == 0.0 && y == 0.0 && z == 0.0) break;
            sscfg.positions.push_back(cv::Point3d(x, y, z));
        }
        std::cin.clear();

        std::cout << "采样半径 (输入 0 结束, 空格分隔, 如: 80 100 120 0): ";
        double r;
        while (std::cin >> r && r > 0.001)
            sscfg.radii.push_back(r);
        std::cin.clear();

        std::cout << "背景类型 [" << sscfg.background_type << "]: ";
        std::cin >> sscfg.background_type;
    }

    sscfg.output_dir = g_paramPath + "screw_samples";

    std::cout << "\n配置确认:";
    for (size_t i = 0; i < sscfg.positions.size(); i++) {
        std::cout << "\n  目标点 " << i << ": (" << sscfg.positions[i].x << ", "
                  << sscfg.positions[i].y << ", " << sscfg.positions[i].z << ") mm";
    }
    std::cout << "\n  半径: [";
    for (size_t i = 0; i < sscfg.radii.size(); i++) {
        if (i) std::cout << ", ";
        std::cout << sscfg.radii[i];
    }
    std::cout << "] mm";
    std::cout << "\n  偏航角: [";
    for (size_t i = 0; i < sscfg.yaw_angles.size(); i++) {
        if (i) std::cout << ", ";
        std::cout << sscfg.yaw_angles[i] << "°";
    }
    std::cout << "]";
    std::cout << "\n  曝光数/位姿: " << sscfg.num_exposures;
    std::cout << "\n  输出目录: " << sscfg.output_dir;
    std::cout << "\n  总采样数: " << (sscfg.positions.size() * sscfg.radii.size() * sscfg.yaw_angles.size() * 180 * sscfg.num_exposures) << std::endl;

    // std::cout << "\n开始采样? (y/n): ";
    // char confirm = 'n';
    // std::cin >> confirm;
    // if (confirm != 'y' && confirm != 'Y') {
    //     std::cout << "已取消" << std::endl;
    //     return;
    // }

    {
        CircleSampler sampler(g_camera, sscfg);
        //g_camera->setColorExposureTime(42);    
        int total = sampler.run();
        std::cout << "\n采样完成，共 " << total << " 张图像" << std::endl;
    }
}

void try_camera_exposure(){
     for (size_t i = 0; i < 100; i++)
    {
        bool bRet = g_camera->setColorExposureTime(5000+i*2);

        if (!bRet)
        {
            std::cout<< "error "<<std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200));   
    }
}

// ============================================================
// CircleSample 测试图采集
// ============================================================
static void circle_test_collection(int argc, char** argv) {
    // Parse args: --collect-test <config_path> [--dry-run]
    std::string cfg_path;
    bool dry_run = false;
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--dry-run") == 0) {
            dry_run = true;
        } else if (argv[i][0] != '-') {
            cfg_path = argv[i];
        }
    }

    if (cfg_path.empty()) {
        cfg_path = g_appPath + "circlesampler/circle_test_config.yaml";
        std::cout << "使用默认配置路径: " << cfg_path << std::endl;
    }

    CircleTestConfig cfg;
    if (!loadCircleTestConfig(cfg_path, cfg)) {
        std::cerr << "加载测试配置失败" << std::endl;
        return;
    }

    // Try to load T_flange_camera from hand_eye_result.yaml as fallback
    if (cfg.T_flange_camera.empty()) {
        cv::FileStorage hfs(g_appPath + "hand_eye_result.yaml", cv::FileStorage::READ);
        if (hfs.isOpened()) {
            cv::Mat T;
            hfs["T_flange_camera"] >> T;
            if (!T.empty() && T.size() == cv::Size(4, 4))
                cfg.T_flange_camera = T;
        }
    }

    if (!validateCircleTestConfig(cfg)) {
        std::cerr << "测试配置校验失败" << std::endl;
        return;
    }

    // Open camera if needed
    if (!g_camera || !g_camera->isOpened()) {
#ifdef useORB
        g_camera = std::make_shared<OrbbecCameraV2>(g_cfg);
#elif defined(useOAK)
        g_camera = std::make_shared<OAKCamera>(g_cfg);
#else
        g_camera = std::make_shared<MVSCamera>(g_cfg);
#endif
        if (!g_camera->open()) {
            std::cerr << "打开相机失败" << std::endl;
            return;
        }
    }

    if (dry_run) {
        std::cout << "\n========== DRY RUN MODE ==========" << std::endl;
        std::cout << "仅验证 IK 可达性，不拍照" << std::endl;
        CircleSampler sampler(g_camera, cfg);
        sampler.dryRunTestCollection();
        return;
    }

    CircleSampler sampler(g_camera, cfg);
    int total = sampler.runTestCollection();
    std::cout << "\n采集完成，共 " << total << " 张图像" << std::endl;
    std::cout << "输出目录: " << cfg.output_dir << std::endl;
}

// ============================================================
// 工件检测采样（菜单项 5）
// ============================================================
static void piece_sampling() {
    PieceSamplerConfig cfg;
    std::string cfg_path = g_appPath + "piecesampler/piece_sampling_config.yaml";
    if (!loadPieceSamplerConfig(cfg_path, cfg)) {
        std::cerr << "加载工件采样配置失败: " << cfg_path << std::endl;
        return;
    }

    // Try to load T_flange_camera from hand_eye_result.yaml as fallback
    if (cfg.T_flange_camera.empty()) {
        cv::FileStorage hfs(g_appPath + "hand_eye_result.yaml", cv::FileStorage::READ);
        if (hfs.isOpened()) {
            cv::Mat T;
            hfs["T_flange_camera"] >> T;
            if (!T.empty() && T.size() == cv::Size(4, 4))
                cfg.T_flange_camera = T;
        }
    }

    if (!validatePieceSamplerConfig(cfg)) {
        std::cerr << "工件采样配置校验失败" << std::endl;
        return;
    }

    // Open camera if needed
    if (!g_camera || !g_camera->isOpened()) {
#ifdef useORB
        g_camera = std::make_shared<OrbbecCameraV2>(g_cfg);
#elif defined(useOAK)
        g_camera = std::make_shared<OAKCamera>(g_cfg);
#else
        g_camera = std::make_shared<MVSCamera>(g_cfg);
#endif
        if (!g_camera->open()) {
            std::cerr << "打开相机失败" << std::endl;
            return;
        }
    }

    PieceSampler sampler(g_camera, cfg);
    int total = sampler.run();
    std::cout << "\n采样完成，共 " << total << " 张图像" << std::endl;
    std::cout << "输出目录: " << cfg.output_dir << std::endl;
}

// ============================================================
// 深度陈旧帧测试
// ============================================================
static void depth_test(const std::string& cfg_path) {
    DepthTestConfig cfg;
    if (!loadDepthTestConfig(cfg_path, cfg)) {
        std::cerr << "加载深度测试配置失败: " << cfg_path << std::endl;
        return;
    }

    // Open camera
    if (!g_camera || !g_camera->isOpened()) {
#ifdef useORB
        g_camera = std::make_shared<OrbbecCameraV2>(g_cfg);
#elif defined(useOAK)
        g_camera = std::make_shared<OAKCamera>(g_cfg);
#else
        g_camera = std::make_shared<MVSCamera>(g_cfg);
#endif
        if (!g_camera->open()) {
            std::cerr << "打开相机失败" << std::endl;
            return;
        }
    }

    DepthTester tester(g_camera, robotHandle, cfg);
    tester.run();
}

// ============================================================
// 半自动眼在手内标定入口（菜单项 5）
// ============================================================
static void saveCalibratedCameraParams(const cv::Mat& old_camera_matrix,
                                        const cv::Mat& new_camera_matrix,
                                        const cv::Mat& new_dist_coeffs) {
    if (!old_camera_matrix.empty())
        return;
    cv::FileStorage outCam(g_appPath + "camera_params.yaml", cv::FileStorage::WRITE);
    outCam << "camera_matrix" << new_camera_matrix;
    outCam << "distortion_coefficients" << new_dist_coeffs;
    outCam.release();
    std::cout << "相机内参已保存到: camera_params.yaml" << std::endl;
}

static bool previewBoardDetection(CharucoDetector& detector,
                                   const std::string& window_name,
                                   const std::string& instruction,
                                   const std::string& cancel_tag) {
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name, 960, 600);

    std::cout << "\n----------------------------------------" << std::endl;
    std::cout << "  " << instruction << std::endl;
    std::cout << "  按 's' 开始" << std::endl;
    std::cout << "  按 'q' 取消" << std::endl;
    std::cout << "----------------------------------------\n" << std::endl;

    while (true) {
        cv::Mat frame = g_camera->captureOneFrame(1000);
        if (frame.empty()) continue;

        cv::Mat display;
        cv::Mat rvec, tvec;
        if (detector.detectAndDraw(frame, display, rvec, tvec)) {
            cv::putText(display, "Press 's' to start",
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                        0.7, cv::Scalar(0, 255, 0), 2);
        } else {
            frame.copyTo(display);
            cv::putText(display, "No board detected",
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                        0.7, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow(window_name, display);
        int key = cv::waitKey(30);
        if (key == 's' || key == 'S') {
            cv::destroyWindow(window_name);
            return true;
        }
        if (key == 'q' || key == 'Q') {
            std::cout << "[" << cancel_tag << "] 用户取消" << std::endl;
            cv::destroyWindow(window_name);
            return false;
        }
    }
}


// ============================================================
// 自动手眼标定入口（菜单项 5）
// ============================================================
//void moveToJointPos(int robot_handle, const double joint[6]);
// extern bool moveToAnglePos(int robot_handle, double* anglePos);
void auto_handeye_calibration(int cameraToTcpEnd, char *pchRobotIP) {
    // 尝试加载已有相机内参
    cv::Mat camera_matrix, dist_coeffs;
    std::string cam_file = g_paramPath + "camera_params.yaml";
    {
        cv::FileStorage storage(cam_file, cv::FileStorage::READ);
        if (storage.isOpened()) {
            storage["camera_matrix"] >> camera_matrix;
            storage["distortion_coefficients"] >> dist_coeffs;
            storage.release();
        }
    }

    checkAndRestoreRobotHand();
    applyRobotSpeedPercent();
    // double init_tcp_pose[6] = {-703.396, -198.517, 211.509, 140.34, 112.72, 0.00616}, euler_fixed[6];
    //double init_tcp_pose[6] = {-502.377, 204.607, 207.971,1.76, -0.918, -47.680}, euler_fixed[6];// offseted
    double init_flangeAxisPos[6]={-676.9377, 155.607, 275.971, 146.58, 93.1018, 7.71780}; // no-offset
    double init_tcp_pose[6] = {-676.9377, 155.607, 275.971, 146.58, 93.1018, 7.71780}, euler_fixed[6];// no-offseted
    
   
    TCPMsg tcp_msg;   
    convertTcpAxisPos2FlangPos(robotHandle, init_tcp_pose, init_flangeAxisPos);

    CharucoBoardConfig board_cfg;
    CharucoDetector detector(board_cfg, cam_file);

    if (!previewBoardDetection(detector, "Auto Calibration",
                               "请拖拽机器人到任意标定板可见的位姿\n  系统将自动调整到最佳位姿",
                               "autoCalib"))
        return;

    // 记录初始位姿
    CRresult ret = cr_get_tcpActualPose(robotHandle, init_tcp_pose);
    if (ret != sucess) {
        std::cerr << "[autoCalib] 获取 TCP 位姿失败" << std::endl;               
    }
    ret = cr_get_currentTCPmsg(robotHandle, &tcp_msg);
    if (ret != sucess) {
          std::cerr << "[autoCalib] 获取 TCP 偏移失败" << std::endl;               
    }
   
    // CharucoBoardConfig 使用默认值（7x5 方格, 40mm 边长, 28mm 标记）

    // 调用标定模块
    CalibrationResult result;
    bool ok = autoEyeInHandCharucoCalibration(
        robotHandle,
        pchRobotIP,
        g_camera,
        board_cfg,
        camera_matrix,
        dist_coeffs,
        cameraToTcpEnd,
        result,
        g_calibrationPath,
        g_paramPath + "hand_eye_result.yaml",
        nullptr,
        cam_file
    );

    if (ok) {
        // 结果已由 calibrateHandEyeFromDisk 输出并保存
        saveCalibratedCameraParams(camera_matrix, result.camera_matrix, result.dist_coeffs);
    } else {
        std::cerr << "\n[autoCalib] 标定失败: " << result.message << std::endl;
    }
}


static void semi_auto_eye_in_hand_calibration_wrapper(char* pchRobotIP) {
    // 标定开始前设置机械臂运行速度
    applyRobotSpeedPercent();

    if (open_camera() != 0) return;

    CharucoBoardConfig board_cfg;
    CharucoDetector detector(board_cfg);

    if (!previewBoardDetection(detector, "Semi-Auto Eye-in-Hand",
                               "请将机器人拖到起始位姿（标定板在相机视野内）",
                               "semiAutoEyeInHand"))
        return;

    // 半自动标定的运动范围属于安全关键配置，缺失或非法时禁止静默回退默认值。
    PoseGenerationConfig pose_config;
    std::string pose_config_error;
    const std::string pose_config_path = g_paramPath + "eye_in_hand_config.yaml";
    if (!tryLoadPoseGenerationConfig(pose_config_path, pose_config,
                                     pose_config_error)) {
        std::cerr << "[semiAutoEyeInHand] 位姿配置加载失败，已阻止机器人运动: "
                  << pose_config_error << std::endl;
        return;
    }

    // 加载已知相机内参（如有）
    cv::Mat camera_matrix, dist_coeffs;
    std::string cam_file = g_paramPath + "camera_params.yaml";
    if (fs::exists(cam_file)) {
        cv::FileStorage storage(cam_file, cv::FileStorage::READ);
        if (storage.isOpened()) {
            storage["camera_matrix"] >> camera_matrix;
            storage["distortion_coefficients"] >> dist_coeffs;
            storage.release();
        }
    }

    bool bootstrap_intrinsics = camera_matrix.empty() || dist_coeffs.empty();
#ifdef useMVS
    if (g_camera) {
        const CameraDeviceAuditInfo device_info = g_camera->getDeviceAuditInfo();
        intrinsic_gate::IntrinsicIdentity identity;
        std::string decision_reason;
        const intrinsic_gate::MvsIntrinsicDecision decision =
            intrinsic_gate::decideMvsIntrinsics(
                cam_file, camera_matrix, dist_coeffs, device_info.serial,
                device_info.actual_width, device_info.actual_height,
                g_cfg.allow_legacy_intrinsics, identity, decision_reason);
        if (decision == intrinsic_gate::MvsIntrinsicDecision::Block) {
            std::cerr << "[semiAutoEyeInHand] MVS 内参身份冲突，已阻止机器人运动: "
                      << decision_reason << std::endl;
            return;
        }
        if (decision ==
            intrinsic_gate::MvsIntrinsicDecision::BootstrapFromCollection) {
            camera_matrix.release();
            dist_coeffs.release();
            bootstrap_intrinsics = true;
            std::cout << "[semiAutoEyeInHand] 现有内参不适用于当前相机，"
                         "将从本次采集数据重新标定: "
                      << decision_reason << std::endl;
        } else {
            bootstrap_intrinsics = false;
            std::cout << "[semiAutoEyeInHand] 使用已验证的 MVS 内参" << std::endl;
        }
    }
#endif
    std::string save_dir = g_paramPath + "semi_eye_in_hand/";

    CalibrationResult result;
    bool ok = semiAutoEyeInHandCalibration(
        robotHandle, pchRobotIP, g_camera, detector,
        pose_config, board_cfg,
        camera_matrix, dist_coeffs,
        result, save_dir,
        g_paramPath + "hand_eye_result.yaml");

    if (ok) {
#ifdef useMVS
        if (bootstrap_intrinsics && g_camera) {
            const CameraDeviceAuditInfo device_info = g_camera->getDeviceAuditInfo();
            const CONFIG& camera_config = g_camera->getConfig();
            intrinsic_gate::IntrinsicIdentity identity;
            identity.serial = device_info.serial;
            identity.width = device_info.actual_width > 0
                ? device_info.actual_width : camera_config.cam_width;
            identity.height = device_info.actual_height > 0
                ? device_info.actual_height : camera_config.cam_height;
            identity.pixel_format = device_info.pixel_format;
            std::string save_error;
            if (!intrinsic_gate::saveCameraParamsWithIdentity(
                    cam_file, result.camera_matrix, result.dist_coeffs,
                    identity, "calibrated_from_semi_auto_collection",
                    save_error)) {
                std::cerr << "[semiAutoEyeInHand] 标定成功，但保存 MVS 内参失败: "
                          << save_error << std::endl;
            } else {
                std::cout << "[semiAutoEyeInHand] MVS 内参已保存到: "
                          << cam_file << std::endl;
            }
        }
#else
        saveCalibratedCameraParams(camera_matrix, result.camera_matrix, result.dist_coeffs);
#endif
    } else {
        std::cerr << "\n[semiAutoEyeInHand] 标定失败: " << result.message << std::endl;
    }
}

// ============================================================
// 眼在手上本地数据标定入口（菜单项 9）
// 图像与机械臂位姿已采集完成，仅离线计算相机内参与手眼矩阵
// ============================================================

// 菜单选项 → 旋转角度: 0:0  1:-90  2:90  3:180
static int rotationChoiceToAngle(int choice) {
    switch (choice) {
    case 1:  return -90;
    case 2:  return 90;
    case 3:  return 180;
    default: return 0;
    }
}

static void rotateImageForCalibration(const cv::Mat& src, cv::Mat& dst, int angle_deg) {
    if (angle_deg == 90)
        cv::rotate(src, dst, cv::ROTATE_90_CLOCKWISE);
    else if (angle_deg == -90)
        cv::rotate(src, dst, cv::ROTATE_90_COUNTERCLOCKWISE);
    else if (angle_deg == 180 || angle_deg == -180)
        cv::rotate(src, dst, cv::ROTATE_180);
    else
        dst = src.clone();
}

// 将 src_dir 中的图像按指定角度旋转后写入工作目录，并复制 rob_pos.csv
// 成功返回工作目录，失败返回空字符串
static std::string prepareRotatedDataset(const std::string& src_dir, int angle_deg) {
    std::string work_dir = src_dir + "rotated_" + std::to_string(angle_deg) + "/";

    if (fs::exists(work_dir))
        fs::remove_all(work_dir);
    fs::create_directories(work_dir);

    int count = 0;
    for (const auto& entry : fs::directory_iterator(src_dir)) {
        if (entry.path().extension() != ".jpg") continue;
        cv::Mat img = cv::imread(entry.path().string());
        if (img.empty()) continue;
        cv::Mat rotated;
        rotateImageForCalibration(img, rotated, angle_deg);
        cv::imwrite(work_dir + entry.path().filename().string(), rotated);
        count++;
    }
    if (count == 0) {
        std::cerr << "[localEyeInHand] 未找到可旋转的图像" << std::endl;
        return "";
    }

    std::error_code ec;
    fs::copy_file(src_dir + "rob_pos.csv", work_dir + "rob_pos.csv",
                  fs::copy_options::overwrite_existing, ec);
    if (ec) {
        std::cerr << "[localEyeInHand] 复制 rob_pos.csv 失败: " << ec.message() << std::endl;
        return "";
    }

    std::cout << "[localEyeInHand] 已生成旋转数据: " << work_dir
              << " (" << count << " 张, " << angle_deg << "°)" << std::endl;
    return work_dir;
}

static void local_eye_in_hand_calibration_wrapper() {
    std::cout << "\n========== 眼在手上本地数据标定 ==========" << std::endl;

    // 数据目录（默认为半自动眼在手上标定的采集目录）
    std::string save_dir = g_paramPath + "semi_eye_in_hand/";
    std::cout << "数据目录 (含图像与 rob_pos.csv) [" << save_dir << "]: ";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::string user_dir;
    std::getline(std::cin, user_dir);
    if (!user_dir.empty()) {
        save_dir = user_dir;
        if (save_dir.back() != '/' && save_dir.back() != '\\')
            save_dir += '/';
    }

    if (!fs::exists(save_dir + "rob_pos.csv")) {
        std::cerr << "[localEyeInHand] 未找到位姿文件: " << save_dir + "rob_pos.csv" << std::endl;
        return;
    }
    int jpg_count = 0;
    for (const auto& entry : fs::directory_iterator(save_dir))
        if (entry.path().extension() == ".jpg") jpg_count++;
    if (jpg_count == 0) {
        std::cerr << "[localEyeInHand] 数据目录中没有图像: " << save_dir << std::endl;
        return;
    }
    std::cout << "[localEyeInHand] 找到 " << jpg_count << " 张图像" << std::endl;

    // 选择标定前的图像旋转角度
    std::cout << "\n请选择标定前的图像旋转角度:" << std::endl;
    std::cout << "  0: 0° (不旋转)" << std::endl;
    std::cout << "  1: -90° (逆时针)" << std::endl;
    std::cout << "  2: 90° (顺时针)" << std::endl;
    std::cout << "  3: 180°" << std::endl;
    std::cout << "请选择 (0-3) [0]: ";
    int rot_choice = 0;
    std::cin >> rot_choice;
    int angle_deg = rotationChoiceToAngle(rot_choice);

    // 需要旋转时生成旋转副本目录，否则直接使用原始数据
    std::string work_dir = save_dir;
    if (angle_deg != 0) {
        std::cout << "[localEyeInHand] 图像旋转角度: " << angle_deg << "°" << std::endl;
        work_dir = prepareRotatedDataset(save_dir, angle_deg);
        if (work_dir.empty()) return;
    }

    CharucoBoardConfig board_cfg;
    cv::Mat camera_matrix, dist_coeffs;  // 空: 由本地数据重新标定内参

    CalibrationResult result;
    bool ok = calibrateEyeInHandFromDisk(work_dir, board_cfg,
                                         camera_matrix, dist_coeffs, result,
                                         g_paramPath + "hand_eye_result.yaml");

    if (ok) {
        saveCalibratedCameraParams(cv::Mat(), result.camera_matrix, result.dist_coeffs);
    } else {
        std::cerr << "\n[localEyeInHand] 标定失败: " << result.message << std::endl;
    }
}

// ============================================================
// 眼在手外标定入口（菜单项 6）
// ============================================================
static void auto_eye_to_hand_calibration(char* pchRobotIP) {
    PoseGenerationConfig pose_config;
    std::string config_error;
    if (!tryLoadPoseGenerationConfig(g_paramPath + "eye_to_hand_config.yaml",
                                     pose_config, config_error)) {
        std::cerr << "[eyeToHand] 配置加载失败: " << config_error << std::endl;
        return;
    }
    // 标定开始前设置机械臂运行速度
    applyRobotSpeedPercent();

    if (open_camera() != 0) return;

    CharucoBoardConfig board_cfg;
    CharucoDetector detector(board_cfg);  // 不加载相机内参，采图后从图像标定而来

    // 预览：用户将标定板移到视野中央后按 s
    double flangEule[] = {-398.817, 45.0551, -307.713, -102.154, -77.578, 109.514};
    moveToFlangeEulePos(robotHandle, flangEule);
    if (!previewBoardDetection(detector, "Eye-to-Hand Calibration",
                               "请将标定板（装在 TCP 上）移到相机视野中央",
                               "eyeToHand"))
        return;

    std::string save_dir = g_paramPath + "eye_to_hand/";

    EyeToHandResult result;
    bool ok = autoEyeToHandCharucoCalibration(
        robotHandle, pchRobotIP, g_camera, detector,
        pose_config, result, save_dir);

    if (ok) {
        if (!saveCalibrationResult(g_paramPath + "eye_to_hand_result.yaml", "T_base_camera",
                               result.T_base_camera, result.camera_matrix, result.dist_coeffs,
                               result.reproj_error * 1000.0, result.valid_frames)) {
            std::cerr << "[eyeToHand] 标定结果保存失败" << std::endl;
        }
    } else {
        std::cerr << "\n[eyeToHand] 标定失败: " << result.message << std::endl;
    }
}

bool tryMoveByLineOrJoint(int robotHandle, double tcpAxisPosZ[6], double x_off, double y_off, double z_off){
    CRresult ret = moveToJointPosByLine(robotHandle, tcpAxisPosZ);
    //cr_get_jointActualPos(robotHandle, jointPos);
    if (sucess != ret)
    {                     
        bool   ok = moveToTcpAxisPos(robotHandle, tcpAxisPosZ);                    
        if(!ok)
                return false;
        else
          std::cout << "movel失败， moveJ成功：x_off=" << x_off << ", y_ff=" << y_off << ", z=" <<  z_off << std::endl;
    }
   return true;
}

// ============================================================
// 奇异点测试分析（菜单项 8）
// ============================================================
static void singularity_test() {
    std::cout << "\n===== 奇异点测试分析 =====" << std::endl;
    std::cout << "机械臂将遍历 x/y 偏移组合，逐步上抬直到不可达" << std::endl;
    std::cout << "请确保机器人处于安全状态，按 Enter 开始..." << std::endl;
    std::cin.get();

    
    double tcpAxisPoseInit[ROB_AXIS_NUM] = {-805.5958993, -50.970954, -192.117255, -140.155821, -99.117461, -9.923929};
    moveToTcpAxisPos(robotHandle, tcpAxisPoseInit);
    // if (cr_get_tcpActualPose(robotHandle, tcpAxisPoseInit) != sucess)     {
    //     std::cerr << "获取初始tcp轴角失败" << std::endl;
    //     return;
    // }
    
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "\n初始法兰位姿 (X Y Z RX RY RZ): "
              << tcpAxisPoseInit[0] << " " << tcpAxisPoseInit[1] << " " << tcpAxisPoseInit[2] << "  "
              << tcpAxisPoseInit[3] << " " << tcpAxisPoseInit[4] << " " << tcpAxisPoseInit[5] << std::endl;

    const double x_offsets[] = { -200, -100, 0, 100, 200, 300};
    const double y_offsets[] = {-500, -400, -300, -200, 0, 200, 300, 400, 500};
    const double step_z = 50.0;  // 5cm

    std::cout << "\n" << std::string(75, '-') << std::endl;
    std::cout << std::left
              << std::setw(8) << "X偏移"
              << std::setw(8) << "Y偏移"
              << std::setw(10) << "初始Z"
              << std::setw(10) << "最大Z"
              << std::setw(10) << "抬升"
              << std::setw(6) << "步数"
              << "结果" << std::endl;
    std::cout << std::string(75, '-') << std::endl;

    for (double x_off : x_offsets) {
        for (double y_off : y_offsets) {
            double tcpAxisPos[ROB_AXIS_NUM], tcpAxisPosZ[ROB_AXIS_NUM], jointPos[ROB_AXIS_NUM];
            memcpy(tcpAxisPos, tcpAxisPoseInit, sizeof(tcpAxisPoseInit));
            tcpAxisPos[0] = tcpAxisPoseInit[0] + x_off;
            tcpAxisPos[1] = tcpAxisPoseInit[1] + y_off;
            bool ok = moveToTcpAxisPos(robotHandle, tcpAxisPos);
            if (!ok) {
                std::cout << std::left
                          << std::setw(8) << x_off
                          << std::setw(8) << y_off
                          << "运动失败" << std::endl;
                continue;
            }

            // 逐步上抬
            int steps = 0;
            double max_z = tcpAxisPos[2];
            memcpy(tcpAxisPosZ, tcpAxisPos, sizeof(tcpAxisPosZ));

            while (true) {
                tcpAxisPosZ[2] += step_z;
                if(!tryMoveByLineOrJoint(robotHandle, tcpAxisPosZ, x_off, y_off, tcpAxisPosZ[2] -tcpAxisPos[2]))
                   break;               

                steps++;
                max_z = tcpAxisPosZ[2];
            }

            double lift = max_z - tcpAxisPos[2];
            std::cout << std::fixed << std::setprecision(1);
            std::cout << std::left
                      << std::setw(8) << x_off
                      << std::setw(8) << y_off
                      << std::setw(10) << tcpAxisPos[2]
                      << std::setw(10) << max_z
                      << std::setw(10) << lift
                      << std::setw(6) << steps
                      << (steps > 0 ? "OK" : "失败") << std::endl;

            // 回到初始点
            //moveToJointPosByLine(robotHandle, tcpAxisPoseInit);
            tryMoveByLineOrJoint(robotHandle, tcpAxisPoseInit, -1, -1, -1);
        }
    }

    std::cout << std::string(75, '-') << std::endl;
    std::cout << "测试完成" << std::endl;
}

// ============================================================
// CLI 命令处理（返回 true 表示已处理，main 应退出）
// ============================================================
static bool handleCLI(int argc, char** argv) {
    if (argc < 2) return false;

    // --collect-test <config_path> [--dry-run]
    if (strcmp(argv[1], "--collect-test") == 0) {
        circle_test_collection(argc, argv);
        return true;
    }

    // --depth-test <config_path>
    if (strcmp(argv[1], "--depth-test") == 0) {
        if (argc >= 3) depth_test(argv[2]);
        return true;
    }

    // --piece-sampler
    if (strcmp(argv[1], "--piece-sampler") == 0) {
        piece_sampling();
        return true;
    }

    // --calibrate-eye-to-hand <save_dir>
    if (strcmp(argv[1], "--calibrate-eye-to-hand") == 0) {
        EyeToHandResult result;
        if (argc < 3 || !calibrateEyeToHandFromDisk(argv[2], result)) {
            std::cerr << "\n[main] 离线标定失败: " << result.message << std::endl;
        }
        return true;
    }

    // --calibrate-eye-in-hand <save_dir> [camera_params.yaml] [output.yaml]
    if (strcmp(argv[1], "--calibrate-eye-in-hand") == 0) {
        CharucoBoardConfig board_cfg;
        cv::Mat camera_matrix, dist_coeffs;
        CalibrationResult result;
        if (argc < 3) {
            std::cerr << "用法: " << argv[0]
                      << " --calibrate-eye-in-hand <save_dir>"
                         " [camera_params.yaml] [output.yaml]" << std::endl;
            return true;
        }

        fs::path intrinsics_path;
        if (argc >= 4) {
            intrinsics_path = argv[3];
        } else {
            // 标准目录结构为 param/calibration + param/camera_params.yaml。
            intrinsics_path = fs::path(argv[2]).parent_path() / "camera_params.yaml";
        }
        if (fs::exists(intrinsics_path)) {
            cv::FileStorage storage(intrinsics_path.string(), cv::FileStorage::READ);
            if (storage.isOpened()) {
                storage["camera_matrix"] >> camera_matrix;
                storage["distortion_coefficients"] >> dist_coeffs;
                storage.release();
            }
            if (camera_matrix.empty() || dist_coeffs.empty()) {
                std::cerr << "[main] 相机内参读取失败: "
                          << intrinsics_path << std::endl;
                return true;
            }
            std::cout << "[main] 使用相机内参: " << intrinsics_path << std::endl;
        } else {
            std::cout << "[main] 未找到已知内参，将从采集图像重新标定: "
                      << intrinsics_path << std::endl;
        }

        const std::string output_yaml = argc >= 5 ? argv[4] : std::string();
        if (!calibrateEyeInHandFromDisk(argv[2], board_cfg,
                camera_matrix, dist_coeffs, result, output_yaml)) {
            std::cerr << "\n[main] 离线标定失败: " << result.message << std::endl;
        } else if (!output_yaml.empty()) {
            std::cout << "[main] 离线标定结果: " << output_yaml << std::endl;
        }
        return true;
    }

    // --convert-eye-to-hand-poses <save_dir>
    if (strcmp(argv[1], "--convert-eye-to-hand-poses") == 0) {
        if (argc >= 3 && convertEulerCsvToAxisCsv(argv[2]))
            std::cout << "[main] 转换成功!" << std::endl;
        else
            std::cerr << "[main] 转换失败" << std::endl;
        return true;
    }

    return false;
}

int main(int argc, char** argv) {

    if (!GetBasePathInfo())
        return EXIT_FAILURE;

    std::set<std::string> auditRoots{
        g_calibrationPath,
        g_paramPath + "semi_eye_in_hand/"
    };
    if (argc >= 3 && std::string(argv[1]) == "--calibrate-eye-in-hand")
        auditRoots.insert(argv[2]);
    for (const std::string& root : auditRoots) {
        std::vector<std::string> recovered;
        std::string recoveryError;
        if (!CalibrationSession::recoverPendingSessions(root, recovered, recoveryError)) {
            std::cerr << "[main] 标定 Session 启动恢复失败: " << recoveryError
                      << ", root=" << root << std::endl;
            return EXIT_FAILURE;
        }
        for (const std::string& session : recovered)
            std::cout << "[main] 已恢复未完成标定 Session: " << session << std::endl;
    }

    if (handleCLI(argc, argv))
        return 0;

    int choice = 0;

    std::setlocale(LC_ALL, "en_US.utf8");

    std::cout << "当前路径：" << g_appPath << std::endl;

    load_config(g_appPath+"config.yaml");

    // 初始化机器人

    CRresult result;
    std::cout << "robot_ip: " << g_cfg.robot_ip << ", port: " << g_cfg.robot_port << ", pwd: " << g_cfg.robot_pwd << std::endl;
    result = cr_create_robot(&robotHandle, g_cfg.robot_ip.c_str(), g_cfg.robot_port, g_cfg.robot_pwd.c_str());  // 机械臂连接 

    if (result != sucess){
        std::cout <<"[main] 连接机械臂失败\n"<<std::flush;
        return -1;
    }
    // 使用默认配置

#ifdef useORB
    g_camera = std::make_shared<OrbbecCameraV2>(g_cfg);
#else    
    g_camera = std::make_shared<MVSCamera>(g_cfg);
#endif
    
    while (true) {
        display_menu();
        std::cin >> choice;

        int ret = -1;
        switch (choice) {
        case 1:
            ret = open_camera();
            if (-1==ret) break;

            auto_handeye_calibration(150, (char*)g_cfg.robot_ip.c_str());
            break;
        case 2:
            semi_auto_eye_in_hand_calibration_wrapper((char*)g_cfg.robot_ip.c_str());
            break;
        case 3:
            auto_eye_to_hand_calibration((char*)g_cfg.robot_ip.c_str());
            break;
        case 4:
            ret = open_camera();
            screw_sampling();
            break;
        case 5:
            circle_test_collection(0, nullptr);
            break;
        case 6:
            piece_sampling();
            break;
        case 7:
            std::cout << "感谢使用机器人手眼标定系统，再见！" << std::endl;
            return 0;
        case 8:
            singularity_test();
            break;
        case 9:
            local_eye_in_hand_calibration_wrapper();
            break;
        default:
            std::cout << "无效选择，请重新输入 (1-9)" << std::endl;
        }

        // 清空输入缓冲区
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }


    g_camera->close();
    g_camera = nullptr;

    result = cr_destroy_robot(robotHandle);
    if (result != sucess)
        return -1;

    return 0;
}
