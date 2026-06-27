#include "auto_eye_in_hand_charuco_calibration.hpp"
#include "eye_to_hand_calibration.hpp"
#include "calibration_utils.hpp"
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
#if 1 // 最大长度限制：1024 - 1
    char buff[4096] = { 0 };
    va_list args;
    va_start(args, format);
    vsprintf(buff, format, args);
    va_end(args);
    std::string str(buff);
    return str;
#else // 无长度限制
    va_list args;
    va_start(args, format);
    int count = vsnprintf(NULL, 0, format, args); // 使用vsnprintf：warning C4996; 使用vsnprintf_s：无法自动计算长度
    va_end(args);
    va_start(args, format);
    char* buff = (char*)malloc(count * sizeof(wchar_t));
    vsnprintf(buff, count, format, args);
    va_end(args);
    std::string str(buff, count);
    free(buff);
    return str;
#endif
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
    std::cout << "========================================" << std::endl;
    std::cout << "请选择操作 (1-8): ";
}


bool GetBasePathInfo() {

    char szPath[2048] = { 0 };
    char* p = szPath;
#ifdef _WIN32
    GetModuleFileNameA(NULL, szPath, sizeof(szPath)); // �õ�ȫ��·��
    char split = '\\';
#else
    readlink("/proc/self/exe", szPath, sizeof(szPath));
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

    std::cout << "相机型号: " << cam_section << std::endl;
    std::cout << "图像尺寸: " << g_cfg.cam_width << " x " << g_cfg.cam_height << std::endl;

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

    checkAndRestoreRobotHand();
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
        g_appPath + "hand_eye_result.yaml"
    );

    if (ok) {
        // 结果已由 calibrateHandEyeFromDisk 输出并保存
        saveCalibratedCameraParams(camera_matrix, result.camera_matrix, result.dist_coeffs);
    } else {
        std::cerr << "\n[autoCalib] 标定失败: " << result.message << std::endl;
    }
}


static void semi_auto_eye_in_hand_calibration_wrapper(char* pchRobotIP) {
    if (open_camera() != 0) return;

    CharucoBoardConfig board_cfg;
    CharucoDetector detector(board_cfg);

    if (!previewBoardDetection(detector, "Semi-Auto Eye-in-Hand",
                               "请将机器人拖到起始位姿（标定板在相机视野内）",
                               "semiAutoEyeInHand"))
        return;

    // 加载配置或使用默认
    PoseGenerationConfig pose_config = loadPoseGenerationConfig(g_paramPath + "eye_in_hand_config.yaml");

    // 加载已知相机内参（如有）
    cv::Mat camera_matrix, dist_coeffs;
    std::string cam_file = g_paramPath + "camera_params.yaml";  
    std::string save_dir = g_paramPath + "semi_eye_in_hand/";

    CalibrationResult result;
    bool ok = semiAutoEyeInHandCalibration(
        robotHandle, pchRobotIP, g_camera, detector,
        pose_config, board_cfg,
        camera_matrix, dist_coeffs,
        result, save_dir,
        g_appPath + "hand_eye_result.yaml");

    if (ok) {
        saveCalibratedCameraParams(camera_matrix, result.camera_matrix, result.dist_coeffs);
    } else {
        std::cerr << "\n[semiAutoEyeInHand] 标定失败: " << result.message << std::endl;
    }
}

// ============================================================
// 眼在手外标定入口（菜单项 6）
// ============================================================
static void auto_eye_to_hand_calibration(char* pchRobotIP) {
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

    // 加载配置
    PoseGenerationConfig pose_config = loadPoseGenerationConfig(g_paramPath + "eye_to_hand_config.yaml");

    std::string save_dir = g_paramPath + "eye_to_hand/";

    EyeToHandResult result;
    bool ok = autoEyeToHandCharucoCalibration(
        robotHandle, pchRobotIP, g_camera, detector,
        pose_config, result, save_dir);

    if (ok) {
        saveCalibrationResult(g_appPath + "eye_to_hand_result.yaml", "T_base_camera",
                               result.T_base_camera, result.camera_matrix, result.dist_coeffs,
                               result.reproj_error * 1000.0, result.valid_frames);
    } else {
        std::cerr << "\n[eyeToHand] 标定失败: " << result.message << std::endl;
    }
}

// ============================================================
// 奇异点测试分析（菜单项 8）
// ============================================================
static void singularity_test() {
    std::cout << "\n===== 奇异点测试分析 =====" << std::endl;
    std::cout << "机械臂将遍历 x/y 偏移组合，逐步上抬直到不可达" << std::endl;
    std::cout << "请确保机器人处于安全状态，按 Enter 开始..." << std::endl;
    std::cin.get();

    double init_joint[ROB_AXIS_NUM], init_eule[ROB_AXIS_NUM];
    if (cr_get_jointActualPos(robotHandle, init_joint) != sucess) {
        std::cerr << "获取初始关节角失败" << std::endl;
        return;
    }
    convertJointToFlangeEule(robotHandle, init_joint, init_eule);

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "\n初始法兰位姿 (X Y Z RX RY RZ): "
              << init_eule[0] << " " << init_eule[1] << " " << init_eule[2] << "  "
              << init_eule[3] << " " << init_eule[4] << " " << init_eule[5] << std::endl;

    const double x_offsets[] = {-200, -100, 0, 100, 200};
    const double y_offsets[] = {-200, 0, 200};
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
            double target_eule[ROB_AXIS_NUM];
            memcpy(target_eule, init_eule, sizeof(target_eule));
            target_eule[0] = init_eule[0] + x_off;
            target_eule[1] = init_eule[1] + y_off;
            target_eule[2] = init_eule[2];

            double target_joint[ROB_AXIS_NUM];
            if (!convertFlangeOulePos2Joint(robotHandle, target_eule, target_joint)) {
                std::cout << std::left
                          << std::setw(8) << x_off
                          << std::setw(8) << y_off
                          << "IK不可达" << std::endl;
                continue;
            }

            CRresult ret = moveToJointPos(robotHandle, target_joint);
            if (ret != sucess) {
                std::cout << std::left
                          << std::setw(8) << x_off
                          << std::setw(8) << y_off
                          << "运动失败 (ret=" << ret << ")" << std::endl;
                if (ret == operate_timeout)
                    moveToJointPos(robotHandle, init_joint);
                continue;
            }

            // 逐步上抬
            int steps = 0;
            double max_z = init_eule[2];
            target_eule[2] = init_eule[2];

            while (true) {
                target_eule[2] += step_z;

                double next_joint[ROB_AXIS_NUM];
                if (!convertFlangeOulePos2Joint(robotHandle, target_eule, next_joint))
                    break;

                ret = moveToJointPosByLine(robotHandle, next_joint);
                if (ret == operate_timeout) {
                    moveToJointPosByLine(robotHandle, init_joint);
                    break;
                }
                if (ret != sucess)
                    break;

                steps++;
                max_z = target_eule[2];
            }

            double lift = max_z - init_eule[2];
            std::cout << std::fixed << std::setprecision(1);
            std::cout << std::left
                      << std::setw(8) << x_off
                      << std::setw(8) << y_off
                      << std::setw(10) << init_eule[2]
                      << std::setw(10) << max_z
                      << std::setw(10) << lift
                      << std::setw(6) << steps
                      << (steps > 0 ? "OK" : "失败") << std::endl;

            // 回到初始点
            moveToJointPosByLine(robotHandle, init_joint);
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

    // --calibrate-eye-in-hand <save_dir>
    if (strcmp(argv[1], "--calibrate-eye-in-hand") == 0) {
        CharucoBoardConfig board_cfg;
        cv::Mat camera_matrix, dist_coeffs;
        CalibrationResult result;
        if (argc < 3 || !calibrateEyeInHandFromDisk(argv[2], board_cfg,
                camera_matrix, dist_coeffs, result)) {
            std::cerr << "\n[main] 离线标定失败: " << result.message << std::endl;
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

    GetBasePathInfo();

    if (handleCLI(argc, argv))
        return 0;

    int choice = 0;

    std::setlocale(LC_ALL, "en_US.utf8");

    std::cout << "当前路径：" << g_appPath << std::endl;

    load_config(g_appPath+"config.yaml");

    // 初始化机器人
    char *pchRobotIp = "192.168.1.6";

    CRresult result;
    result = cr_create_robot(&robotHandle, pchRobotIp, 2323, "123");  // 机械臂连接 
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
            auto_handeye_calibration(150, pchRobotIp);
            break;
        case 2:
            semi_auto_eye_in_hand_calibration_wrapper(pchRobotIp);
            break;
        case 3:
            auto_eye_to_hand_calibration(pchRobotIp);
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
        default:
            std::cout << "无效选择，请重新输入 (1-8)" << std::endl;
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