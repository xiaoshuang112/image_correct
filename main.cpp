#include "OrbbecCamera.hpp"
#include <iostream>

int main() {
    // 方式1：使用默认配置
    Config config;
    config.ip = "192.168.1.10";
    config.width = 1280;
    config.height = 800;
    config.fps = 30;
    
    OrbbecCameraV2 camera(config);
    
    // 打开相机
    if (!camera.open()) {
        std::cerr << "打开相机失败" << std::endl;
        return -1;
    }
    
    std::cout << "开始预览，按 'ESC' 或 'q' 退出" << std::endl;
    cv::namedWindow("Camera Preview", cv::WINDOW_NORMAL);
    
    // 持续捕获并显示
    while (true) {
        // 捕获一帧
        cv::Mat frame = camera.captureOneFrame(1000);
        
        if (frame.empty()) {
            std::cerr << "捕获帧失败" << std::endl;
            break;
        }
        
        // 显示图像
        // cv::imshow("Camera Preview", frame);

        cv::imwrite("1.jpg",frame);
        break;
        
        // 检查退出条件
        char key = (char)cv::waitKey(1);
        if (key == 27 || key == 'q') {
            std::cout << "退出程序" << std::endl;
            break;
        }
    }
    
    // 关闭相机（析构函数会自动调用，但显式调用更好）
    camera.close();
    cv::destroyAllWindows();
    
    return 0;
}