#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " --anti-distortion <image_dir>\n"
              << "  [--camera-params <yaml_path>]  指定相机参数文件（默认 hand_eye_result.yaml）\n"
              << "  [--eye-to-hand]                使用 eye_to_hand_result.yaml 读取内参\n";
}

static bool loadCameraParams(const std::string& path, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "无法打开相机参数文件: " << path << std::endl;
        return false;
    }
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    if (dist_coeffs.empty())
        fs["distortion_coefficients"] >> dist_coeffs;
    fs.release();

    if (camera_matrix.empty() || dist_coeffs.empty()) {
        std::cerr << "相机参数文件中缺少 camera_matrix 或 dist_coeffs" << std::endl;
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    std::string image_dir;
    std::string cam_params_path = "hand_eye_result.yaml";
    bool eye_to_hand = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--anti-distortion" && i + 1 < argc) {
            image_dir = argv[++i];
        } else if (arg == "--camera-params" && i + 1 < argc) {
            cam_params_path = argv[++i];
        } else if (arg == "--eye-to-hand") {
            eye_to_hand = true;
        } else {
            std::cerr << "未知参数: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    if (image_dir.empty()) {
        printUsage(argv[0]);
        return 1;
    }

    if (eye_to_hand && cam_params_path == "hand_eye_result.yaml")
        cam_params_path = "eye_to_hand_result.yaml";

    if (!fs::exists(image_dir)) {
        std::cerr << "图像目录不存在: " << image_dir << std::endl;
        return 1;
    }

    while (!image_dir.empty() && image_dir.back() == '/')
        image_dir.pop_back();

    cv::Mat camera_matrix, dist_coeffs;
    if (!loadCameraParams(cam_params_path, camera_matrix, dist_coeffs))
        return 1;

    std::cout << "相机内参文件: " << cam_params_path << std::endl;
    std::cout << "camera_matrix: " << camera_matrix << std::endl;
    std::cout << "dist_coeffs: " << dist_coeffs << std::endl;

    std::string out_dir = image_dir + "_anti";
    if (!fs::exists(out_dir)) {
        fs::create_directory(out_dir);
    }

    std::vector<std::string> exts = {".jpg", ".jpeg", ".png", ".bmp", ".tiff"};
    int count = 0;

    for (const auto& entry : fs::directory_iterator(image_dir)) {
        if (!entry.is_regular_file()) continue;

        std::string ext = entry.path().extension().string();
        std::string lower_ext;
        for (char c : ext) lower_ext += (char)std::tolower(c);

        bool matched = false;
        for (const auto& e : exts) {
            if (lower_ext == e) { matched = true; break; }
        }
        if (!matched) continue;

        cv::Mat img = cv::imread(entry.path().string());
        if (img.empty()) {
            std::cerr << "无法读取: " << entry.path().string() << std::endl;
            continue;
        }

        cv::Mat undistorted;
        cv::undistort(img, undistorted, camera_matrix, dist_coeffs);

        std::string out_path = out_dir + "/" + entry.path().filename().string();
        cv::imwrite(out_path, undistorted);
        ++count;
    }

    std::cout << "反畸变完成，共处理 " << count << " 张图片，输出到: " << out_dir << std::endl;
    return 0;
}
