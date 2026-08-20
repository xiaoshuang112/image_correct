#include "calibration/semi_auto_eye_in_hand_calibration.hpp"

#include <opencv2/core.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool loadIntrinsics(const std::string& path, cv::Mat& cameraMatrix,
                    cv::Mat& distortion) {
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened())
        return false;
    storage["camera_matrix"] >> cameraMatrix;
    storage["distortion_coefficients"] >> distortion;
    storage.release();
    return cameraMatrix.rows == 3 && cameraMatrix.cols == 3 &&
           !distortion.empty() && cv::checkRange(cameraMatrix) &&
           cv::checkRange(distortion);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <calibration_dir> <camera_params.yaml> <output.yaml>"
                  << std::endl;
        return EXIT_FAILURE;
    }

    cv::Mat cameraMatrix;
    cv::Mat distortion;
    if (!loadIntrinsics(argv[2], cameraMatrix, distortion)) {
        std::cerr << "[offlineEyeInHand] 无法读取相机内参: " << argv[2]
                  << std::endl;
        return EXIT_FAILURE;
    }

    CharucoBoardConfig boardConfig;
    CalibrationResult result;
    if (!calibrateHandEyeFromDisk(argv[1], cameraMatrix, distortion,
                                  boardConfig, result, argv[3])) {
        std::cerr << "[offlineEyeInHand] 标定失败: " << result.message
                  << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "[offlineEyeInHand] 标定成功"
              << ", valid_frames=" << result.valid_frames
              << ", translation_rms_mm=" << result.reproj_error * 1000.0
              << ", output=" << argv[3] << std::endl;
    return EXIT_SUCCESS;
}
