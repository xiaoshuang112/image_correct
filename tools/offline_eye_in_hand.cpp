#include "calibration/semi_auto_eye_in_hand_calibration.hpp"
#include "calibration/calibration_session.hpp"

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
    if (cameraMatrix.empty()) {
        storage["intrinsic"]["camera_matrix"] >> cameraMatrix;
        storage["intrinsic"]["distortion_coefficients"] >> distortion;
    }
    storage.release();
    return cameraMatrix.rows == 3 && cameraMatrix.cols == 3 &&
           !distortion.empty() && cv::checkRange(cameraMatrix) &&
           cv::checkRange(distortion);
}

void loadBoardConfig(const std::string& sessionDirectory,
                     CharucoBoardConfig& config) {
    cv::FileStorage storage(sessionDirectory + "/effective_config.yaml",
                            cv::FileStorage::READ);
    if (!storage.isOpened())
        return;
    const cv::FileNode board = storage["board"];
    if (board.empty())
        return;
    board["dictionary_id"] >> config.dictionary_id;
    board["squares_x"] >> config.squares_x;
    board["squares_y"] >> config.squares_y;
    board["square_length_m"] >> config.square_length;
    board["marker_length_m"] >> config.marker_length;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--verify-session") {
        std::vector<std::string> problems;
        if (!CalibrationSession::verifyManifest(argv[2], problems)) {
            std::cerr << "[offlineEyeInHand] Session 完整性校验失败" << std::endl;
            for (const std::string& problem : problems)
                std::cerr << "  " << problem << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "[offlineEyeInHand] Session 完整性校验通过: "
                  << argv[2] << std::endl;
        return EXIT_SUCCESS;
    }
    if (argc == 4 && std::string(argv[1]) == "--replay-session") {
        const std::string sessionDirectory = argv[2];
        cv::Mat cameraMatrix;
        cv::Mat distortion;
        if (!loadIntrinsics(sessionDirectory + "/effective_config.yaml",
                            cameraMatrix, distortion)) {
            std::cerr << "[offlineEyeInHand] Session 内参无效" << std::endl;
            return EXIT_FAILURE;
        }
        CharucoBoardConfig boardConfig;
        loadBoardConfig(sessionDirectory, boardConfig);
        CalibrationResult result;
        if (!calibrateHandEyeFromDisk(sessionDirectory, cameraMatrix, distortion,
                                      boardConfig, result, argv[3], nullptr,
                                      sessionDirectory + "/effective_config.yaml")) {
            std::cerr << "[offlineEyeInHand] Session 复算失败: "
                      << result.message << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "[offlineEyeInHand] Session 复算成功"
                  << ", valid_frames=" << result.valid_frames
                  << ", translation_rms_mm=" << result.reproj_error * 1000.0
                  << ", session=" << result.session_dir
                  << ", output=" << argv[3] << std::endl;
        return EXIT_SUCCESS;
    }
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <calibration_dir> <camera_params.yaml> <output.yaml>"
                  << "\n       " << argv[0] << " --verify-session <session_dir>"
                  << "\n       " << argv[0] << " --replay-session <session_dir> <output.yaml>"
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
                                  boardConfig, result, argv[3], nullptr, argv[2])) {
        std::cerr << "[offlineEyeInHand] 标定失败: " << result.message
                  << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "[offlineEyeInHand] 标定成功"
              << ", valid_frames=" << result.valid_frames
              << ", translation_rms_mm=" << result.reproj_error * 1000.0
              << ", session=" << result.session_dir
              << ", output=" << argv[3] << std::endl;
    return EXIT_SUCCESS;
}
