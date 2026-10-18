#include "utils/collection_utils.hpp"
#include "utils/calib_transform.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
template<class F> void rejects(F operation) {
    bool rejected = false;
    try { operation(); } catch (const std::exception&) { rejected = true; }
    require(rejected, "invalid input accepted");
}
void checkLevel(const cv::Mat& baseFlange, const cv::Mat& flangeCamera) {
    const cv::Mat before = baseFlange * flangeCamera;
    const cv::Mat target = collection::downwardFlangeTarget(baseFlange, flangeCamera);
    const cv::Mat after = target * flangeCamera;
    collection::validateTransform(target);
    require(cv::norm(before(cv::Rect(3, 0, 1, 3)) - after(cv::Rect(3, 0, 1, 3))) < 1e-9,
            "levelling moved optical centre");
    require(cv::norm(after(cv::Rect(2, 0, 1, 3)) - (cv::Mat_<double>(3, 1) << 0, 0, -1)) < 1e-9,
            "camera does not point down");
    const double x = before.at<double>(0, 0), y = before.at<double>(1, 0);
    if (std::hypot(x, y) > 1e-6) {
        require(std::abs(after.at<double>(0, 0) - x / std::hypot(x, y)) < 1e-9 &&
                std::abs(after.at<double>(1, 0) - y / std::hypot(x, y)) < 1e-9,
                "horizontal heading changed");
    }
    double pose[6]{};
    calib_transform::transformToAxisAnglePose(target, pose);
    require(cv::norm(calib_transform::axisAnglePoseToTransform(pose) - target) < 1e-8,
            "robot pose conversion changed target or units");
    require(cv::norm(collection::downwardFlangeTarget(target, flangeCamera) - target) < 1e-8,
            "already level camera moved");
}
}

int main() {
    try {
        // Sweep mounting offsets, tilts and yaw; translations here are metres.
        for (int i = 0; i < 90; ++i) {
            double robot[] = {500, -200, 750, -110.0 + i, 17.0, 2.0 * i};
            double mounting[] = {25, -34, 91, 3.0, -2.0, 76.0};
            checkLevel(calib_transform::axisAnglePoseToTransform(robot),
                       calib_transform::axisAnglePoseToTransform(mounting));
        }
        // Camera X pointing vertically exercises the heading fallback.
        double vertical[] = {200, 300, 500, 0, 90, 0};
        checkLevel(calib_transform::axisAnglePoseToTransform(vertical), cv::Mat::eye(4, 4, CV_64F));
        rejects([] { collection::validateTransform(cv::Mat()); });
        cv::Mat bad = cv::Mat::eye(4, 4, CV_64F);
        bad.at<double>(2, 2) = -1;
        rejects([&] { collection::validateTransform(bad); });
        bad = cv::Mat::eye(4, 4, CV_64F); bad.at<double>(3, 0) = 1;
        rejects([&] { collection::validateTransform(bad); });

        std::vector<cv::Mat> exposures;
        cv::Mat scene(65, 97, CV_8UC3);
        for (int row = 0; row < scene.rows; ++row)
            for (int col = 0; col < scene.cols; ++col)
                scene.at<cv::Vec3b>(row, col) = cv::Vec3b(col * 2, row * 3, (col + row) % 256);
        for (double gain : {0.4, 1.0, 2.0}) {
            cv::Mat frame; scene.convertTo(frame, CV_8UC3, gain); exposures.push_back(frame);
        }
        cv::Mat fused = collection::fuseMertens(exposures);
        require(fused.size() == scene.size() && fused.type() == CV_8UC3, "invalid fused image");
        require(cv::norm(fused, exposures.back()) > 0, "fusion returned only last exposure");
        std::vector<uchar> png;
        require(cv::imencode(".png", fused, png), "PNG encode failed");
        require(cv::norm(cv::imdecode(png, cv::IMREAD_COLOR), fused) == 0, "saved image lost pixels");
        // Synthetic equal-channel BGR inputs exercise fusion/PNG only.
        // cvtColor here is OpenCV test setup, not MVS Mono8 conversion coverage.
        std::vector<cv::Mat> equalChannelBgr;
        cv::Mat gray(scene.size(), CV_8UC1);
        for (int row = 0; row < gray.rows; ++row)
            for (int col = 0; col < gray.cols; ++col)
                gray.at<uchar>(row, col) = (row + col) % 160;
        for (double gain : {0.4, 1.0, 2.0}) {
            cv::Mat frame, bgr;
            gray.convertTo(frame, CV_8UC1, gain);
            cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
            equalChannelBgr.push_back(bgr);
        }
        const cv::Mat equalChannelFused = collection::fuseMertens(equalChannelBgr);
        require(equalChannelFused.type() == CV_8UC3 && equalChannelFused.size() == gray.size(),
                "synthetic equal-channel BGR fusion contract changed");
        std::vector<cv::Mat> channels;
        cv::split(equalChannelFused, channels);
        require(cv::norm(channels[0], channels[1]) == 0 &&
                cv::norm(channels[1], channels[2]) == 0, "synthetic equal-channel BGR fusion introduced color");
        png.clear();
        require(cv::imencode(".png", equalChannelFused, png), "synthetic equal-channel BGR PNG encode failed");
        require(cv::norm(cv::imdecode(png, cv::IMREAD_COLOR), equalChannelFused) == 0,
                "saved synthetic equal-channel BGR image lost pixels");
        rejects([&] { collection::fuseMertens({exposures.front()}); });
        rejects([&] { collection::fuseMertens({exposures.front(), cv::Mat()}); });
        rejects([&] { collection::fuseMertens({exposures.front(), cv::Mat(66, 97, CV_8UC3)}); });
        rejects([&] { collection::fuseMertens({exposures.front(), cv::Mat(65, 97, CV_8UC1)}); });
        std::cout << "Collection geometry, synthetic BGR fusion and PNG checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
