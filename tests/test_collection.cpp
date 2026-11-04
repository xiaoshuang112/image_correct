#include "utils/collection_utils.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
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

}

int main() {
    try {
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
        std::cout << "Collection synthetic BGR fusion and PNG checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
