#include "hand_eye_gui/CollectionCapture.hpp"
#include "utils/collection_undistortion.hpp"
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QTemporaryDir>
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
QJsonObject record(const QString& path) {
    QFile file(path); require(file.open(QIODevice::ReadOnly), "pose record missing");
    return QJsonDocument::fromJson(file.readAll()).object();
}
QString jsonPath(const QString& image) { return image.left(image.size() - 4) + ".json"; }
class Camera : public AICamera {
public:
    Camera() : AICamera(CameraConfig{}) {}
    int acquisitions = 0;
    bool incomplete = false;
    std::function<void()> acquiring;
    bool open() override { return true; }
    void close() override {}
    bool isOpened() const override { return true; }
    cv::Mat captureOneFrame(int) override { return {}; }
    bool setColorExposureTime(int) override { return true; }
    int getColorExposureTime() override { return 5000; }
    bool setAutoExposure(bool) override { return true; }
    bool getAutoExposure() override { return false; }
    bool getExposureRange(int& low, int& high) override { low = 100; high = 50000; return true; }
    std::vector<cv::Mat> captureMultiExposureImages(const std::vector<int>& values) override {
        ++acquisitions;
        if (acquiring) acquiring();
        std::vector<cv::Mat> frames;
        for (int value : values) frames.emplace_back(32, 32, CV_8UC3, cv::Scalar(value / 100, 80, 100));
        if (incomplete) frames.pop_back();
        return frames;
    }
};
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    try {
        QTemporaryDir temp;
        require(temp.isValid(), "temporary directory failed");
        Camera camera;
        std::atomic<bool> cancelled{false};
        const auto time = QDateTime::fromString("2026-11-03T23:59:59.999", Qt::ISODateWithMs);
        const std::array<double, 6> pose{100, 200, 300, 180, 0, 0};
        auto readPose = [&] { return pose; };
        auto capture = [&](const QString& root, const std::function<std::array<double, 6>()>& reader) {
            return collection::captureAndSave(camera, root, time, {5000, 8500}, {}, reader, cancelled);
        };
        const auto first = capture(temp.path(), readPose);
        require(first.complete && QFileInfo::exists(first.path), "valid capture incomplete");
        require(QFileInfo(first.path).dir().dirName() == "2026-11-03", "capture date changed");
        require(QRegularExpression("^capture_20261103_235959_999_[0-9a-f]{8}_fused\\.png$")
                    .match(QFileInfo(first.path).fileName()).hasMatch(), "existing filename format changed");
        require(cv::imread(first.path.toStdString()).size() == cv::Size(32, 32), "PNG invalid");
        const auto json = record(jsonPath(first.path));
        require(json.size() == 6 && json["x"].toDouble() == 100 && json["rz"].toDouble() == 0,
                "existing six-field pose format changed");
        const auto second = capture(temp.path(), readPose);
        require(second.complete && second.path != first.path && QFileInfo::exists(first.path), "duplicate shutter overwrote image");

        const auto noRobot = capture(temp.path(), {});
        require(!noRobot.complete && !noRobot.path.isEmpty() && record(jsonPath(noRobot.path))["x"].isNull(),
                "manual image without robot lost or automatic success fabricated");
        const auto rpc = capture(temp.path(), []() -> std::array<double, 6> { throw std::runtime_error("RPC timeout"); });
        require(!rpc.complete && QFileInfo::exists(rpc.path) && rpc.message.contains("RPC timeout"),
                "RPC failure blocked manual image or enabled next automatic movement");
        const auto invalid = capture(temp.path(), [&] { auto p = pose; p[0] = std::numeric_limits<double>::quiet_NaN(); return p; });
        require(!invalid.complete && record(jsonPath(invalid.path))["x"].isNull(), "nonfinite pose accepted");
        const int before = camera.acquisitions;
        const auto exposure = collection::captureAndSave(camera, temp.path(), time, {1, 8500}, {}, readPose, cancelled);
        require(exposure.path.isEmpty() && camera.acquisitions == before, "unsafe exposure acquired");
        camera.incomplete = true;
        require(capture(temp.path(), readPose).path.isEmpty(), "incomplete bracket saved");
        camera.incomplete = false;
        camera.acquiring = [&] { cancelled = true; };
        require(capture(temp.path(), readPose).path.isEmpty(), "cancelled capture saved");
        cancelled = false; camera.acquiring = {};

        const auto blocked = temp.filePath("blocked");
        QFile blocker(blocked); require(blocker.open(QIODevice::WriteOnly), "cannot create blocked directory"); blocker.close();
        const auto failed = capture(blocked, readPose);
        require(!failed.complete && failed.path.isEmpty(), "failed image save marked successful");

        const auto brokenRoot = temp.filePath("metadata-failure");
        const auto broken = capture(brokenRoot, [&] {
            // Let the image commit, then prevent its pose sidecar from committing.
            QDir day(QDir(brokenRoot).filePath("2026-11-03"));
            for (int i = 0; i < 5000; ++i) {
                const auto images = day.entryList({"*.png"}, QDir::Files);
                if (!images.isEmpty()) {
                    require(QDir().mkdir(jsonPath(day.filePath(images[0]))), "cannot block metadata path");
                    return pose;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            throw std::runtime_error("image did not commit");
        });
        require(!broken.complete && QFileInfo::exists(broken.path) && broken.message.contains("法兰位姿文件保存失败"),
                "metadata failure discarded image or allowed automatic continuation");

        collection::UndistortionCalibration calibration;
        calibration.cameraMatrix = (cv::Mat_<double>(3,3) << 30,0,16, 0,30,16, 0,0,1);
        calibration.distortion = cv::Mat::zeros(1, 5, CV_64F);
        calibration.calibratedSize = {32,32};
        const auto maps = std::make_shared<collection::UndistortionMaps>(calibration, cv::Size(32,32), "");
        const auto corrected = collection::captureAndSave(camera, temp.path(), time, {5000,8500}, maps, readPose, cancelled);
        const auto rawPath = corrected.path;
        QString raw = rawPath; raw.replace("_undistorted.png", ".png");
        require(corrected.complete && corrected.path.endsWith("_fused_undistorted.png") && QFileInfo::exists(raw) &&
                record(jsonPath(raw)) == record(jsonPath(corrected.path)) &&
                QFileInfo::exists(corrected.path.left(corrected.path.size()-4) + ".pixel_frame.json"),
                "undistorted/raw pair or metadata format changed");
        std::cout << "Capture format, pose degradation, cancellation, commit barriers and undistorted pairs passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
