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
        const QDir day(QDir(temp.path()).filePath("2026-11-03"));
        QString filename = QFileInfo(corrected.path).fileName();
        filename.replace("_undistorted.png", ".png");
        const QString raw = day.filePath("raw/" + filename);
        require(corrected.complete && QFileInfo(corrected.path).dir().absolutePath() == day.filePath("undistorted") &&
                corrected.path.endsWith("_fused_undistorted.png") && QFileInfo::exists(raw) &&
                record(jsonPath(raw)) == record(jsonPath(corrected.path)) &&
                QFileInfo::exists(corrected.path.left(corrected.path.size()-4) + ".pixel_frame.json"),
                "undistorted/raw pair or metadata format changed");
        require(day.entryList({"*.png"}, QDir::Files).size() > 0 &&
                !QFileInfo::exists(day.filePath(filename)) &&
                QFileInfo::exists(day.filePath("raw/T_base_flange.json")) &&
                QFileInfo::exists(day.filePath("undistorted/T_base_flange.json")),
                "paired capture leaked into day root or omitted folder pose records");
        const auto configuredRoot = temp.filePath("configured");
        const auto correctedOnly = collection::captureAndSave(camera, configuredRoot, time,
            {5000,8500}, maps, readPose, cancelled, {}, {"P01", false});
        const QDir configuredDay(QDir(configuredRoot).filePath("2026-11-03"));
        require(correctedOnly.complete && correctedOnly.path == configuredDay.filePath(
                    "undistorted/capture_20261103_235959_999_P01_fused_undistorted.png") &&
                QFileInfo::exists(jsonPath(correctedOnly.path)) &&
                QFileInfo::exists(correctedOnly.path.left(correctedOnly.path.size() - 4) + ".pixel_frame.json") &&
                !QFileInfo::exists(configuredDay.filePath("raw")),
                "corrected-only capture created an original or omitted a corrected record");
        const auto configuredPair = collection::captureAndSave(camera, configuredRoot, time,
            {5000,8500}, maps, readPose, cancelled, {}, {"P02", true});
        const auto configuredRaw = configuredDay.filePath("raw/capture_20261103_235959_999_P02_fused.png");
        require(configuredPair.complete && QFileInfo::exists(configuredRaw) &&
                record(jsonPath(configuredRaw)) == record(jsonPath(configuredPair.path)),
                "checked RAW option did not save a matching original");
        const auto noCorrection = collection::captureAndSave(camera, configuredRoot, time,
            {5000,8500}, {}, readPose, cancelled, {}, {"P03", false});
        require(noCorrection.complete && noCorrection.path == configuredDay.filePath(
                    "capture_20261103_235959_999_P03_fused.png") && QFileInfo::exists(noCorrection.path),
                "unchecked RAW option suppressed the only image without correction");
        int stage = 0;
        collection::ExposureCameraGate gate{
            [&] { require(stage == 0, "preview suspended twice"); stage = 1; },
            [&] { require(stage == 1, "preview resumed out of order"); stage = 2; }};
        camera.acquiring = [&] { require(stage == 1, "exposure changed before the preview frame was released"); };
        const auto gated = collection::captureAndSave(camera, temp.path(), time, {5000, 8500}, {}, readPose, cancelled, gate);
        require(gated.complete && stage == 2, "preview did not resume after the exposure was restored");
        stage = 0; camera.incomplete = true;
        const auto gatedGap = collection::captureAndSave(camera, temp.path(), time, {5000, 8500}, {}, readPose, cancelled, gate);
        require(!gatedGap.complete && gatedGap.path.isEmpty() && stage == 2, "incomplete exposure left preview suspended");
        stage = 0; camera.incomplete = false;
        camera.acquiring = [&] { throw std::runtime_error("exposure failed"); };
        const auto gatedError = collection::captureAndSave(camera, temp.path(), time, {5000, 8500}, {}, readPose, cancelled, gate);
        require(!gatedError.complete && gatedError.path.isEmpty() && stage == 2 && gatedError.message.contains("exposure failed"),
                "exposure exception left preview suspended");
        camera.acquiring = {};

        // Dual-camera parallel capture & paired saving under <date>/cam1 and <date>/cam2
        Camera cam1, cam2;
        std::atomic<int> inFlight{0};
        std::atomic<bool> overlapped{false};
        cam1.acquiring = [&] {
            ++inFlight;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (inFlight.load() < 2 && std::chrono::steady_clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            if (inFlight.load() >= 2) overlapped = true;
            --inFlight;
        };
        cam2.acquiring = [&] {
            ++inFlight;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (!overlapped.load() && std::chrono::steady_clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            --inFlight;
        };
        collection::UndistortionCalibration cal2;
        cal2.cameraMatrix = (cv::Mat_<double>(3, 3) << 28, 0, 16, 0, 28, 16, 0, 0, 1);
        cal2.distortion = cv::Mat::zeros(1, 5, CV_64F);
        cal2.calibratedSize = {32, 32};
        const auto maps2 = std::make_shared<collection::UndistortionMaps>(cal2, cv::Size(32, 32), "");

        const QString dualRoot = temp.filePath("dual-out");
        const auto dualBoth = collection::captureDualAndSave(
            cam1, cam2, dualRoot, time, {5000, 8500}, {6000, 9500},
            maps, maps2, readPose, cancelled, {}, {"P01", true});
        const QDir dualDay(QDir(dualRoot).filePath("2026-11-03"));
        const QString expectedCam1 = dualDay.filePath("cam1/undistorted/capture_20261103_235959_999_P01_fused_undistorted.png");
        const QString expectedCam2 = dualDay.filePath("cam2/undistorted/capture_20261103_235959_999_P01_fused_undistorted.png");
        const QString expectedRaw1 = dualDay.filePath("cam1/raw/capture_20261103_235959_999_P01_fused.png");
        const QString expectedRaw2 = dualDay.filePath("cam2/raw/capture_20261103_235959_999_P01_fused.png");
        require(overlapped.load(), "dual cameras did not acquire in parallel");
        require(dualBoth.complete && dualBoth.path1 == expectedCam1 && dualBoth.path2 == expectedCam2 &&
                QFileInfo::exists(expectedCam1) && QFileInfo::exists(expectedCam2) &&
                QFileInfo::exists(expectedRaw1) && QFileInfo::exists(expectedRaw2) &&
                record(jsonPath(expectedCam1)) == record(jsonPath(expectedCam2)) &&
                QFileInfo::exists(expectedCam1.left(expectedCam1.size() - 4) + ".pixel_frame.json") &&
                QFileInfo::exists(expectedCam2.left(expectedCam2.size() - 4) + ".pixel_frame.json"),
                "dual paired output layout or metadata mismatch");
        cam1.acquiring = {};
        cam2.acquiring = {};

        // Camera 2 failure rolls back/avoids saving Camera 1 half-pair
        cam2.incomplete = true;
        const QString rollbackRoot = temp.filePath("dual-rollback");
        const auto dualIncomplete = collection::captureDualAndSave(
            cam1, cam2, rollbackRoot, time, {5000, 8500}, {6000, 9500},
            maps, {}, readPose, cancelled, {}, {"P02", true});
        require(!dualIncomplete.complete && dualIncomplete.path1.isEmpty() && dualIncomplete.path2.isEmpty() &&
                !QDir(rollbackRoot).exists("2026-11-03/cam1/undistorted"),
                "incomplete cam2 saved half pair");
        cam2.incomplete = false;

        // Camera 2 PNG write failure removes Camera 1 PNGs just written
        const QString writeFailRoot = temp.filePath("dual-write-fail");
        const QDir writeFailDay(QDir(writeFailRoot).filePath("2026-11-03"));
        require(QDir().mkpath(writeFailDay.absolutePath()), "cannot create dual write-fail day dir");
        QFile cam2Blocker(writeFailDay.filePath("cam2"));
        require(cam2Blocker.open(QIODevice::WriteOnly), "cannot create cam2 file blocker");
        cam2Blocker.close();
        const auto dualWriteFail = collection::captureDualAndSave(
            cam1, cam2, writeFailRoot, time, {5000, 8500}, {6000, 9500},
            maps, {}, readPose, cancelled, {}, {"P03", true});
        require(!dualWriteFail.complete && dualWriteFail.path1.isEmpty() &&
                writeFailDay.entryList({"*.png"}, QDir::Files).isEmpty() &&
                !QFileInfo::exists(writeFailDay.filePath("cam1/undistorted/capture_20261103_235959_999_P03_fused_undistorted.png")) &&
                !QFileInfo::exists(writeFailDay.filePath("cam1/raw/capture_20261103_235959_999_P03_fused.png")),
                "cam2 write failure left cam1 PNGs behind");

        // Pose RPC failure produces null poses and complete=false
        const auto dualNoPose = collection::captureDualAndSave(
            cam1, cam2, dualRoot, time, {5000, 8500}, {6000, 9500},
            {}, {}, []() -> std::array<double, 6> { throw std::runtime_error("RPC timeout"); },
            cancelled, {}, {"P04", false});
        require(!dualNoPose.complete && !dualNoPose.path1.isEmpty() && !dualNoPose.path2.isEmpty() &&
                record(jsonPath(dualNoPose.path1))["x"].isNull() &&
                record(jsonPath(dualNoPose.path2))["x"].isNull(),
                "dual pose RPC failure did not degrade to null or reported complete");

        std::cout << "Capture format, pose degradation, cancellation, commit barriers and undistorted pairs passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
