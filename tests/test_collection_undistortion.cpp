#include "utils/collection_undistortion.hpp"
#include "hand_eye_gui/CollectionImageRecord.hpp"
#include "hand_eye_gui/CollectionPoseRecord.hpp"
#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
template<class F> void rejects(F action) {
    bool rejected = false;
    try { action(); } catch (const std::exception&) { rejected = true; }
    require(rejected, "invalid parameters accepted");
}
QByteArray read(const QString& path) {
    QFile file(path); require(file.open(QIODevice::ReadOnly), "cannot read saved file");
    return file.readAll();
}
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    try {
        QTemporaryDir temp;
        require(temp.isValid(), "temporary directory failed");
        const auto path = temp.filePath("hand_eye_result.yaml").toStdString();
        const cv::Mat k = (cv::Mat_<double>(3, 3) << 180, 0, 160, 0, 180, 120, 0, 0, 1);
        const cv::Mat d = (cv::Mat_<double>(1, 5) << -0.24, 0.03, 0.002, -0.001, 0.0);
        auto write = [&](const cv::Mat& matrix, const cv::Mat& distortion,
                         const char* key = "distortion_coefficients", bool identity = true) {
            cv::FileStorage file(path, cv::FileStorage::WRITE);
            file << "T_flange_camera" << cv::Mat::eye(4, 4, CV_64F);
            if (!matrix.empty()) file << "camera_matrix" << matrix;
            if (!distortion.empty()) file << key << distortion;
            if (identity) file << "image_width" << 320 << "image_height" << 240 << "camera_serial" << "CAM-1";
        };
        write(k, d);
        auto calibration = collection::UndistortionCalibration::load(path);
        collection::UndistortionMaps maps(calibration, {320, 240}, "CAM-1");
        rejects([&] { collection::UndistortionMaps wrong(calibration, {640, 480}, "CAM-1"); });
        rejects([&] { collection::UndistortionMaps wrong(calibration, {320, 240}, "CAM-2"); });
        rejects([&] { collection::UndistortionMaps wrong(calibration, {320, 240}, ""); });
        rejects([&] { maps.apply(cv::Mat()); });
        rejects([&] { maps.apply(cv::Mat(480, 640, CV_8UC3)); });

        cv::Mat raw(240, 320, CV_8UC3);
        cv::RNG random(5342); random.fill(raw, cv::RNG::UNIFORM, 0, 256);
        const cv::Mat original = raw.clone();
        const auto corrected = maps.apply(raw);
        cv::Mat reference; cv::undistort(raw, reference, k, d, k);
        require(cv::norm(corrected, reference, cv::NORM_INF) <= 1, "correction differs from OpenCV reference");
        require(cv::norm(corrected, raw) > 100, "nonzero distortion did not correct image");
        require(cv::norm(raw, original) == 0, "correction modified raw image");
        require(corrected.size() == raw.size() && corrected.type() == raw.type(), "correction changed image format");

        // This is the same persistence function used by capturePhoto. The
        // primary image and recent thumbnail must point at corrected pixels.
        const QString rawPath = temp.filePath("capture_fused.png");
        const auto primary = collection::saveCaptureImages(raw, corrected, rawPath);
        require(primary == temp.filePath("capture_fused_undistorted.png"), "primary path is not corrected image");
        require(cv::norm(cv::imread(primary.toStdString()), corrected) == 0, "saved primary is not corrected");
        require(cv::norm(cv::imread(rawPath.toStdString()), original) == 0, "raw backup lost pixels");
        const double pose[] = {1, 2, 3, 4, 5, 6};
        require(collection::saveFlangePoseRecords(primary, pose).isEmpty(), "corrected pose failed");
        const auto initialMatrix = read(temp.filePath("T_base_flange.json"));
        require(collection::saveFlangePoseRecords(rawPath, pose).isEmpty(), "raw pose failed");
        require(read(temp.filePath("capture_fused.json")) == read(temp.filePath("capture_fused_undistorted.json")),
                "raw and corrected images have different pose metadata");
        require(read(temp.filePath("T_base_flange.json")) == initialMatrix, "paired save overwrote initial matrix");
        const auto savedRaw = read(rawPath), savedCorrected = read(primary);
        rejects([&] { collection::saveCaptureImages(raw, corrected, rawPath); });
        require(read(rawPath) == savedRaw && read(primary) == savedCorrected, "collision overwrote images");
        const auto offPath = temp.filePath("disabled.png");
        require(collection::saveCaptureImages(raw, {}, offPath) == offPath, "disabled mode changed primary path");
        require(!QFile::exists(temp.filePath("disabled_undistorted.png")), "disabled mode saved corrected image");
        require(cv::norm(cv::imread(offPath.toStdString()), raw) == 0, "disabled mode changed image");
        rejects([&] { collection::saveCaptureImages(raw, corrected, temp.filePath("absent/capture.png")); });
        rejects([&] { collection::saveCaptureImages(raw, cv::Mat(5, 5, CV_8UC3), temp.filePath("bad.png")); });
        require(!QFile::exists(temp.filePath("bad.png")), "invalid correction created partial image");

        // Legacy hand-eye results use dist_coeffs and lack identity. Accept
        // these without pretending that their resolution was verified.
        write(k, d.t(), "dist_coeffs", false);
        calibration = collection::UndistortionCalibration::load(path);
        require(calibration.calibratedSize.empty() && calibration.cameraSerial.empty(), "legacy identity invented");
        collection::UndistortionMaps legacy(calibration, raw.size(), "CAM-1");
        require(cv::norm(legacy.apply(raw), corrected) == 0, "legacy coefficient alias changed correction");
        write(k, cv::Mat::zeros(1, 5, CV_64F));
        collection::UndistortionMaps zero(collection::UndistortionCalibration::load(path), raw.size(), "CAM-1");
        require(cv::norm(zero.apply(raw), raw) == 0, "zero distortion changed pixels");
        write({}, {}); rejects([&] { collection::UndistortionCalibration::load(path); });
        write(k, {}); rejects([&] { collection::UndistortionCalibration::load(path); });
        write(cv::Mat::eye(4, 4, CV_64F), d); rejects([&] { collection::UndistortionCalibration::load(path); });
        cv::Mat badK = k.clone(); badK.at<double>(0, 0) = 0;
        write(badK, d); rejects([&] { collection::UndistortionCalibration::load(path); });
        badK = k.clone(); badK.at<double>(1, 1) = std::numeric_limits<double>::quiet_NaN();
        write(badK, d); rejects([&] { collection::UndistortionCalibration::load(path); });
        write(k, cv::Mat::zeros(1, 6, CV_64F)); rejects([&] { collection::UndistortionCalibration::load(path); });
        write(k, cv::Mat::zeros(2, 2, CV_64F)); rejects([&] { collection::UndistortionCalibration::load(path); });
        for (int length : {4, 5, 8, 12, 14}) {
            write(k, cv::Mat::zeros(1, length, CV_64F));
            collection::UndistortionCalibration::load(path);
        }
        write(k, d);
        { cv::FileStorage file(path, cv::FileStorage::APPEND); file << "distortion_model" << "fisheye"; }
        rejects([&] { collection::UndistortionCalibration::load(path); });
        write(k, d, "dist_coeffs", false);
        { cv::FileStorage file(path, cv::FileStorage::APPEND); file << "image_width" << 320; }
        rejects([&] { collection::UndistortionCalibration::load(path); });
        { QFile file(QString::fromStdString(path)); require(file.open(QIODevice::WriteOnly), "cannot create malformed YAML"); file.write("not a YAML file : ["); }
        rejects([&] { collection::UndistortionCalibration::load(path); });
        std::cout << "Collection undistortion, validation, primary image and pose checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
