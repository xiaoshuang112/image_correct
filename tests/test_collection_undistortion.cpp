#include "utils/collection_undistortion.hpp"
#include "hand_eye_gui/CollectionImageRecord.hpp"
#include "hand_eye_gui/CollectionPoseRecord.hpp"
#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
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
        cv::Mat computation;
        k.convertTo(computation, CV_32F);
        computation.convertTo(computation, CV_64F);
        cv::Rect roi;
        const cv::Mat newK = cv::getOptimalNewCameraMatrix(
            computation, d, raw.size(), 1.0, raw.size(), &roi, false);
        cv::Mat map1, map2, reference;
        cv::initUndistortRectifyMap(computation, d, cv::Mat(), newK, raw.size(), CV_16SC2, map1, map2);
        cv::remap(raw, reference, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        require(cv::norm(corrected, reference, cv::NORM_INF) == 0, "correction differs from production alpha=1 map");
        require(maps.fixedPointMapType() == CV_16SC2 && maps.interpolationMapType() == CV_16UC1,
                "map type is not the production fixed-point pair");
        const auto& geometry = maps.geometry();
        require(std::string(geometry.pixelFrame) == collection::kProductionAlpha1PixelFrame,
                "new images are not marked with the production pixel frame");
        require(std::string(collection::kLegacyOriginalKPixelFrame) == "collect_original_k_undistorted_full_frame",
                "legacy original-K frame name changed");
        require(cv::norm(geometry.originalCameraMatrix, k) == 0, "original K was replaced");
        require(cv::norm(geometry.computationCameraMatrix, computation) == 0, "computation K skipped the float round trip");
        require(cv::norm(geometry.outputCameraMatrix, newK) == 0, "output K is not the computed new camera matrix");
        require(geometry.inputSize == raw.size() && geometry.outputSize == raw.size() &&
                geometry.alpha == 1.0 && geometry.validPixROI == roi && !geometry.cropped &&
                !geometry.flipHorizontal && !geometry.flipVertical, "full-frame production parameters changed");
        cv::Mat legacyReference;
        cv::undistort(raw, legacyReference, k, d, k);
        require(cv::norm(corrected, legacyReference) > 0, "alpha=1 output still matches original-K undistortion");
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
        require(collection::saveUndistortionMetadata(primary, geometry).isEmpty(), "pixel-frame record failed");
        const auto frame = QJsonDocument::fromJson(read(temp.filePath("capture_fused_undistorted.pixel_frame.json"))).object();
        require(frame["pixel_frame"].toString() == QString::fromLatin1(collection::kProductionAlpha1PixelFrame),
                "saved pixel frame is not the production definition");
        require(!frame["cropped"].toBool() && !frame["flip_horizontal"].toBool() && !frame["flip_vertical"].toBool(),
                "saved record crops or flips the image");
        require(frame["alpha"].toDouble() == 1.0, "saved alpha is not 1");
        require(frame["input_size_wh"].toArray() == frame["output_size_wh"].toArray(), "saved sizes differ");
        require(std::abs(frame["output_camera_matrix"].toArray()[0].toArray()[0].toDouble() - newK.at<double>(0, 0)) < 1e-9,
                "saved K_new differs from the computed matrix");
        require(!QFile::exists(temp.filePath("capture_fused.pixel_frame.json")), "raw image received a pixel-frame record");
        require(collection::saveUndistortionMetadata(primary, geometry).contains(QStringLiteral("不覆盖")),
                "pixel-frame record was replaced");
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

        // Field calibration from the 2026-10-18 production comparison. K_new is
        // computed here; the published values only check that computation.
        const cv::Mat fieldK = (cv::Mat_<double>(3, 3) <<
            740.42150408757766, 0, 615.92682315059858,
            0, 740.86801562742789, 530.66704271527044,
            0, 0, 1);
        const cv::Mat fieldD = (cv::Mat_<double>(1, 5) <<
            -0.13485895392804906, 0.088630466705486202,
            -0.00012377657072698922, -0.0001135698438625164,
            -0.011270877792727303);
        write(fieldK, fieldD, "distortion_coefficients", false);
        { cv::FileStorage file(path, cv::FileStorage::APPEND);
          file << "image_width" << 1280 << "image_height" << 1024 << "camera_serial" << "DB2650007"; }
        const cv::Size fieldSize(1280, 1024);
        collection::UndistortionMaps fieldMaps(collection::UndistortionCalibration::load(path), fieldSize, "DB2650007");
        const cv::Mat& fieldNewK = fieldMaps.geometry().outputCameraMatrix;
        const double expectedNewK[4] = {697.563127074, 615.707952065, 698.282779635, 530.385497800};
        const double actualNewK[4] = {
            fieldNewK.at<double>(0, 0), fieldNewK.at<double>(0, 2),
            fieldNewK.at<double>(1, 1), fieldNewK.at<double>(1, 2)};
        for (int i = 0; i < 4; ++i)
            require(std::abs(actualNewK[i] - expectedNewK[i]) < 1e-4, "field K_new does not match production alpha=1");
        cv::Mat fieldRaw(fieldSize, CV_8UC3);
        cv::RNG fieldRandom(20261018); fieldRandom.fill(fieldRaw, cv::RNG::UNIFORM, 0, 256);
        cv::Mat fieldComputation;
        fieldK.convertTo(fieldComputation, CV_32F);
        fieldComputation.convertTo(fieldComputation, CV_64F);
        cv::Rect fieldRoi;
        const cv::Mat fieldReferenceK = cv::getOptimalNewCameraMatrix(
            fieldComputation, fieldD, fieldSize, 1.0, fieldSize, &fieldRoi, false);
        cv::Mat fieldMap1, fieldMap2, fieldReference;
        cv::initUndistortRectifyMap(fieldComputation, fieldD, cv::Mat(), fieldReferenceK,
            fieldSize, CV_16SC2, fieldMap1, fieldMap2);
        cv::remap(fieldRaw, fieldReference, fieldMap1, fieldMap2, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        const cv::Mat fieldCorrected = fieldMaps.apply(fieldRaw);
        require(fieldCorrected.size() == fieldSize && fieldMaps.geometry().validPixROI == fieldRoi &&
                !fieldMaps.geometry().cropped, "field output was cropped or resized");
        require(cv::norm(fieldCorrected, fieldReference, cv::NORM_INF) == 0,
                "field image differs from the production remap");
        std::vector<uchar> fieldPng;
        require(cv::imencode(".png", fieldCorrected, fieldPng), "field PNG encode failed");
        require(cv::norm(cv::imdecode(fieldPng, cv::IMREAD_COLOR), fieldCorrected) == 0,
                "lossless field PNG changed pixels");
        std::cout << "Collection undistortion, validation, primary image and pose checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
