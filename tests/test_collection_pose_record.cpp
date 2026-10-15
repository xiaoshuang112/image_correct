#include "hand_eye_gui/CollectionPoseRecord.hpp"
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
QByteArray read(const QString& path) {
    QFile file(path);
    require(file.open(QIODevice::ReadOnly), "cannot read record");
    return file.readAll();
}
QJsonObject object(const QString& path) {
    QJsonParseError error;
    const auto document = QJsonDocument::fromJson(read(path), &error);
    require(error.error == QJsonParseError::NoError && document.isObject(), "invalid JSON record");
    return document.object();
}
void checkNullPose(const QString& path) {
    const auto pose = object(path);
    require(pose.size() == 6, "invalid pose has extra fields");
    for (const char* key : {"x", "y", "z", "rx", "ry", "rz"})
        require(pose.value(key).isNull(), "missing pose was replaced by numeric data");
}
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    try {
        QTemporaryDir temp;
        require(temp.isValid(), "cannot create temporary directory");
        const QString image = temp.filePath("first_fused.png");
        QFile png(image);
        require(png.open(QIODevice::WriteOnly), "cannot create image placeholder");
        png.write("image must remain intact"); png.close();
        const QByteArray imageBytes = read(image);

        // A failed initial read still records nulls, leaves the image alone,
        // and does not reserve the initial matrix with fabricated data.
        require(collection::saveFlangePoseRecords(image, nullptr).isEmpty(), "null record failed");
        checkNullPose(temp.filePath("first_fused.json"));
        const QString matrixPath = temp.filePath("T_base_flange.json");
        require(!QFile::exists(matrixPath), "invalid initial pose created matrix");
        require(read(image) == imageBytes, "record handling changed image");

        const double pose[] = {500, -200, 750, 0, 0, 90};
        require(collection::saveFlangePoseRecords(image, pose).isEmpty(), "valid record failed");
        const auto values = object(temp.filePath("first_fused.json"));
        require(values.size() == 6 && values["x"].toDouble() == 500 &&
                values["y"].toDouble() == -200 && values["rz"].toDouble() == 90,
                "raw pose format or units changed");
        const auto matrix = object(matrixPath);
        require(matrix.size() == 1, "matrix file contains extra fields");
        const auto rows = matrix["T_base_flange"].toArray();
        require(rows.size() == 4, "matrix is not 4x4");
        for (const auto& row : rows) require(row.toArray().size() == 4, "matrix row has wrong size");
        require(std::abs(rows[0].toArray()[3].toDouble() - 0.5) < 1e-12 &&
                std::abs(rows[1].toArray()[3].toDouble() + 0.2) < 1e-12 &&
                std::abs(rows[2].toArray()[3].toDouble() - 0.75) < 1e-12,
                "matrix translation is not metres");
        require(std::abs(rows[0].toArray()[1].toDouble() + 1) < 1e-12 &&
                std::abs(rows[1].toArray()[0].toDouble() - 1) < 1e-12 &&
                rows[3].toArray()[3].toDouble() == 1,
                "axis-angle rotation or homogeneous matrix is wrong");
        const auto initialMatrix = read(matrixPath);

        // Each photograph gets its current raw pose; even a new helper call
        // must preserve the first matrix already stored on disk.
        const double later[] = {1000, 2000, 3000, 10, 20, 30};
        require(collection::saveFlangePoseRecords(temp.filePath("second.png"), later).isEmpty(),
                "later record failed");
        require(object(temp.filePath("second.json"))["x"].toDouble() == 1000,
                "later photograph reused first pose");
        require(read(matrixPath) == initialMatrix, "later pose overwrote initial matrix");
        require(collection::saveFlangePoseRecords(temp.filePath("third.png"), nullptr).isEmpty(),
                "later failed read record failed");
        checkNullPose(temp.filePath("third.json"));
        require(read(matrixPath) == initialMatrix, "failed read overwrote initial matrix");

        double invalid[] = {1, 2, 3, 4, 5, std::numeric_limits<double>::quiet_NaN()};
        require(collection::saveFlangePoseRecords(temp.filePath("invalid.png"), invalid).isEmpty(),
                "invalid numeric pose record failed");
        checkNullPose(temp.filePath("invalid.json"));

        // Disk failures report a warning and leave the existing image intact.
        require(QDir().mkpath(temp.filePath("first_fused.json.blocked")), "cannot create blocked path");
        QFile::remove(temp.filePath("first_fused.json"));
        require(QDir().rename(temp.filePath("first_fused.json.blocked"), temp.filePath("first_fused.json")),
                "cannot block record destination");
        require(!collection::saveFlangePoseRecords(image, later).isEmpty(), "disk error was hidden");
        require(read(image) == imageBytes && read(matrixPath) == initialMatrix,
                "record failure changed image or initial matrix");

        // Standalone points contain exactly six raw values and never create an image.
        const QString points = temp.filePath("standalone");
        QString saved;
        require(collection::saveStandaloneFlangePose(points, "pose_工位A", pose, saved).isEmpty(),
                "standalone point save failed");
        require(saved == QDir(points).filePath("pose_工位A.json"), "point name was not preserved");
        const QByteArray firstPoint = read(saved);
        require(firstPoint == QJsonDocument(values).toJson(QJsonDocument::Indented),
                "standalone JSON differs from per-image pose JSON for the same pose");
        const auto pointValues = object(saved);
        require(pointValues.size() == 6 && pointValues["x"].toDouble() == 500 &&
                pointValues["rz"].toDouble() == 90, "standalone pose format or units changed");
        const QString pointMatrix = QDir(points).filePath("T_base_flange.json");
        const auto initialPointMatrix = read(pointMatrix);
        require(initialPointMatrix == initialMatrix, "standalone initial matrix differs from image matrix");
        require(QDir(points).entryList({"*.png", "*.jpg"}, QDir::Files).isEmpty(),
                "standalone save unexpectedly created an image");
        require(collection::saveStandaloneFlangePose(points, "pose_工位A", later, saved).isEmpty(),
                "duplicate point save failed");
        require(saved.endsWith("pose_工位A_2.json"), "duplicate point did not receive suffix");
        require(read(QDir(points).filePath("pose_工位A.json")) == firstPoint,
                "duplicate point overwrote original");
        require(read(pointMatrix) == initialPointMatrix, "standalone point replaced initial matrix");
        require(collection::saveStandaloneFlangePose(points, "pose_工位A", later, saved).isEmpty() &&
                saved.endsWith("pose_工位A_3.json"), "third duplicate reused an occupied name");
        require(collection::saveStandaloneFlangePose(points, "pose_20261015_123456_001", later, saved).isEmpty(),
                "timestamp point name failed");
        require(collection::saveStandaloneFlangePose(points, "pose_A.v1", later, saved).isEmpty() &&
                saved.endsWith("pose_A.v1.json"), "dotted point name failed");
        const auto pointFiles = QDir(points).entryList(QDir::Files);
        require(!collection::saveStandaloneFlangePose(points, "pose_invalid", nullptr, saved).isEmpty() &&
                saved.isEmpty(), "missing standalone pose was reported as saved");
        require(!collection::saveStandaloneFlangePose(points, "pose_invalid", invalid, saved).isEmpty() &&
                saved.isEmpty(), "non-finite standalone pose was reported as saved");
        require(QDir(points).entryList(QDir::Files) == pointFiles,
                "invalid standalone pose created a JSON file");
        require(!collection::saveStandaloneFlangePose(image, "pose_blocked", pose, saved).isEmpty() &&
                saved.isEmpty(), "unwritable standalone directory did not fail");
        require(read(image) == imageBytes, "standalone disk failure changed existing image");
        std::cout << "Flange pose records, null failures and initial matrix persistence passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
