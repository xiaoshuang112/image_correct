#include "hand_eye_gui/CollectionPoseRecord.hpp"
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLockFile>
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

        // TCP records carry the exact tool identity and offset used by the read.
        const QString points = temp.filePath("standalone");
        const QString base = "tcp_探针A_工位A";
        const double offset[] = {10, -20, 120, 0, 0, 90};
        auto save = [&](const double* data, const QString& tool, const double* toolOffset, QString& path) {
            return collection::saveStandaloneTcpPose(points, base, data, tool, toolOffset, path);
        };
        QString saved;
        require(save(pose, "探针A", offset, saved).isEmpty(), "TCP point save failed");
        const QString firstPath = saved;
        const auto firstPoint = read(saved);
        const auto point = object(saved);
        require(point.size() == 8 && point["x"].toDouble() == 500 && point["rz"].toDouble() == 90 &&
                point["tcp_name"].toString() == "探针A", "TCP identity or pose units changed");
        const auto toolOffset = point["tcp_offset"].toArray();
        require(toolOffset.size() == 6, "TCP offset has wrong size");
        for (int i = 0; i < 6; ++i) require(toolOffset[i].toDouble() == offset[i], "TCP offset changed");
        const QString pointMatrix = QDir(points).filePath("T_base_flange.json");
        require(!QFile::exists(pointMatrix), "TCP point created flange matrix");
        require(collection::saveFlangePoseRecords(QDir(points).filePath("capture.png"), later).isEmpty(),
                "image flange record after TCP point failed");
        const auto initialPointMatrix = read(pointMatrix);
        require(object(pointMatrix)["T_base_flange"].toArray()[0].toArray()[3].toDouble() == 1.0,
                "TCP point polluted initial flange matrix");
        require(save(later, "探针A", offset, saved).isEmpty() && saved.endsWith(base + "_2.json"),
                "duplicate TCP point overwrote original");
        require(read(firstPath) == firstPoint && read(pointMatrix) == initialPointMatrix,
                "later TCP save changed prior point or flange matrix");
        require(save(later, "探针A", offset, saved).isEmpty() && saved.endsWith(base + "_3.json"),
                "third duplicate reused occupied name");
        const auto files = QDir(points).entryList(QDir::Files);
        for (const double* data : {static_cast<const double*>(nullptr), static_cast<const double*>(invalid)})
            require(!save(data, "探针A", offset, saved).isEmpty() && saved.isEmpty(), "invalid TCP pose saved");
        require(!save(pose, "", offset, saved).isEmpty() && saved.isEmpty(), "missing TCP name saved");
        require(!save(pose, "探针A", nullptr, saved).isEmpty() && saved.isEmpty(), "missing TCP offset saved");
        require(!save(pose, "探针A", invalid, saved).isEmpty() && saved.isEmpty(), "NaN TCP offset saved");
        invalid[5] = std::numeric_limits<double>::infinity();
        require(!save(invalid, "探针A", offset, saved).isEmpty() && saved.isEmpty(), "infinite TCP pose saved");
        require(!save(pose, "探针A", invalid, saved).isEmpty() && saved.isEmpty(), "infinite TCP offset saved");
        QLockFile lock(QDir(points).filePath(".pose-save.lock"));
        require(lock.tryLock(0), "cannot lock TCP point directory");
        require(!save(pose, "探针A", offset, saved).isEmpty() && saved.isEmpty(), "concurrent writer ignored");
        lock.unlock();
        require(QDir(points).entryList(QDir::Files) == files, "failed TCP save created a file");
        require(!collection::saveStandaloneTcpPose(image, base, pose, "探针A", offset, saved).isEmpty() &&
                saved.isEmpty(), "blocked directory did not fail");

        QString coordinates, tool;
        require(collection::loadPoseCoordinates(firstPath, coordinates, tool).isEmpty() &&
                coordinates == "500,-200,750,0,0,90" && tool == "探针A", "saved TCP JSON did not import in axis order");
        const QString imported = temp.filePath("导入点位.json");
        auto import = [&](const QByteArray& bytes) {
            QFile file(imported);
            require(file.open(QIODevice::WriteOnly), "cannot write import fixture");
            require(file.write(bytes) == bytes.size(), "cannot write complete import fixture");
            file.close();
            return collection::loadPoseCoordinates(imported, coordinates, tool);
        };
        require(import(R"({"rx":179.99967118142848,"ry":-0.012766000975997492,"rz":0.02149207938819223,"x":-122.22233831535885,"y":-517.3887484334697,"z":131.66895532412371})").isEmpty(), "screenshot JSON rejected");
        const auto parts = coordinates.split(',');
        require(parts.size() == 6 && parts[0].toDouble() == -122.22233831535885 &&
                parts[3].toDouble() == 179.99967118142848 && parts[5].toDouble() == 0.02149207938819223,
                "import rounded coordinates or used JSON key order");
        for (const QByteArray& invalidJson : {QByteArray("[]"), QByteArray("{broken"),
                QByteArray(R"({"x":1,"y":2,"z":3,"rx":4,"ry":5})"),
                QByteArray(R"({"x":null,"y":2,"z":3,"rx":4,"ry":5,"rz":6})"),
                QByteArray(R"({"x":"1","y":2,"z":3,"rx":4,"ry":5,"rz":6})"),
                QByteArray(R"({"x":1e999,"y":2,"z":3,"rx":4,"ry":5,"rz":6})"), QByteArray(65537, ' ')})
            require(!import(invalidJson).isEmpty() && coordinates.isEmpty(), "malformed or oversized JSON yielded motion coordinates");
        require(!collection::loadPoseCoordinates(temp.filePath("missing.json"), coordinates, tool).isEmpty(), "missing file imported");
        require(!collection::loadPoseCoordinates(matrixPath, coordinates, tool).isEmpty(), "transform matrix mistaken for six-axis coordinates");
        require(QDir(points).entryList({"*.png", "*.jpg"}, QDir::Files).isEmpty(), "TCP save created an image");
        require(read(image) == imageBytes, "TCP save changed image");
        std::cout << "Flange image records and named TCP metadata passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
