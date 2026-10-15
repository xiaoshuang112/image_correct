#include "CollectionPoseRecord.hpp"
#include "utils/calib_transform.hpp"
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLockFile>
#include <QSaveFile>
#include <QStringList>
#include <algorithm>
#include <cmath>

namespace {
QString writeJson(const QString& path, const QJsonObject& object) {
    const QByteArray bytes = QJsonDocument(object).toJson(QJsonDocument::Indented);
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
        return file.errorString();
    return {};
}
}

namespace collection {
QString saveFlangePoseRecords(const QString& imagePath, const double* pose) {
    const bool valid = pose && std::all_of(pose, pose + 6,
        [](double value) { return std::isfinite(value); });
    const QFileInfo image(imagePath);
    const QDir directory = image.dir();
    const char* keys[] = {"x", "y", "z", "rx", "ry", "rz"};
    QJsonObject values;
    for (int i = 0; i < 6; ++i)
        values.insert(keys[i], valid ? QJsonValue(pose[i]) : QJsonValue(QJsonValue::Null));

    QStringList warnings;
    const QString error = writeJson(directory.filePath(image.completeBaseName() + ".json"), values);
    if (!error.isEmpty()) warnings << "法兰位姿文件保存失败：" + error;
    const QString matrixPath = directory.filePath("T_base_flange.json");
    if (valid && !QFileInfo::exists(matrixPath)) {
        // Serialise creation across app instances, then recheck: never replace
        // the directory's initial matrix with a later photograph's pose.
        QLockFile lock(matrixPath + ".lock");
        if (!lock.tryLock(0)) {
            warnings << "初始法兰矩阵文件正在写入，本次未重复写入";
        } else if (!QFileInfo::exists(matrixPath)) {
            const cv::Mat matrix = calib_transform::axisAnglePoseToTransform(pose);
            QJsonArray rows;
            for (int row = 0; row < 4; ++row) {
                QJsonArray values;
                for (int col = 0; col < 4; ++col) values.append(matrix.at<double>(row, col));
                rows.append(values);
            }
            const QString error = writeJson(matrixPath, QJsonObject{{"T_base_flange", rows}});
            if (!error.isEmpty()) warnings << "初始法兰矩阵保存失败：" + error;
        }
    }
    return warnings.join("；");
}
QString saveStandaloneFlangePose(const QString& directoryPath, const QString& base, const double* pose, QString& savedPath) {
    savedPath.clear();
    if (!pose || !std::all_of(pose, pose + 6, [](double v) { return std::isfinite(v); }))
        return "法兰位姿无效，未保存点位";
    QDir directory(directoryPath);
    if (!QDir().mkpath(directoryPath)) return "无法创建点位保存目录";
    QLockFile lock(directory.filePath(".pose-save.lock"));
    if (!lock.tryLock(0)) return "其他点位正在保存，请重试";
    QString path = directory.filePath(base + ".json");
    for (int i = 2; QFileInfo::exists(path); ++i)
        path = directory.filePath(base + "_" + QString::number(i) + ".json");
    // Reuse the per-image writer and its once-only initial matrix creation.
    const QString warnings = saveFlangePoseRecords(path, pose);
    if (QFileInfo::exists(path)) savedPath = path;
    return warnings;
}
} // namespace collection
