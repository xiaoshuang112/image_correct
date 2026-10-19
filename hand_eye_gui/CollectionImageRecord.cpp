#include "CollectionImageRecord.hpp"
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>
#include <vector>

namespace collection {
namespace {
std::vector<uchar> encode(const cv::Mat& image) {
    std::vector<uchar> bytes;
    if (!cv::imencode(".png", image, bytes)) throw std::runtime_error("图片编码失败");
    return bytes;
}
void write(const QString& path, const std::vector<uchar>& bytes) {
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) ||
        file.write(reinterpret_cast<const char*>(bytes.data()), bytes.size()) != static_cast<qint64>(bytes.size()) ||
        !file.commit())
        throw std::runtime_error(("图片保存失败：" + file.errorString()).toStdString());
}
}

QString saveCaptureImages(const cv::Mat& raw, const cv::Mat& corrected, const QString& rawPath) {
    if (raw.empty() || raw.type() != CV_8UC3 || !rawPath.endsWith(".png"))
        throw std::runtime_error("融合图像或图片文件名无效");
    if (!corrected.empty() && (corrected.size() != raw.size() || corrected.type() != raw.type()))
        throw std::runtime_error("去畸变图像格式或分辨率无效");
    const QString correctedPath = rawPath.left(rawPath.size() - 4) + "_undistorted.png";
    if (QFileInfo::exists(rawPath) || (!corrected.empty() && QFileInfo::exists(correctedPath)))
        throw std::runtime_error("图片文件已存在，本次不覆盖");
    const auto rawBytes = encode(raw);
    const auto correctedBytes = corrected.empty() ? std::vector<uchar>{} : encode(corrected);
    write(rawPath, rawBytes);
    if (corrected.empty()) return rawPath;
    try {
        write(correctedPath, correctedBytes);
    } catch (...) {
        QFile::remove(rawPath);
        throw;
    }
    return correctedPath;
}

QString saveUndistortionMetadata(const QString& correctedImagePath,
                                 const UndistortionGeometry& geometry) {
    const QFileInfo image(correctedImagePath);
    if (image.suffix() != QLatin1String("png") || geometry.outputCameraMatrix.empty())
        return QStringLiteral("去畸变坐标记录无效，图片已保存");
    const QString path = image.dir().filePath(image.completeBaseName() + ".pixel_frame.json");
    if (QFileInfo::exists(path))
        return QStringLiteral("去畸变坐标记录已存在，本次不覆盖");
    const auto matrix = [](const cv::Mat& source) {
        cv::Mat values;
        source.convertTo(values, CV_64F);
        QJsonArray rows;
        for (int row = 0; row < values.rows; ++row) {
            QJsonArray line;
            for (int col = 0; col < values.cols; ++col)
                line.append(values.at<double>(row, col));
            rows.append(line);
        }
        return rows;
    };
    const auto flat = [](const cv::Mat& source) {
        cv::Mat values;
        source.reshape(1, 1).convertTo(values, CV_64F);
        QJsonArray line;
        for (int col = 0; col < values.cols; ++col)
            line.append(values.at<double>(0, col));
        return line;
    };
    const QJsonObject record{
        {"pixel_frame", QString::fromLatin1(geometry.pixelFrame)},
        {"original_camera_matrix", matrix(geometry.originalCameraMatrix)},
        {"computation_camera_matrix", matrix(geometry.computationCameraMatrix)},
        {"distortion_coefficients", flat(geometry.distortion)},
        {"output_camera_matrix", matrix(geometry.outputCameraMatrix)},
        {"input_size_wh", QJsonArray{geometry.inputSize.width, geometry.inputSize.height}},
        {"output_size_wh", QJsonArray{geometry.outputSize.width, geometry.outputSize.height}},
        {"alpha", geometry.alpha},
        {"valid_pix_roi_xywh", QJsonArray{geometry.validPixROI.x, geometry.validPixROI.y,
                                          geometry.validPixROI.width, geometry.validPixROI.height}},
        {"cropped", geometry.cropped},
        {"flip_horizontal", geometry.flipHorizontal},
        {"flip_vertical", geometry.flipVertical},
    };
    const QByteArray bytes = QJsonDocument(record).toJson(QJsonDocument::Indented);
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
        return QStringLiteral("去畸变坐标记录保存失败：") + file.errorString();
    return {};
}

} // namespace collection
