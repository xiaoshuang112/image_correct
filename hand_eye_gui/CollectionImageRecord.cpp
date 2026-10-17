#include "CollectionImageRecord.hpp"
#include <QFile>
#include <QFileInfo>
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

} // namespace collection
