#include "collection_undistortion.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>

namespace collection {

UndistortionCalibration UndistortionCalibration::load(const std::string& path) {
    try {
        cv::FileStorage file(path, cv::FileStorage::READ);
        if (!file.isOpened()) throw std::runtime_error("无法读取手眼标定文件");
        UndistortionCalibration result;
        file["camera_matrix"] >> result.cameraMatrix;
        auto coefficients = file["distortion_coefficients"];
        if (coefficients.empty()) coefficients = file["dist_coeffs"];
        coefficients >> result.distortion;
        if (result.cameraMatrix.empty() || result.distortion.empty())
            throw std::runtime_error("手眼文件缺少相机内参或畸变系数，无法去畸变");
        if (result.cameraMatrix.size() != cv::Size(3, 3) || result.cameraMatrix.channels() != 1)
            throw std::runtime_error("相机内参须为 3×3 单通道矩阵");
        const auto count = result.distortion.total();
        if (result.distortion.channels() != 1 ||
            (result.distortion.rows != 1 && result.distortion.cols != 1) ||
            (count != 4 && count != 5 && count != 8 && count != 12 && count != 14))
            throw std::runtime_error("畸变系数须为含 4、5、8、12 或 14 个数值的向量");
        result.cameraMatrix.convertTo(result.cameraMatrix, CV_64F);
        result.distortion = result.distortion.reshape(1, 1);
        result.distortion.convertTo(result.distortion, CV_64F);
        const auto& k = result.cameraMatrix;
        if (!cv::checkRange(k) || !cv::checkRange(result.distortion) ||
            k.at<double>(0, 0) <= 0 || k.at<double>(1, 1) <= 0 ||
            std::abs(k.at<double>(0, 1)) > 1e-9 || std::abs(k.at<double>(1, 0)) > 1e-9 ||
            std::abs(k.at<double>(2, 0)) > 1e-9 || std::abs(k.at<double>(2, 1)) > 1e-9 ||
            std::abs(k.at<double>(2, 2) - 1.0) > 1e-9)
            throw std::runtime_error("相机内参或畸变系数包含无效数值");
        if (!file["distortion_model"].empty()) {
            std::string model; file["distortion_model"] >> model;
            if (model != "opencv_plumb_bob" && model != "plumb_bob" && model != "rational_polynomial")
                throw std::runtime_error("不支持此畸变模型：" + model);
        }
        const bool hasWidth = !file["image_width"].empty();
        const bool hasHeight = !file["image_height"].empty();
        if (hasWidth != hasHeight) throw std::runtime_error("标定分辨率缺少宽度或高度");
        if (hasWidth) {
            file["image_width"] >> result.calibratedSize.width;
            file["image_height"] >> result.calibratedSize.height;
            if (result.calibratedSize.width <= 0 || result.calibratedSize.height <= 0)
                throw std::runtime_error("标定分辨率无效");
        }
        if (!file["camera_serial"].empty()) file["camera_serial"] >> result.cameraSerial;
        return result;
    } catch (const cv::Exception& error) {
        throw std::runtime_error(std::string("手眼文件中的去畸变参数格式无效：") + error.what());
    }
}

void UndistortionCalibration::validateCamera(cv::Size size, const std::string& serial) const {
    if (size.width <= 0 || size.height <= 0 || size.width >= 32767 || size.height >= 32767)
        throw std::runtime_error("相机图像尺寸无效或超出去畸变范围");
    if (!calibratedSize.empty() && size != calibratedSize)
        throw std::runtime_error("标定分辨率 " + std::to_string(calibratedSize.width) + "×" +
            std::to_string(calibratedSize.height) + " 与相机 " + std::to_string(size.width) +
            "×" + std::to_string(size.height) + " 不匹配");
    if (!cameraSerial.empty() && cameraSerial != serial)
        throw std::runtime_error("标定相机序列号与当前相机不匹配");
}

UndistortionMaps::UndistortionMaps(const UndistortionCalibration& calibration,
                                 cv::Size size, const std::string& serial) {
    calibration.validateCamera(size, serial);
    // Production stores the calibration intrinsics in float camera fields
    // before getOptimalNewCameraMatrix. Reproduce that rounding, then keep
    // the returned K_new at full precision for the remap.
    cv::Mat computation;
    calibration.cameraMatrix.convertTo(computation, CV_32F);
    computation.convertTo(computation, CV_64F);
    cv::Rect validRoi;
    const cv::Mat newK = cv::getOptimalNewCameraMatrix(
        computation, calibration.distortion, size, 1.0, size, &validRoi, false);
    if (newK.size() != cv::Size(3, 3) || !cv::checkRange(newK))
        throw std::runtime_error("去畸变输出内参无效");
    cv::Mat output;
    newK.convertTo(output, CV_64F);
    {
        cv::Mat x, y;
        cv::initUndistortRectifyMap(computation, calibration.distortion,
            cv::Mat(), output, size, CV_32FC1, x, y);
        if (!cv::checkRange(x) || !cv::checkRange(y))
            throw std::runtime_error("去畸变映射包含无效数值");
    }
    cv::initUndistortRectifyMap(computation, calibration.distortion,
        cv::Mat(), output, size, kMapType, m_map1, m_map2);
    if (m_map1.type() != CV_16SC2 || m_map2.type() != CV_16UC1 || m_map1.size() != size)
        throw std::runtime_error("去畸变映射类型或尺寸无效");

    m_geometry.originalCameraMatrix = calibration.cameraMatrix.clone();
    m_geometry.computationCameraMatrix = computation;
    m_geometry.distortion = calibration.distortion.clone();
    m_geometry.outputCameraMatrix = output;
    m_geometry.inputSize = size;
    m_geometry.outputSize = size;
    m_geometry.alpha = 1.0;
    m_geometry.validPixROI = validRoi;
    m_geometry.cropped = false;
    m_geometry.flipHorizontal = false;
    m_geometry.flipVertical = false;
    m_geometry.pixelFrame = kProductionAlpha1PixelFrame;
}

cv::Mat UndistortionMaps::apply(const cv::Mat& image) const {
    if (image.empty() || image.size() != m_geometry.inputSize)
        throw std::runtime_error("图像分辨率与去畸变映射不匹配");
    cv::Mat corrected;
    cv::remap(image, corrected, m_map1, m_map2, kInterpolation, kBorder, cv::Scalar(0, 0, 0));
    if (corrected.size() != m_geometry.outputSize)
        throw std::runtime_error("去畸变输出尺寸与输入不一致");
    return corrected;
}

} // namespace collection
