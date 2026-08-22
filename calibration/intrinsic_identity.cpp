#include "intrinsic_identity.hpp"

#include <cmath>

namespace intrinsic_gate {

bool validateIntrinsicsShape(const cv::Mat& cameraMatrix,
                             const cv::Mat& distCoeffs,
                             int imageW,
                             int imageH,
                             std::string& reason) {
    if (cameraMatrix.empty()) {
        reason = "camera_matrix 为空";
        return false;
    }
    if (cameraMatrix.rows != 3 || cameraMatrix.cols != 3) {
        reason = "camera_matrix 必须是 3x3，实际 " +
                 std::to_string(cameraMatrix.rows) + "x" +
                 std::to_string(cameraMatrix.cols);
        return false;
    }
    cv::Mat K;
    cameraMatrix.convertTo(K, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            if (!std::isfinite(K.at<double>(r, c))) {
                reason = "camera_matrix 含非有限值";
                return false;
            }
    const double fx = K.at<double>(0, 0);
    const double fy = K.at<double>(1, 1);
    const double cx = K.at<double>(0, 2);
    const double cy = K.at<double>(1, 2);
    if (fx <= 0.0 || fy <= 0.0) {
        reason = "焦距必须为正: fx=" + std::to_string(fx) +
                 " fy=" + std::to_string(fy);
        return false;
    }
    if (imageW > 0 && imageH > 0) {
        // 焦距合理性：工业相机 fx/fy 一般在 0.1~50 倍图像宽度之间
        const double fxLimit = 50.0 * imageW;
        if (fx > fxLimit || fy > fxLimit || fx < 0.05 * imageW ||
            fy < 0.05 * imageH) {
            reason = "焦距相对分辨率不合理: fx=" + std::to_string(fx) +
                     " fy=" + std::to_string(fy) + " image=" +
                     std::to_string(imageW) + "x" + std::to_string(imageH);
            return false;
        }
        // 主点允许适度偏出画面（ROI/裁剪），但不能离谱
        if (cx < -imageW || cx > 2.0 * imageW ||
            cy < -imageH || cy > 2.0 * imageH) {
            reason = "主点超出合理范围: cx=" + std::to_string(cx) +
                     " cy=" + std::to_string(cy) + " image=" +
                     std::to_string(imageW) + "x" + std::to_string(imageH);
            return false;
        }
    }

    if (distCoeffs.empty()) {
        reason = "distortion_coefficients 为空";
        return false;
    }
    const int n = static_cast<int>(distCoeffs.total());
    if (n != 4 && n != 5 && n != 8 && n != 12 && n != 14) {
        reason = "distortion_coefficients 长度必须是 4/5/8/12/14，实际 " +
                 std::to_string(n);
        return false;
    }
    cv::Mat D;
    distCoeffs.convertTo(D, CV_64F);
    for (int i = 0; i < n; ++i)
        if (!std::isfinite(D.at<double>(i))) {
            reason = "distortion_coefficients 含非有限值";
            return false;
        }
    return true;
}

bool loadCameraParamsWithIdentity(const std::string& path,
                                  cv::Mat& cameraMatrix,
                                  cv::Mat& distCoeffs,
                                  IntrinsicIdentity& identity,
                                  std::string& error) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        error = "无法打开内参文件: " + path;
        return false;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    if (cameraMatrix.empty() || distCoeffs.empty()) {
        error = "内参文件缺少 camera_matrix 或 distortion_coefficients: " + path;
        return false;
    }

    identity = IntrinsicIdentity{};
    if (!fs["camera_serial"].empty())
        fs["camera_serial"] >> identity.serial;
    if (!fs["image_width"].empty())
        fs["image_width"] >> identity.width;
    if (!fs["image_height"].empty())
        fs["image_height"] >> identity.height;
    if (!fs["pixel_format"].empty())
        fs["pixel_format"] >> identity.pixel_format;
    cv::FileNode roi = fs["roi"];
    if (!roi.empty() && roi.isMap()) {
        if (!roi["x"].empty()) roi["x"] >> identity.roi_x;
        if (!roi["y"].empty()) roi["y"] >> identity.roi_y;
        if (!roi["width"].empty()) roi["width"] >> identity.roi_width;
        if (!roi["height"].empty()) roi["height"] >> identity.roi_height;
    }
    if (!fs["binning"].empty())
        fs["binning"] >> identity.binning;
    fs.release();
    return true;
}

bool checkIntrinsicBinding(const IntrinsicIdentity& identity,
                           const std::string& deviceSerial,
                           int actualWidth,
                           int actualHeight,
                           bool allowLegacy,
                           std::string& reason) {
    if (identity.hasSerial()) {
        if (deviceSerial.empty()) {
            reason = "内参绑定了序列号 " + identity.serial +
                     "，但设备未提供序列号";
            return false;
        }
        if (identity.serial != deviceSerial) {
            reason = "内参序列号与相机不匹配: intrinsic_serial=" +
                     identity.serial + " device_serial=" + deviceSerial;
            return false;
        }
    }
    if (identity.hasResolution()) {
        if (actualWidth > 0 && actualHeight > 0 &&
            (identity.width != actualWidth || identity.height != actualHeight)) {
            reason = "内参分辨率与实际流分辨率不匹配: intrinsic=" +
                     std::to_string(identity.width) + "x" +
                     std::to_string(identity.height) + " actual=" +
                     std::to_string(actualWidth) + "x" +
                     std::to_string(actualHeight);
            return false;
        }
    }
    if (!identity.hasAny()) {
        if (!allowLegacy) {
            reason = "内参文件未绑定相机身份（缺少 camera_serial/image_width/"
                     "image_height），且兼容开关 allow_legacy_intrinsics 未开启";
            return false;
        }
        reason = "警告: 内参文件身份未绑定（旧版仅 K/D），"
                 "兼容开关 allow_legacy_intrinsics 已放行";
        return true;
    }
    return true;
}

}  // namespace intrinsic_gate
