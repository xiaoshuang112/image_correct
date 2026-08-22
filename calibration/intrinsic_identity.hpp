#ifndef INTRINSIC_IDENTITY_HPP
#define INTRINSIC_IDENTITY_HPP

#include <opencv2/core.hpp>

#include <string>

// MVS 内参安全门禁（纯 C++ + OpenCV，无 Qt/MVS 依赖，可无硬件单测）。
//
// MVS SDK 不等于镜头标定数据库：SDK 不提供内参，内参必须来自外部
// camera_params.yaml，且必须与当前相机身份（序列号/实际分辨率/ROI）
// 绑定。身份不匹配或身份未绑定（且未显式开启兼容开关）时，阻断标定。
namespace intrinsic_gate {

// camera_params.yaml 中可选的身份绑定字段
struct IntrinsicIdentity {
    std::string serial;        // camera_serial
    int width = 0;             // image_width（标定内参时的图像宽度）
    int height = 0;            // image_height
    std::string pixel_format;  // pixel_format（可选）
    // ROI / binning（可选）：全 0 表示未声明
    int roi_x = 0;
    int roi_y = 0;
    int roi_width = 0;
    int roi_height = 0;
    int binning = 0;

    bool hasSerial() const { return !serial.empty(); }
    bool hasResolution() const { return width > 0 && height > 0; }
    bool hasAny() const { return hasSerial() || hasResolution(); }
};

// 校验 K/D 的形状、有限性与焦距合理性。
// imageW/imageH 为实际流分辨率（>0 时用于 cx/cy/焦距合理性检查）。
bool validateIntrinsicsShape(const cv::Mat& cameraMatrix,
                             const cv::Mat& distCoeffs,
                             int imageW,
                             int imageH,
                             std::string& reason);

// 加载 camera_params.yaml（K/D 必填，身份字段可选）。
bool loadCameraParamsWithIdentity(const std::string& path,
                                  cv::Mat& cameraMatrix,
                                  cv::Mat& distCoeffs,
                                  IntrinsicIdentity& identity,
                                  std::string& error);

// 身份绑定校验。
//   - 内参声明了 serial：必须与设备序列号精确匹配；
//   - 内参声明了分辨率：必须与设备实际流分辨率一致；
//   - 内参无任何身份字段（旧版仅 K/D）：allowLegacy=false 时拒绝，
//     allowLegacy=true 时给出警告并放行（reason 带回警告内容）。
bool checkIntrinsicBinding(const IntrinsicIdentity& identity,
                           const std::string& deviceSerial,
                           int actualWidth,
                           int actualHeight,
                           bool allowLegacy,
                           std::string& reason);

}  // namespace intrinsic_gate

#endif  // INTRINSIC_IDENTITY_HPP
