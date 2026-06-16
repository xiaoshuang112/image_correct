#pragma once

#include <opencv2/opencv.hpp>

namespace calib_transform {

// axis-angle pose [x,y,z,rx,ry,rz] (mm, deg) → 4x4 homogeneous matrix (m)
cv::Mat axisAnglePoseToTransform(const double pose[6]);

// rvec/tvec → 4x4 transform matrix (m)
cv::Mat rvecTvecToTransform(const cv::Mat& rvec, const cv::Mat& tvec);

// 4x4 → axis-angle pose [x,y,z,rx,ry,rz] (mm, deg)
void transformToAxisAnglePose(const cv::Mat& T, double pose[6]);

// 4x4 inverse of a homogeneous transformation matrix
cv::Mat invertTransform(const cv::Mat& T);

} // namespace calib_transform
