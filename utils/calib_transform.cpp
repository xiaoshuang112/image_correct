#include "calib_transform.hpp"

namespace calib_transform {

cv::Mat axisAnglePoseToTransform(const double pose[6]) {
    double x = pose[0] / 1000.0;
    double y = pose[1] / 1000.0;
    double z = pose[2] / 1000.0;
    double rx = pose[3] * CV_PI / 180.0;
    double ry = pose[4] * CV_PI / 180.0;
    double rz = pose[5] * CV_PI / 180.0;
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx, ry, rz);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << x, y, z);
    return rvecTvecToTransform(rvec, tvec);
}

cv::Mat rvecTvecToTransform(const cv::Mat& rvec, const cv::Mat& tvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    T.at<double>(0, 3) = tvec.at<double>(0);
    T.at<double>(1, 3) = tvec.at<double>(1);
    T.at<double>(2, 3) = tvec.at<double>(2);
    return T;
}

void transformToAxisAnglePose(const cv::Mat& T, double pose[6]) {
    pose[0] = T.at<double>(0, 3) * 1000.0;
    pose[1] = T.at<double>(1, 3) * 1000.0;
    pose[2] = T.at<double>(2, 3) * 1000.0;
    cv::Mat R = T(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    pose[3] = rvec.at<double>(0) * 180.0 / CV_PI;
    pose[4] = rvec.at<double>(1) * 180.0 / CV_PI;
    pose[5] = rvec.at<double>(2) * 180.0 / CV_PI;
}

cv::Mat invertTransform(const cv::Mat& T) {
    cv::Mat R = T(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat t = T(cv::Rect(3, 0, 1, 3)).clone();
    cv::Mat invT = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat R_inv = R.t();
    R_inv.copyTo(invT(cv::Rect(0, 0, 3, 3)));
    cv::Mat tinv = -R_inv * t;
    tinv.copyTo(invT(cv::Rect(3, 0, 1, 3)));
    return invT;
}

} // namespace calib_transform
