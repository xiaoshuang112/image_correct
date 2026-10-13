#include "collection_utils.hpp"
#include "calib_transform.hpp"
#include <opencv2/photo.hpp>
#include <cmath>
#include <stdexcept>

namespace collection {

void validateTransform(const cv::Mat& t) {
    if (t.rows != 4 || t.cols != 4 || t.type() != CV_64FC1 || !cv::checkRange(t))
        throw std::runtime_error("手眼矩阵必须为有限数值的 4×4 矩阵");
    const cv::Mat r = t(cv::Rect(0, 0, 3, 3));
    if (cv::norm(r.t() * r - cv::Mat::eye(3, 3, CV_64F)) > 1e-3 ||
        std::abs(cv::determinant(r) - 1.0) > 1e-3 ||
        cv::norm(t.row(3) - (cv::Mat_<double>(1, 4) << 0, 0, 0, 1)) > 1e-6)
        throw std::runtime_error("手眼矩阵的旋转或齐次行无效");
}

cv::Mat downwardFlangeTarget(const cv::Mat& baseFlange,
                             const cv::Mat& flangeCamera) {
    validateTransform(baseFlange);
    validateTransform(flangeCamera);
    cv::Mat camera = baseFlange * flangeCamera;
    cv::Vec3d x(camera.at<double>(0, 0), camera.at<double>(1, 0), 0);
    if (cv::norm(x) < 1e-6) {
        // When camera X is vertical, use Y to choose the nearest heading.
        x = cv::Vec3d(-camera.at<double>(1, 1), camera.at<double>(0, 1), 0);
    }
    x /= cv::norm(x);
    const cv::Vec3d z(0, 0, -1);
    const cv::Vec3d y = z.cross(x);
    for (int row = 0; row < 3; ++row) {
        camera.at<double>(row, 0) = x[row];
        camera.at<double>(row, 1) = y[row];
        camera.at<double>(row, 2) = z[row];
    }
    return camera * calib_transform::invertTransform(flangeCamera);
}

cv::Mat fuseMertens(const std::vector<cv::Mat>& frames) {
    if (frames.size() < 2)
        throw std::runtime_error("多曝光融合至少需要两张图像");
    for (const auto& frame : frames) {
        if (frame.empty() || frame.type() != CV_8UC3 || frame.size() != frames.front().size())
            throw std::runtime_error("曝光图像必须是相同尺寸的 BGR8 图像");
    }
    // Match det-xialiao-MVS-nano MultiExposureFusion::method_mertens.
    cv::Mat merged, result;
    cv::createMergeMertens()->process(frames, merged);
    if (!cv::checkRange(merged))
        throw std::runtime_error("曝光融合产生无效像素");
    merged.convertTo(result, CV_8UC3, 255.0);
    return result;
}

} // namespace collection
