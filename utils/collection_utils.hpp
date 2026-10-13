#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace collection {

// Both transforms use metres. Preserve the camera centre and its horizontal
// heading while making camera +Z point down along base -Z.
cv::Mat downwardFlangeTarget(const cv::Mat& baseFlange,
                             const cv::Mat& flangeCamera);
void validateTransform(const cv::Mat& transform);
cv::Mat fuseMertens(const std::vector<cv::Mat>& frames);

} // namespace collection
