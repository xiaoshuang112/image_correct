#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace collection {

cv::Mat fuseMertens(const std::vector<cv::Mat>& frames);

} // namespace collection
