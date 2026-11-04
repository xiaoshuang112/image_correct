#include "collection_utils.hpp"
#include <opencv2/photo.hpp>
#include <stdexcept>

namespace collection {

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
