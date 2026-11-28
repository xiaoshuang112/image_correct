#pragma once

#include <optional>
#include <string_view>

namespace collection {

enum class PixelProjection { Alpha1, SquarePixels, Invalid };

inline constexpr char kAlpha1Projection[] = "opencv_alpha1_v1";
inline constexpr char kSquarePixelsProjection[] = "square_pixels_v1";

inline PixelProjection parsePixelProjection(std::string_view policy) {
    if (policy == kAlpha1Projection) return PixelProjection::Alpha1;
    if (policy == kSquarePixelsProjection) return PixelProjection::SquarePixels;
    return PixelProjection::Invalid;
}

inline const char* pixelProjectionPolicy(PixelProjection projection) {
    switch (projection) {
    case PixelProjection::Alpha1: return kAlpha1Projection;
    case PixelProjection::SquarePixels: return kSquarePixelsProjection;
    case PixelProjection::Invalid: return "";
    }
    return "";
}

struct CameraOrientation {
    std::optional<bool> reverseX;
    std::optional<bool> reverseY;
};

} // namespace collection
