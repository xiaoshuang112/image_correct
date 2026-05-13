#include "exposure_fusion.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

cv::Mat ExposureFusion::fuse(const std::vector<cv::Mat>& images, const Params& params) {
    if (images.empty()) {
        return cv::Mat();
    }
    
    // 确保所有图像尺寸一致
    cv::Size size = images[0].size();
    for (const auto& img : images) {
        if (img.size() != size || img.type() != CV_8UC3) {
            CV_Error(cv::Error::StsBadArg, "All images must have same size and be CV_8UC3 type");
        }
    }
    
    // 1. 计算权重图
    std::vector<cv::Mat> weights = computeWeights(images, params);
    
    // 2. 归一化权重
    normalizeWeights(weights);
    
    // 3. 构建拉普拉斯金字塔（用于图像）
    std::vector<std::vector<cv::Mat>> laplacianPyramids(images.size());
    for (size_t i = 0; i < images.size(); ++i) {
        cv::Mat floatImage;
        images[i].convertTo(floatImage, CV_32FC3, 1.0 / 255.0);
        laplacianPyramids[i] = buildLaplacianPyramid(floatImage, params.levels);
    }
    
    // 4. 构建高斯金字塔（用于权重）
    std::vector<std::vector<cv::Mat>> weightPyramids(weights.size());
    for (size_t i = 0; i < weights.size(); ++i) {
        weightPyramids[i] = buildGaussianPyramid(weights[i], params.levels);
    }
    
    // 5. 逐层融合
    std::vector<cv::Mat> fusedPyramid(params.levels);
    for (int level = 0; level < params.levels; ++level) {
        cv::Mat fusedLevel;
        
        for (size_t i = 0; i < images.size(); ++i) {
            // 对每个颜色通道分别处理
            std::vector<cv::Mat> channels(3);
            cv::split(laplacianPyramids[i][level], channels);
            
            cv::Mat weight = weightPyramids[i][level];
            
            for (int c = 0; c < 3; ++c) {
                cv::Mat weighted = channels[c].mul(weight);
                if (i == 0) {
                    channels[c] = weighted;
                } else {
                    channels[c] += weighted;
                }
            }
            
            if (i == images.size() - 1) {
                cv::merge(channels, fusedLevel);
            }
        }
        
        fusedPyramid[level] = fusedLevel;
    }
    
    // 6. 从金字塔重建图像
    cv::Mat result = reconstructFromLaplacianPyramid(fusedPyramid);
    
    // 7. 转换回 8UC3
    cv::Mat result8u;
    result.convertTo(result8u, CV_8UC3, 255.0);
    
    return result8u;
}
cv::Mat ExposureFusion::fuseFast(const std::vector<cv::Mat>& images) {
    if (images.empty()) {
        std::cerr << "No images provided for fusion" << std::endl;
        return cv::Mat();
    }
    
    cv::Size size = images[0].size();
    
    // 创建累积器
    cv::Mat accumulated = cv::Mat::zeros(size, CV_32FC3);
    cv::Mat weightAccum = cv::Mat::zeros(size, CV_32F);
    
    for (const auto& img : images) {
        // 转换到浮点
        cv::Mat floatImg;
        img.convertTo(floatImg, CV_32FC3, 1.0 / 255.0);
        
        // 计算灰度权重
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::Mat weight;
        gray.convertTo(weight, CV_32F, 1.0 / 255.0);
        weight = 1.0f - cv::abs(weight - 0.5f) * 2.0f;
        weight = cv::max(weight, 0.01f);
        
        // 逐通道加权累加（避免通道数不匹配问题）
        std::vector<cv::Mat> accChannels(3);
        cv::split(accumulated, accChannels);
        
        std::vector<cv::Mat> imgChannels(3);
        cv::split(floatImg, imgChannels);
        
        for (int c = 0; c < 3; ++c) {
            accChannels[c] += imgChannels[c].mul(weight);
        }
        
        cv::merge(accChannels, accumulated);
        weightAccum += weight;
    }
    
    // 归一化 - 对每个通道分别处理
    std::vector<cv::Mat> resultChannels(3);
    cv::split(accumulated, resultChannels);
    
    for (int c = 0; c < 3; ++c) {
        // 逐元素除法，避免除零
        cv::divide(resultChannels[c], weightAccum, resultChannels[c]);
    }
    
    cv::Mat result;
    cv::merge(resultChannels, result);
    
    // 转换回8位
    cv::Mat result8u;
    result.convertTo(result8u, CV_8UC3, 255.0);
    
    return result8u;
}

std::vector<cv::Mat> ExposureFusion::computeWeights(const std::vector<cv::Mat>& images, const Params& params) {
    std::vector<cv::Mat> weights(images.size());
    
    for (size_t i = 0; i < images.size(); ++i) {
        cv::Mat contrast = computeContrastWeight(images[i]);
        cv::Mat saturation = computeSaturationWeight(images[i]);
        cv::Mat wellExposed = computeWellExposedWeight(images[i], params.sigma);
        
        // 加权组合
        weights[i] = params.contrastWeight * contrast + 
                     params.saturationWeight * saturation + 
                     params.wellExposedWeight * wellExposed;
        
        // 添加小值防止除零
        weights[i] = weights[i] + 1e-6f;
    }
    
    return weights;
}

cv::Mat ExposureFusion::computeContrastWeight(const cv::Mat& image) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_32F, 3);
    
    return cv::abs(laplacian);
}

cv::Mat ExposureFusion::computeSaturationWeight(const cv::Mat& image) {
    cv::Mat floatImg;
    image.convertTo(floatImg, CV_32FC3, 1.0 / 255.0);
    
    std::vector<cv::Mat> channels(3);
    cv::split(floatImg, channels);
    
    // 饱和度 = 标准差 of (R,G,B)
    cv::Mat mean = (channels[0] + channels[1] + channels[2]) / 3.0f;
    cv::Mat variance = (channels[0] - mean).mul(channels[0] - mean) +
                       (channels[1] - mean).mul(channels[1] - mean) +
                       (channels[2] - mean).mul(channels[2] - mean);
    
    cv::Mat saturation;
    cv::sqrt(variance / 3.0f, saturation);
    
    return saturation;
}

cv::Mat ExposureFusion::computeWellExposedWeight(const cv::Mat& image, float sigma) {
    cv::Mat floatImg;
    image.convertTo(floatImg, CV_32FC3, 1.0 / 255.0);
    
    std::vector<cv::Mat> channels(3);
    cv::split(floatImg, channels);
    
    cv::Mat weight = cv::Mat::ones(image.size(), CV_32F);
    
    for (int c = 0; c < 3; ++c) {
        // 高斯权重：像素值越接近0.5权重越高
        cv::Mat expTerm = (channels[c] - 0.5f).mul(channels[c] - 0.5f);
        expTerm = -expTerm / (2.0f * sigma * sigma);
        cv::Mat channelWeight;
        cv::exp(expTerm, channelWeight);
        weight = weight.mul(channelWeight);
    }
    
    return weight;
}

std::vector<cv::Mat> ExposureFusion::buildGaussianPyramid(const cv::Mat& image, int levels) {
    std::vector<cv::Mat> pyramid;
    pyramid.push_back(image);
    
    for (int i = 1; i < levels; ++i) {
        cv::Mat down;
        cv::pyrDown(pyramid[i - 1], down);
        pyramid.push_back(down);
    }
    
    return pyramid;
}

std::vector<cv::Mat> ExposureFusion::buildLaplacianPyramid(const cv::Mat& image, int levels) {
    std::vector<cv::Mat> pyramid;
    cv::Mat current = image;
    
    for (int i = 0; i < levels - 1; ++i) {
        cv::Mat down, up;
        cv::pyrDown(current, down);
        cv::pyrUp(down, up, current.size());
        
        cv::Mat laplacian = current - up;
        pyramid.push_back(laplacian);
        current = down;
    }
    
    pyramid.push_back(current);  // 顶层是高斯残差
    return pyramid;
}

cv::Mat ExposureFusion::reconstructFromLaplacianPyramid(const std::vector<cv::Mat>& pyramid) {
    cv::Mat result = pyramid.back();
    
    for (int i = pyramid.size() - 2; i >= 0; --i) {
        cv::Mat up;
        cv::pyrUp(result, up, pyramid[i].size());
        result = up + pyramid[i];
    }
    
    return result;
}

void ExposureFusion::normalizeWeights(std::vector<cv::Mat>& weights) {
    if (weights.empty()) return;
    
    cv::Mat weightSum = cv::Mat::zeros(weights[0].size(), CV_32F);
    for (const auto& w : weights) {
        weightSum += w;
    }
    
    for (auto& w : weights) {
        cv::divide(w, weightSum, w);
    }
}