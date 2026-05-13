#ifndef EXPOSURE_FUSION_HPP
#define EXPOSURE_FUSION_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

/**
 * @brief 多曝光图像融合算法实现
 * 基于 Mertens 曝光融合算法 ( Exposure Fusion )
 * 参考论文: "Exposure Fusion" by Mertens, Kautz, Van Reeth
 */
class ExposureFusion {
public:
    struct Params {
        float contrastWeight = 1.0f;      // 对比度权重系数
        float saturationWeight = 1.0f;    // 饱和度权重系数
        float wellExposedWeight = 1.0f;   // 良好曝光权重系数
        
        float sigma = 0.2f;               // 高斯金字塔的标准差
        int levels = 5;                   // 金字塔层数
        
        // 构造函数
        Params() 
            : contrastWeight(1.0f)
            , saturationWeight(1.0f)
            , wellExposedWeight(1.0f)
            , sigma(0.2f)
            , levels(5) {
        }
        
        // 参数构造函数
        Params(float contrast, float saturation, float wellExposed, float s = 0.2f, int l = 5)
            : contrastWeight(contrast)
            , saturationWeight(saturation)
            , wellExposedWeight(wellExposed)
            , sigma(s)
            , levels(l) {
        }
    };
    
    /**
     * @brief 融合多曝光图像序列
     * @param images 输入图像序列 (BGR格式)
     * @param params 融合参数
     * @return 融合后的HDR结果图像
     */
    static cv::Mat fuse(const std::vector<cv::Mat>& images, const Params& params = Params());
    
    /**
     * @brief 快速融合（使用简化权重计算，速度更快）
     */
    static cv::Mat fuseFast(const std::vector<cv::Mat>& images);

private:
    // 计算权重图
    static std::vector<cv::Mat> computeWeights(const std::vector<cv::Mat>& images, const Params& params);
    
    // 计算对比度权重（使用拉普拉斯滤波器）
    static cv::Mat computeContrastWeight(const cv::Mat& image);
    
    // 计算饱和度权重（像素值与灰度值的标准差）
    static cv::Mat computeSaturationWeight(const cv::Mat& image);
    
    // 计算良好曝光权重（使用高斯曲线）
    static cv::Mat computeWellExposedWeight(const cv::Mat& image, float sigma = 0.2f);
    
    // 构建高斯金字塔
    static std::vector<cv::Mat> buildGaussianPyramid(const cv::Mat& image, int levels);
    
    // 构建拉普拉斯金字塔
    static std::vector<cv::Mat> buildLaplacianPyramid(const cv::Mat& image, int levels);
    
    // 重建图像（从拉普拉斯金字塔）
    static cv::Mat reconstructFromLaplacianPyramid(const std::vector<cv::Mat>& pyramid);
    
    // 归一化权重图
    static void normalizeWeights(std::vector<cv::Mat>& weights);
};

#endif // EXPOSURE_FUSION_HPP