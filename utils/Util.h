#pragma once

#include <ctime> // for std::time_t, std::tm
#include <string>

#include <opencv2/imgproc.hpp> // for cv::Mat, cv::INTER_LINEAR

class Util {
public:
    static std::string GBKToUTF8(const char* src_str);
    static std::string UTF8ToGBK(const char* src_str);

    static std::string UnicodeToUTF8(const std::wstring& str);
    static std::wstring UTF8ToUnicode(const std::string& str);

    static std::string UnicodeToANSI(const std::wstring& str);
    static std::wstring GBKToUNICODE(const std::string& str);

    static cv::Mat resizeKeepAspectRatio(const cv::Mat& input, const cv::Size& dstSize, int img_pos = -1 /* 0 top  1 center 2 bottom */, cv::Scalar fill_color = cv::Scalar(0),
        int interpolation = cv::INTER_LINEAR);

    static bool exists_img(std::string path);
    static std::string get_time();

    static float IOU_cv(const cv::Rect& r1, const cv::Rect& r2);

    // 计算重叠率，r_num 决定分母是 r1 还是 r2
    static float intersect_area_ratio(const cv::Rect& r1, const cv::Rect& r2, int r_num);

    static int pnpoly(int nvert, float* vertx, float* verty, float testx, float testy);

    static std::time_t StringToTimeStamp(std::string str);
    static std::tm getFileTime(const std::string& path);
    static double distance_to_Line(cv::Point line_start, cv::Point line_end, cv::Point point);
};
