#ifndef PUTTEXT_H_
#define PUTTEXT_H_

#ifdef _WIN32

#else
 
#endif


#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>



void putTextZH(cv::Mat& dst, const char* str, cv::Point org, cv::Scalar color, int fontSize, int font_type=0, const char* fn = "微软雅黑", bool italic = false, bool underline = false);
void putTextZH_w(cv::Mat& dst, const wchar_t* str_w, cv::Point org, cv::Scalar color, int fontSize, const char* fn = "微软雅黑", bool italic = false, bool underline = false);

#endif // PUTTEXT_H_
