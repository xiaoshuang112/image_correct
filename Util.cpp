#ifdef _WIN32
#include <Windows.h>
#else
#include "Texts.h" 
#endif


#include "Util.h"

#include <fstream>
#include <iostream>

using namespace std;

std::string Util::GBKToUTF8(const char* src_str) {
    int len = 0;

#ifdef _WIN32
    len = MultiByteToWideChar(CP_ACP, 0, src_str, -1, NULL, 0);
    wchar_t* wstr = new wchar_t[len + 1];
    memset(wstr, 0, len + 1);
    MultiByteToWideChar(CP_ACP, 0, src_str, -1, wstr, len);
    len = WideCharToMultiByte(CP_UTF8, 0, wstr, -1, NULL, 0, NULL, NULL);
    char* str = new char[len + 1];
    memset(str, 0, len + 1);
    WideCharToMultiByte(CP_UTF8, 0, wstr, -1, str, len, NULL, NULL);
    std::string strTemp = str;
    if (wstr)
        delete[] wstr;
    if (str)
        delete[] str;
#else
    std::string str = src_str;
    wstring strText = util::TextOps::fromNative(str);
    std::string strTemp = util::TextOps::toUtf8(strText);
#endif

    return strTemp;
}

std::string Util::UTF8ToGBK(const char* src_str) {

#ifdef _WIN32
    int len = 0;
    len = MultiByteToWideChar(CP_UTF8, 0, src_str, -1, NULL, 0);
    wchar_t* wszGBK = new wchar_t[len + 1];
    memset(wszGBK, 0, len * 2 + 2);
    MultiByteToWideChar(CP_UTF8, 0, src_str, -1, wszGBK, len);
    len = WideCharToMultiByte(CP_ACP, 0, wszGBK, -1, NULL, 0, NULL, NULL);
    char* szGBK = new char[len + 1];
    memset(szGBK, 0, len + 1);
    WideCharToMultiByte(CP_ACP, 0, wszGBK, -1, szGBK, len, NULL, NULL);
    std::string strTemp(szGBK);
    if (wszGBK)
        delete[] wszGBK;
    if (szGBK)
        delete[] szGBK;
#else
    std::string str = src_str;
    wstring strText = util::TextOps::fromUtf8(str);
    std::string strTemp = util::TextOps::toNative(strText);
#endif

    return strTemp;
}

std::string Util::UnicodeToUTF8(const std::wstring& str) {
#ifdef _WIN32
     char* pElementText;
     int iTextLen;
     // wide char to multi char
     iTextLen = WideCharToMultiByte(CP_UTF8, 0, str.c_str(), -1, NULL, 0, NULL, NULL);
     pElementText = new char[iTextLen + 1];
     memset((void*)pElementText, 0, sizeof(char) * (iTextLen + 1));
     ::WideCharToMultiByte(CP_UTF8, 0, str.c_str(), -1, pElementText, iTextLen, NULL, NULL);
     string strText;
     strText = pElementText;
     delete[] pElementText;
#else
    string strText = util::TextOps::toUtf8(str);
#endif

    return strText;
}

std::wstring Util::UTF8ToUnicode(const std::string& str) {
#ifdef _WIN32
    wchar_t* pElementText;
    int iTextLen;
    // wide char to multi char
    iTextLen = MultiByteToWideChar(CP_UTF8, 0, str.c_str(), str.size(), NULL, NULL);
    pElementText = new wchar_t[iTextLen + 1];
    memset((void*)pElementText, 0, sizeof(wchar_t) * (iTextLen + 1));
    ::MultiByteToWideChar(CP_UTF8, 0, str.c_str(), str.size(), pElementText, iTextLen);
    wstring strText;
    strText = pElementText;
    delete[] pElementText;
#else
    wstring strText = util::TextOps::fromUtf8(str);
#endif

    return strText;
}

std::string Util::UnicodeToANSI(const std::wstring& str) {
#ifdef _WIN32
     char* pElementText;
     int iTextLen;
     // wide char to multi char
     iTextLen = WideCharToMultiByte(CP_ACP, 0, str.c_str(), -1, NULL, 0, NULL, NULL);
     pElementText = new char[iTextLen + 1];
     memset((void*)pElementText, 0, sizeof(char) * (iTextLen + 1));
     ::WideCharToMultiByte(CP_ACP, 0, str.c_str(), -1, pElementText, iTextLen, NULL, NULL);
     string strText;
     strText = pElementText;
     delete[] pElementText;
#else
    string strText = util::TextOps::toNative(str);
#endif

    return strText;
}
std::wstring Util::GBKToUNICODE(const std::string& str) {
    wstring strTo;

#ifdef _WIN32
     int iTextLen;
     // wide char to multi char
     iTextLen = MultiByteToWideChar(CP_ACP, 0, str.c_str(), str.size(), NULL, NULL);
     if (iTextLen > 0) {
         strTo.resize(iTextLen);
         ::MultiByteToWideChar(CP_ACP, 0, str.c_str(), str.size(), (wchar_t*)strTo.c_str(), strTo.size());
     }
#else

#endif

    

    return strTo;
}

cv::Mat Util::resizeKeepAspectRatio(const cv::Mat& input, const cv::Size& dstSize, int img_pos /* 0 top  1 center 2 bottom */, cv::Scalar fill_color, int interpolation) {
    cv::Mat output;

    if (-2 == img_pos) {
        cv::resize(input, output, cv::Size(dstSize.width, dstSize.height), 0, 0, interpolation);
        return output;
    }

    double h1 = dstSize.width * (input.rows / (double)input.cols);
    double w2 = dstSize.height * (input.cols / (double)input.rows);
    if (h1 <= dstSize.height) {
        cv::resize(input, output, cv::Size(dstSize.width, h1), 0, 0, interpolation);
    } else {
        cv::resize(input, output, cv::Size(w2, dstSize.height), 0, 0, interpolation);
    }

    if (-1 == img_pos)
        return output;

    int top = 0;
    int down = 0;
    int left = 0;
    int right = 0;

    if (0 == img_pos) // top
    {
        top = 0;
        down = dstSize.height - output.rows;
        left = (dstSize.width - output.cols) / 2;
        right = (dstSize.width - output.cols + 1) / 2;
    } else if (1 == img_pos) // center
    {
        top = (dstSize.height - output.rows) / 2;
        down = (dstSize.height - output.rows + 1) / 2;
        left = (dstSize.width - output.cols) / 2;
        right = (dstSize.width - output.cols + 1) / 2;
    } else if (2 == img_pos) // bottom
    {
        top = (dstSize.height - output.rows);
        down = 0;
        left = (dstSize.width - output.cols) / 2;
        right = (dstSize.width - output.cols + 1) / 2;
    } else if (3 == img_pos) // left
    {
        top = 0;
        down = dstSize.height - output.rows;
        left = 0;
        right = dstSize.width - output.cols;
    }

    cv::copyMakeBorder(output, output, top, down, left, right, cv::BORDER_CONSTANT, fill_color);

    return output;
}

bool Util::exists_img(std::string path) {
    std::ifstream ifile(path);
    if (!ifile.is_open()) {
        printf("%s does not exist!\n", path.c_str());
        return false;
    }
    ifile.close();
    return true;
}

std::string Util::get_time() {
#ifdef _WIN32
     SYSTEMTIME sys;
     GetLocalTime(&sys);
     char time[30] = {0};
     sprintf_s(time, "[%4d-%02d-%02d %02d:%02d:%02d.%03d]", sys.wYear, sys.wMonth, sys.wDay, sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds);
     return std::string(time);
#else
    return "";
#endif


    
}

float Util::IOU_cv(const cv::Rect& r1, const cv::Rect& r2) {
    cv::Rect and_r = r1 | r2;
    cv::Rect U = r1 & r2;

    return U.area() * 1.0f / and_r.area();
}

// 计算重叠率，r_num 决定分母是 r1 还是 r2
float Util::intersect_area_ratio(const cv::Rect& r1, const cv::Rect& r2, int r_num) {
    cv::Rect and_r = r1 & r2;

    if (1 == r_num)
        return and_r.area() * 1.0f / r1.area();
    else if (2 == r_num)
        return and_r.area() * 1.0f / r2.area();
    else
        return 0;
}

int Util::pnpoly(int nvert, float* vertx, float* verty, float testx, float testy) {
    int i, j, c = 0;
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((verty[i] > testy) != (verty[j] > testy)) && (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
            c = !c;
    }
    return c;
}

time_t Util::StringToTimeStamp(std::string str) {
    struct tm tm_;
    int year, month, day, hour, minute, second;
    sscanf(str.c_str(), "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second);
    tm_.tm_year = year - 1900;
    tm_.tm_mon = month - 1;
    tm_.tm_mday = day;
    tm_.tm_hour = hour;
    tm_.tm_min = minute;
    tm_.tm_sec = second;
    tm_.tm_isdst = 0;

    time_t timeStamp = mktime(&tm_);
    return timeStamp;
}

tm Util::getFileTime(const std::string& path) {
    tm tm_;
#ifdef _WIN32
    int ret;
    struct stat st;
    ret = stat(path.c_str(), &st);

    if (-1 == ret) {
        std::cout << "get file time failed" << std::endl;
    }

    time_t t = st.st_mtime;
    localtime_s(&tm_, &t);
#else

#endif


    return tm_;
}

double Util::distance_to_Line(cv::Point line_start, cv::Point line_end, cv::Point point) {
    cv::Point line; // direction of the line
    line.x = line_end.x - line_start.x;
    line.y = line_end.y - line_start.y;
    cv::Point point_to_start; // vector from the point to the start of the line
    point_to_start.x = line_start.x - point.x;
    point_to_start.y = line_start.y - point.y;
    int dot = point_to_start.x * line.x + point_to_start.y * line.y; // dot product of point_to_start * line
    cv::Point distance; // shortest distance vector from point to line
    distance.x = point_to_start.x - dot * line.x;
    distance.y = point_to_start.y - dot * line.y;
    double output = sqrt((double)(distance.x * distance.x + distance.y * distance.y));
    return output;
}
