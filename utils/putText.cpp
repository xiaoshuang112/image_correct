#include "putText.h"
#include <opencv2/opencv.hpp>
using namespace cv;



// 判断字符 是latin语系字符 或 俄文 或 阿拉伯文
static bool is_arabic_char(wchar_t ch)
{
	// 阿拉伯
	if ((ch >= 0x0600 && ch <= 0x06FF) ||          // 基本阿拉伯
		(ch >= 0x0750 && ch <= 0x077F))            // 补充阿拉伯
		return true;

	return false;
}

// 判断 std::wstring 中基本是latin语系字符 或 俄文 或 阿拉伯文
static int is_mostly_arabic(const std::wstring& ws,
	double threshold = 0.40)
{
	if (ws.empty()) return false;

	size_t total = 0;
	size_t valid = 0;

	for (wchar_t ch : ws) {
		if (iswspace(ch) || iswpunct(ch)) continue; // 跳过空格/标点
		++total;
		if (is_arabic_char(ch)
			) ++valid;
	}

	return total != 0 && static_cast<double>(valid) / total >= threshold ? 1:0;
}


#include "opencv2/freetype.hpp" //Note1:头文件
#include <thread>
#include <chrono>
#include <mutex>
#include "Util.h"
#include <iostream>
#include <filesystem>

extern std::string g_appPath;

// static cv::Ptr<cv::freetype::FreeType2> ft2;
std::map<int,std::string> font_path_map = 
{ 
    {0,"fonts/NotoSansCJK-Regular.ttc"},
    {1,"fonts/opentype/noto/NotoSansArabic-Regular.ttf"}
};

// static std::shared_ptr<cv::String> fontDataPath = std::make_shared<cv::String>("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc");
// static std::shared_ptr<cv::String> fontData_arabic = std::make_shared<cv::String>("/usr/share/fonts/truetype/noto/NotoSansArabic-Regular.ttf");

class FontEnginePool {
public:
    static std::shared_ptr<FontEnginePool> getInstance(int font_type) {

        static std::map<int,std::shared_ptr<FontEnginePool>> instance_map;
        if (instance_map.size()==0)
        {

            font_path_map[0] = g_appPath + "fonts/NotoSansCJK-Regular.ttc";
            if (!std::filesystem::exists(font_path_map[0]))
            {
                font_path_map[0] = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc";

                if (!std::filesystem::exists(font_path_map[0]))
                {
                    font_path_map[0] = "/usr/share/fonts/google-noto-cjk/NotoSansCJK-Regular.ttc";
                }
            }


            font_path_map[1] = g_appPath + "fonts/NotoSansArabic-Regular.ttf";
            if (!std::filesystem::exists(font_path_map[1]))
            {
                font_path_map[1] = "/usr/share/fonts/opentype/noto/NotoSansArabic-Regular.ttf";

                if (!std::filesystem::exists(font_path_map[1]))
                {
                    font_path_map[1] = "/usr/share/fonts/truetype/noto/NotoSansArabic-Regular.ttf";

                    if (!std::filesystem::exists(font_path_map[1]))
                    {
                        font_path_map[1] = "/usr/share/fonts/google-noto-cjk/NotoSansArabic-Regular.ttf";
                    } 
                } 
            }            
            
            for (auto& f:font_path_map)
            {
                instance_map[f.first] = std::make_shared<FontEnginePool>(50,f.first);
            }
        }

        return instance_map[font_type];
    }

    FontEnginePool(int count,int font_type) {
        for (int i = 0; i < count; ++i) {
            fts_map[font_type].emplace(create_(font_type), false);
        }

    }

    cv::Ptr<cv::freetype::FreeType2> get(int font_type) {
        cv::Ptr<cv::freetype::FreeType2> result = nullptr;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto& [ptr, busy] : fts_map[font_type]) {
                if (!busy) {
                    fts_map[font_type][ptr] = true;
                    result = ptr;
                }
            }

            if (result == nullptr) {
                result = create_(font_type);
                fts_map[font_type].emplace(result, true);
            }
        }
        return result;
    }

    void release(cv::Ptr<cv::freetype::FreeType2> ft2,int font_type) {
        std::lock_guard<std::mutex> lock(mutex_);
        fts_map[font_type][ft2] = false;
            
    }

private:
    cv::Ptr<cv::freetype::FreeType2> create_(int font_type) {
        cv::Ptr<cv::freetype::FreeType2> ft2 = nullptr;
        try {
            ft2 = cv::freetype::createFreeType2();
            ft2->loadFontData(font_path_map[font_type], 0);        
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to load font data: " << e.what() << '\n';
            return nullptr;
        }catch (...) {
            std::cerr << "Failed to load font : " << font_path_map[font_type] << '\n';
            return nullptr;
        }


        return ft2;
    }

private:
    std::map<int,std::map<cv::Ptr<cv::freetype::FreeType2>, bool>> fts_map;
    std::mutex mutex_;
};

void putTextZH(Mat& dst, const char* str, Point org, Scalar color, int fontSize, int font_type, const char* fn, bool italic, bool underline) {
    CV_Assert(dst.data != 0 && (dst.channels() == 1 || dst.channels() == 3));
    int x, y, r, b;
    if (org.x > dst.cols || org.y > dst.rows) {
        return;
    }

    // std::cout << "start:" << std::endl;
    // auto start = std::chrono::high_resolution_clock::now();

// cv::String ttf_pathname = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc";
//  cv::Ptr<cv::freetype::FreeType2> ft_tmp = cv::freetype::createFreeType2();
//  ft2 = ft_tmp;
//
#if 0
    thread_local cv::Ptr<cv::freetype::FreeType2> ft2 = nullptr;
    if (!ft2) {
        try {
            ft2 = cv::freetype::createFreeType2();
            ft2->loadFontData(*fontDataPath, 0);
        } catch (const std::exception& e) {
            std::cerr << "Failed to load font data: " << e.what() << '\n';
            return;
        }
    }

    if (!ft2) {
        return;
    }

#endif
    std::shared_ptr<FontEnginePool> fontEnginePool = FontEnginePool::getInstance(font_type);
    cv::Ptr<cv::freetype::FreeType2> ft2 = fontEnginePool->get(font_type);
    if(nullptr==ft2)
    {
        std::cerr<< "laod font error !!!!!" << std::endl;
        return;
    }   

    char buf[4096] = {0};
    cv::Size row_sz = ft2->getTextSize(str, fontSize, cv::FILLED, NULL);
    cv::Point pt_span(0, row_sz.height + 2);
    cv::Point pt_org = org;

    strcpy(buf, str);
    const char* ln = strtok(buf, "\n");
    int outTextY = 0;
    while (ln != 0) {
        if (dst.rows - pt_org.y <= pt_span.y || dst.cols - pt_org.x <= pt_span.x * 6) {
            std::cout << "PutText overflow\n";
            break;
        }

        ft2->putText(dst, ln, pt_org, fontSize, color, cv::FILLED, cv::LINE_AA, true);
        pt_org += pt_span;
        ln = strtok(0, "\n");
    }

    fontEnginePool->release(ft2,font_type);

}

void putTextZH_w(Mat& dst, const wchar_t* str_w, Point org, Scalar color, int fontSize, const char* fn, bool italic, bool underline) {
    
    std::wstring str_unicode = str_w;

    int is_arabic = is_mostly_arabic(str_unicode);

    std::string str = Util::UnicodeToUTF8(str_unicode);

    putTextZH(dst,str.c_str(),org,color,fontSize,is_arabic);
}

