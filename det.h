#pragma once

// #define DEBUG_STITCH_IMAGE  // 全部的调试细节，输出大量中间信息，按需使用
// #define SAVE_STITCH_IMAGE_RESULT // 输出拼接结果，占用一定运行时间，调试时建议打开

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS

#else
  
#endif


#include <vector>
#include <string>
#include <map>
#include <filesystem>
#include <iostream>
#include <optional>


typedef int (*AI_det_callback)(char* msg, int msg_len, int flag);

