#ifndef MVS_ERROR_HPP
#define MVS_ERROR_HPP

#include <string>

// 海康 MVS SDK 错误码 → 诊断信息。
// 纯 C++ 实现，不依赖 MVS 头文件/Qt，便于在无 SDK、无硬件环境做单测。
// 错误码数值与 MvCameraControl.h 附带的 MvErrorDefine.h 保持一致（ABI 稳定常量）。
namespace mvsdiag {

// 返回错误码的符号名（如 "MV_E_NODATA"）；未知返回 "MV_E_UNKNOWN_CODE"。
const char* errorName(unsigned int code);

// 返回错误码的中文含义；未知返回空串。
const char* errorMeaning(unsigned int code);

// 组装统一诊断串：
//   stage=open_device code=0x80000203(MV_E_ACCESS_DENIED) meaning=设备无访问权限
std::string diagnostic(const char* stage, unsigned int code);

}  // namespace mvsdiag

#endif  // MVS_ERROR_HPP
