#include "mvs_error.hpp"

#include <cstdio>

namespace mvsdiag {
namespace {

struct ErrorEntry {
    unsigned int code;
    const char* name;
    const char* meaning;
};

// 与 MvErrorDefine.h（MVS SDK 4.7.x）一致的公开错误码常量。
constexpr ErrorEntry kErrorTable[] = {
    {0x00000000u, "MV_OK", "成功"},
    {0x80000000u, "MV_E_HANDLE", "错误或无效的句柄"},
    {0x80000001u, "MV_E_SUPPORT", "不支持的功能"},
    {0x80000002u, "MV_E_BUFOVER", "缓存已满"},
    {0x80000003u, "MV_E_CALLORDER", "函数调用顺序错误"},
    {0x80000004u, "MV_E_PARAMETER", "错误的参数"},
    {0x80000006u, "MV_E_RESOURCE", "资源申请失败"},
    {0x80000007u, "MV_E_NODATA", "无数据"},
    {0x80000008u, "MV_E_PRECONDITION", "前置条件不满足"},
    {0x80000009u, "MV_E_VERSION", "版本不匹配"},
    {0x8000000Au, "MV_E_NOENOUGH_BUF", "传入的缓存空间不足"},
    {0x8000000Bu, "MV_E_ABNORMAL_IMAGE", "异常图像(丢包/数据不完整)"},
    {0x8000000Cu, "MV_E_LOAD_LIBRARY", "动态加载DLL失败"},
    {0x8000000Du, "MV_E_NOOUTBUF", "没有可用的输出缓存"},
    {0x8000000Eu, "MV_E_ENCRYPT", "加密错误"},
    {0x8000000Fu, "MV_E_OPENFILE", "打开文件错误"},
    {0x80000010u, "MV_E_BUF_IN_USE", "缓存地址已使用"},
    {0x80000011u, "MV_E_BUF_INVALID", "无效的缓存地址"},
    {0x80000012u, "MV_E_NOALIGN_BUF", "缓存地址未对齐"},
    {0x80000013u, "MV_E_NOENOUGH_BUF_NUM", "缓存个数不足"},
    {0x80000014u, "MV_E_PORT_IN_USE", "端口被占用"},
    {0x80000015u, "MV_E_IMAGE_DECODEC", "解码错误"},
    {0x80000016u, "MV_E_UINT32_LIMIT", "数值超出uint32范围"},
    {0x80000017u, "MV_E_IMAGE_HEIGHT", "图像高度超出范围"},
    {0x80000018u, "MV_E_NOENOUGH_DDR", "DDR空间不足"},
    {0x80000019u, "MV_E_NOENOUGH_STREAM", "无足够流通道"},
    {0x8000001Au, "MV_E_NORESPONSE", "设备无响应"},
    {0x800000FFu, "MV_E_UNKNOW", "未知的错误"},
    {0x80000100u, "MV_E_GC_GENERIC", "通用GenICam错误"},
    {0x80000101u, "MV_E_GC_ARGUMENT", "GenICam参数错误"},
    {0x80000102u, "MV_E_GC_RANGE", "GenICam值超出范围"},
    {0x80000103u, "MV_E_GC_PROPERTY", "GenICam属性错误"},
    {0x80000104u, "MV_E_GC_RUNTIME", "GenICam运行时错误"},
    {0x80000105u, "MV_E_GC_LOGICAL", "GenICam逻辑错误"},
    {0x80000106u, "MV_E_GC_ACCESS", "GenICam访问权限错误"},
    {0x80000107u, "MV_E_GC_TIMEOUT", "GenICam超时"},
    {0x80000108u, "MV_E_GC_DYNAMICCAST", "GenICam转换异常"},
    {0x800001FFu, "MV_E_GC_UNKNOW", "GenICam未知错误"},
    {0x80000200u, "MV_E_NOT_IMPLEMENTED", "命令未实现"},
    {0x80000201u, "MV_E_INVALID_ADDRESS", "目标地址无效"},
    {0x80000202u, "MV_E_WRITE_PROTECT", "目标地址不可写"},
    {0x80000203u, "MV_E_ACCESS_DENIED", "设备无访问权限"},
    {0x80000204u, "MV_E_BUSY", "设备忙或网络断开"},
    {0x80000205u, "MV_E_PACKET", "网络包数据错误"},
    {0x80000206u, "MV_E_NETER", "网络相关错误"},
    {0x8000020Eu, "MV_E_SUPPORT_MODIFY_DEVICE_IP", "固定IP模式下不支持修改设备IP"},
    {0x8000020Fu, "MV_E_KEY_VERIFICATION", "切换访问权限的密钥校验失败"},
    {0x80000221u, "MV_E_IP_CONFLICT", "设备IP冲突"},
    {0x80000300u, "MV_E_USB_READ", "USB读错误"},
    {0x80000301u, "MV_E_USB_WRITE", "USB写错误"},
    {0x80000302u, "MV_E_USB_DEVICE", "USB设备异常"},
    {0x80000303u, "MV_E_USB_GENICAM", "USB GenICam错误"},
    {0x80000304u, "MV_E_USB_BANDWIDTH", "USB带宽不足"},
    {0x80000305u, "MV_E_USB_DRIVER", "USB驱动不匹配"},
    {0x800003FFu, "MV_E_USB_UNKNOW", "USB未知错误"},
    {0x80000400u, "MV_E_UPG_FILE_MISMATCH", "升级固件文件不匹配"},
    {0x80000401u, "MV_E_UPG_LANGUSGE_MISMATCH", "升级固件语言不匹配"},
    {0x80000402u, "MV_E_UPG_CONFLICT", "升级冲突(设备已在升级)"},
    {0x80000403u, "MV_E_UPG_INNER_ERR", "升级内部错误"},
    {0x800004FFu, "MV_E_UPG_UNKNOW", "升级未知错误"},
};

const ErrorEntry* find(unsigned int code) {
    for (const ErrorEntry& entry : kErrorTable) {
        if (entry.code == code)
            return &entry;
    }
    return nullptr;
}

}  // namespace

const char* errorName(unsigned int code) {
    const ErrorEntry* entry = find(code);
    return entry ? entry->name : "MV_E_UNKNOWN_CODE";
}

const char* errorMeaning(unsigned int code) {
    const ErrorEntry* entry = find(code);
    return entry ? entry->meaning : "";
}

std::string diagnostic(const char* stage, unsigned int code) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "stage=%s code=0x%08X(%s)",
                  stage ? stage : "unknown", code, errorName(code));
    std::string out(buf);
    const char* meaning = errorMeaning(code);
    if (meaning[0] != '\0') {
        out += " meaning=";
        out += meaning;
    }
    return out;
}

}  // namespace mvsdiag
