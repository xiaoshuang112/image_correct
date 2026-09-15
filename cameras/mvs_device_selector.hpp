#ifndef MVS_DEVICE_SELECTOR_HPP
#define MVS_DEVICE_SELECTOR_HPP

#include <string>
#include <vector>

// 海康 MVS 设备选择策略（纯 C++，不依赖 MVS SDK/Qt，可无硬件单测）。
//
// 固定选择顺序：
//   配置了 serial -> 必须精确匹配 serial
//   否则配置了 IP -> 必须精确匹配 GigE IP
//   否则只有一台设备 -> 使用唯一设备
//   否则 -> 拒绝打开并列出所有候选设备
//
// 禁止“找不到指定设备后静默使用第一台”——内参与相机身份绑定，
// 选错相机整套标定结果就是错的。
namespace mvsdiag {

// 传输层类型（与 MV_CC_DEVICE_INFO::nTLayerType 对应，避免依赖 SDK 头文件）
enum class MvsTransport {
    kGigE = 1,   // MV_GIGE_DEVICE
    kUsb = 4,    // MV_USB_DEVICE
    kOther = 0,
};

struct MvsDeviceDescriptor {
    MvsTransport transport = MvsTransport::kOther;
    std::string serial;
    std::string model;
    unsigned int ip = 0;        // 仅 GigE 有效，主机序（例如 192.168.0.10）
};

enum class MvsSelectionStatus {
    kSelected,          // index 有效
    kNoDevice,          // 枚举不到任何设备
    kSerialNotFound,    // 指定 serial 不存在
    kIpNotFound,        // 指定 IP 不存在（或该 IP 设备不是 GigE）
    kAmbiguousDevices,  // 未指定身份且存在多台设备
    kInvalidConfig,     // IP 字符串非法等配置错误
};

struct MvsSelectionOutcome {
    MvsSelectionStatus status = MvsSelectionStatus::kNoDevice;
    int index = -1;
    // 失败时的人类可读诊断：包含选择条件、设备数和全部候选设备。
    std::string message;
};

// IPv4 文本 ↔ 主机序整数。非法输入返回 false，ipOut 不被修改。
bool ipv4ToInt(const std::string& ipText, unsigned int& ipOut);
std::string ipv4ToString(unsigned int ip);

const char* transportName(MvsTransport transport);

// 列出全部候选设备，每行一台，用于失败诊断。
std::string formatDeviceList(const std::vector<MvsDeviceDescriptor>& devices);

// 执行固定选择策略。configSerial/configIp 为空串表示未配置。
MvsSelectionOutcome selectMvsDevice(
    const std::string& configSerial,
    const std::string& configIp,
    const std::vector<MvsDeviceDescriptor>& devices);

}  // namespace mvsdiag

#endif  // MVS_DEVICE_SELECTOR_HPP
