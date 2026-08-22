#include "mvs_device_selector.hpp"

#include <arpa/inet.h>

#include <cstdio>

namespace mvsdiag {

bool ipv4ToInt(const std::string& ipText, unsigned int& ipOut) {
    if (ipText.empty())
        return false;
    in_addr addr{};
    if (inet_pton(AF_INET, ipText.c_str(), &addr) != 1)
        return false;
    ipOut = ntohl(addr.s_addr);
    return true;
}

std::string ipv4ToString(unsigned int ip) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
                  (ip >> 24) & 0xFF, (ip >> 16) & 0xFF,
                  (ip >> 8) & 0xFF, ip & 0xFF);
    return std::string(buf);
}

const char* transportName(MvsTransport transport) {
    switch (transport) {
    case MvsTransport::kGigE: return "gige";
    case MvsTransport::kUsb: return "usb";
    default: return "other";
    }
}

std::string formatDeviceList(const std::vector<MvsDeviceDescriptor>& devices) {
    std::string out;
    for (size_t i = 0; i < devices.size(); ++i) {
        const MvsDeviceDescriptor& d = devices[i];
        char line[256];
        if (d.transport == MvsTransport::kGigE) {
            std::snprintf(line, sizeof(line),
                          "  [%zu] transport=gige serial=%s model=%s ip=%s",
                          i, d.serial.c_str(), d.model.c_str(),
                          ipv4ToString(d.ip).c_str());
        } else {
            std::snprintf(line, sizeof(line),
                          "  [%zu] transport=%s serial=%s model=%s",
                          i, transportName(d.transport),
                          d.serial.c_str(), d.model.c_str());
        }
        out += line;
        out += '\n';
    }
    return out;
}

MvsSelectionOutcome selectMvsDevice(
    const std::string& configSerial,
    const std::string& configIp,
    const std::vector<MvsDeviceDescriptor>& devices) {

    MvsSelectionOutcome outcome;

    if (devices.empty()) {
        outcome.status = MvsSelectionStatus::kNoDevice;
        outcome.message = "未枚举到任何 MVS 设备 (GigE/USB)";
        return outcome;
    }

    // 1. 显式序列号优先，必须精确匹配
    if (!configSerial.empty()) {
        for (size_t i = 0; i < devices.size(); ++i) {
            if (devices[i].serial == configSerial) {
                outcome.status = MvsSelectionStatus::kSelected;
                outcome.index = static_cast<int>(i);
                return outcome;
            }
        }
        outcome.status = MvsSelectionStatus::kSerialNotFound;
        outcome.message = "配置的相机序列号不存在: serial=" + configSerial +
            "，已枚举 " + std::to_string(devices.size()) + " 台设备:\n" +
            formatDeviceList(devices);
        return outcome;
    }

    // 2. 显式 IP，必须精确匹配 GigE 设备当前 IP
    if (!configIp.empty()) {
        unsigned int targetIp = 0;
        if (!ipv4ToInt(configIp, targetIp)) {
            outcome.status = MvsSelectionStatus::kInvalidConfig;
            outcome.message = "配置的相机 IP 非法: " + configIp;
            return outcome;
        }
        for (size_t i = 0; i < devices.size(); ++i) {
            if (devices[i].transport == MvsTransport::kGigE &&
                devices[i].ip == targetIp) {
                outcome.status = MvsSelectionStatus::kSelected;
                outcome.index = static_cast<int>(i);
                return outcome;
            }
        }
        outcome.status = MvsSelectionStatus::kIpNotFound;
        outcome.message = "配置的相机 IP 不存在: ip=" + configIp +
            "，已枚举 " + std::to_string(devices.size()) + " 台设备:\n" +
            formatDeviceList(devices);
        return outcome;
    }

    // 3. 未配置身份：仅当全网只有一台设备时才允许使用
    if (devices.size() == 1) {
        outcome.status = MvsSelectionStatus::kSelected;
        outcome.index = 0;
        return outcome;
    }

    outcome.status = MvsSelectionStatus::kAmbiguousDevices;
    outcome.message = "未配置相机序列号或 IP，且枚举到 " +
        std::to_string(devices.size()) +
        " 台设备，无法确定目标，拒绝打开。候选设备:\n" +
        formatDeviceList(devices);
    return outcome;
}

}  // namespace mvsdiag
