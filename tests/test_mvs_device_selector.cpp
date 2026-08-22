// MVS 设备选择策略无硬件测试：
//   serial/IP/唯一设备/多设备歧义、IPv4 转换、GigE 设备信息解析与诊断输出。
#include "test_check.hpp"

#include "cameras/mvs_device_selector.hpp"

#include <string>
#include <vector>

using namespace mvsdiag;

namespace {

MvsDeviceDescriptor gige(const std::string& serial,
                         const std::string& model,
                         const std::string& ipText) {
    MvsDeviceDescriptor d;
    d.transport = MvsTransport::kGigE;
    d.serial = serial;
    d.model = model;
    ipv4ToInt(ipText, d.ip);
    return d;
}

MvsDeviceDescriptor usb(const std::string& serial, const std::string& model) {
    MvsDeviceDescriptor d;
    d.transport = MvsTransport::kUsb;
    d.serial = serial;
    d.model = model;
    return d;
}

}  // namespace

namespace suite {

int test_mvs_device_selector() {
    std::cout << "---- mvs_device_selector ----\n";

    // ---- IPv4 转换 ----
    unsigned int ip = 0;
    TEST_CHECK(ipv4ToInt("192.168.3.10", ip) && ip == 0xC0A8030Au,
               "ipv4_to_int_host_order");
    TEST_CHECK(ipv4ToString(0xC0A8030Au) == "192.168.3.10", "ipv4_to_string");
    ip = 12345;  // 非法输入不得修改输出
    TEST_CHECK(!ipv4ToInt("999.1.2.3", ip) && ip == 12345u, "ipv4_invalid_octet");
    TEST_CHECK(!ipv4ToInt("192.168.3", ip), "ipv4_too_short");
    TEST_CHECK(!ipv4ToInt("", ip), "ipv4_empty");
    TEST_CHECK(!ipv4ToInt("192.168.3.10 ", ip), "ipv4_trailing_space");

    // ---- 设备列表诊断 ----
    const std::vector<MvsDeviceDescriptor> two = {
        gige("DA1111111", "MV-CS050-10GM", "192.168.3.10"),
        usb("DA2222222", "MV-CS050-10UC"),
    };
    const std::string list = formatDeviceList(two);
    TEST_CHECK(list.find("DA1111111") != std::string::npos &&
                   list.find("DA2222222") != std::string::npos &&
                   list.find("192.168.3.10") != std::string::npos,
               "format_device_list_contains_all");

    // ---- 空枚举 ----
    {
        const MvsSelectionOutcome r = selectMvsDevice("", "", {});
        TEST_CHECK(r.status == MvsSelectionStatus::kNoDevice, "no_device");
    }

    // ---- serial 优先 ----
    {
        const MvsSelectionOutcome r = selectMvsDevice("DA2222222", "", two);
        TEST_CHECK(r.status == MvsSelectionStatus::kSelected && r.index == 1,
                   "serial_exact_match");
    }
    {
        const MvsSelectionOutcome r = selectMvsDevice("DA9999999", "", two);
        TEST_CHECK(r.status == MvsSelectionStatus::kSerialNotFound &&
                       r.message.find("DA9999999") != std::string::npos &&
                       r.message.find("DA1111111") != std::string::npos,
                   "serial_not_found_lists_candidates");
    }
    // serial 命中时不受 IP 配置影响
    {
        const MvsSelectionOutcome r =
            selectMvsDevice("DA1111111", "192.168.3.99", two);
        TEST_CHECK(r.status == MvsSelectionStatus::kSelected && r.index == 0,
                   "serial_takes_priority_over_ip");
    }

    // ---- IP 选择（仅 GigE）----
    {
        const MvsSelectionOutcome r = selectMvsDevice("", "192.168.3.10", two);
        TEST_CHECK(r.status == MvsSelectionStatus::kSelected && r.index == 0,
                   "ip_exact_match");
    }
    {
        const MvsSelectionOutcome r = selectMvsDevice("", "192.168.3.99", two);
        TEST_CHECK(r.status == MvsSelectionStatus::kIpNotFound, "ip_not_found");
    }
    {
        const MvsSelectionOutcome r = selectMvsDevice("", "not-an-ip", two);
        TEST_CHECK(r.status == MvsSelectionStatus::kInvalidConfig,
                   "ip_invalid_config");
    }

    // ---- 唯一设备 / 多设备歧义 ----
    {
        const std::vector<MvsDeviceDescriptor> one = {
            gige("DA1111111", "MV-CS050-10GM", "192.168.3.10")};
        const MvsSelectionOutcome r = selectMvsDevice("", "", one);
        TEST_CHECK(r.status == MvsSelectionStatus::kSelected && r.index == 0,
                   "unique_device_selected");
    }
    {
        const MvsSelectionOutcome r = selectMvsDevice("", "", two);
        TEST_CHECK(r.status == MvsSelectionStatus::kAmbiguousDevices &&
                       r.message.find("DA1111111") != std::string::npos &&
                       r.message.find("DA2222222") != std::string::npos,
                   "ambiguous_devices_rejected");
    }

    return testutil::takeFailures();
}

}  // namespace suite
