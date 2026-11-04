#include "hand_eye_gui/RobotTcp.hpp"
#include <QCoreApplication>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
template<class F> void rejects(F operation, const char* message) {
    bool failed = false;
    try { operation(); } catch (const std::exception&) { failed = true; }
    require(failed, message);
}
TCPMsg makeTool(const char* name, double z) {
    TCPMsg result{};
    std::strncpy(result.tcpName, name, sizeof(result.tcpName) - 1);
    result.tcpOffset[2] = z;
    return result;
}
std::array<TCPMsg, TCP_MAX_NUM> tools = [] {
    std::array<TCPMsg, TCP_MAX_NUM> result{};
    for (int i = 0; i < TCP_MAX_NUM; ++i)
        result[i] = makeTool(qPrintable(QString("tool-%1").arg(i)), i * 10);
    result[0] = makeTool("probe-a", 100);
    result[1] = makeTool("probe-b", 200);
    return result;
}();
int count = 2, lookups = 0, poseReads = 0;
CRresult listCode = sucess, currentCode = sucess, lookupCode = sucess, poseCode = sucess;
TCPMsg current = tools[0], assigned = tools[1];
bool nonFinitePose = false;
double lastOffset[ROB_AXIS_NUM]{};
}

extern "C" CRresult cr_get_allTCPmsg(RobotHandle, TCPMsg* output, int* valid) {
    *valid = count;
    std::copy(tools.begin(), tools.begin() + std::min(std::max(count, 0), TCP_MAX_NUM), output);
    return listCode;
}
extern "C" CRresult cr_get_currentTCPmsg(RobotHandle, TCPMsg* output) {
    *output = current;
    return currentCode;
}
extern "C" CRresult cr_cfg_tcp_accord_name_get(RobotHandle, char* name, TCPMsg* output) {
    ++lookups;
    require(std::strcmp(name, "probe-b") == 0, "read used active TCP instead of selected TCP");
    *output = assigned;
    return lookupCode;
}
extern "C" CRresult cr_get_AssignTCP_Pose(RobotHandle, double* offset, double* pose) {
    ++poseReads;
    std::copy(offset, offset + ROB_AXIS_NUM, lastOffset);
    for (int i = 0; i < ROB_AXIS_NUM; ++i) pose[i] = 10.0 + i;
    if (nonFinitePose) pose[1] = std::numeric_limits<double>::quiet_NaN();
    return poseCode;
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    const auto error = static_cast<CRresult>(1);
    try {
        const int originalLookups = lookups;
        const auto parsed = collection::parseTcpTarget(" 1， -2.5, +3.0 ， .4,5e1, -6 ");
        require(parsed == std::array<double, ROB_AXIS_NUM>{1, -2.5, 3, .4, 50, -6}, "decimal parser changed values");
        for (const auto* text : {"", "1,2,3,4,5", "1,2,3,4,5,6,", "1,2,3,4,5,6,7",
                "1,2,,4,5,6", "1,2,3,4,5,", "1,2,3,4,5,NaN", "Inf,2,3,4,5,6",
                "1,2,3,4,5,-inf", "1,2,3,4,5,1e999", "1,2,3,4,5,6mm", "1,2,3,4,5,0x10",
                "1,2,3,4,5,6 7", "1;2;3;4;5;6", "1,2,3,4,5,1_000"})
            rejects([&] { collection::parseTcpTarget(QString::fromUtf8(text)); }, "invalid target accepted");
        require(lookups == originalLookups, "pure parser contacted SDK");
        auto list = collection::readTcpList(1);
        require(list.names == QStringList{"probe-a", "probe-b"} && list.current == "probe-a", "TCP list order/default changed");
        count = TCP_MAX_NUM;
        require(collection::readTcpList(1).names.size() == TCP_MAX_NUM, "SDK maximum TCP count rejected");
        count = 2;
        const auto chosen = collection::readTcpPose(1, "probe-b");
        require(chosen.name == "probe-b" && chosen.offset[2] == 200 && lastOffset[2] == 200 && chosen.pose[0] == 10,
                "selected TCP or exact offset not used");
        assigned.tcpOffset[2] = 345;
        const auto updated = collection::readTcpPose(1, "probe-b");
        require(updated.offset[2] == 345 && lastOffset[2] == 345 && lookups == 2, "save reused stale tool offset");
        currentCode = error;
        list = collection::readTcpList(1);
        require(list.names.size() == 2 && list.current.isEmpty() && !list.warning.isEmpty(), "current TCP failure chose another tool");
        currentCode = sucess;
        current = {};
        list = collection::readTcpList(1);
        require(list.names.size() == 2 && list.current.isEmpty(), "malformed active tool discarded usable list");
        current = makeTool("missing", 0);
        require(collection::readTcpList(1).current.isEmpty(), "missing active TCP chose first tool");
        count = 0;
        require(collection::readTcpList(1).names.isEmpty(), "empty list fabricated a tool");
        for (int invalid : {-1, TCP_MAX_NUM + 1}) {
            count = invalid;
            rejects([&] { collection::readTcpList(1); }, "invalid list count accepted");
        }
        count = 2; listCode = error;
        rejects([&] { collection::readTcpList(1); }, "list RPC failure ignored");
        listCode = sucess; tools[1] = tools[0];
        rejects([&] { collection::readTcpList(1); }, "duplicate TCP name accepted");
        tools[1] = makeTool("probe-b", 200);
        std::memset(tools[1].tcpName, 'a', sizeof(tools[1].tcpName));
        rejects([&] { collection::readTcpList(1); }, "unterminated TCP name accepted");
        tools[1] = makeTool("", 200);
        rejects([&] { collection::readTcpList(1); }, "empty TCP name accepted");
        tools[1] = makeTool("probe-b", std::numeric_limits<double>::infinity());
        rejects([&] { collection::readTcpList(1); }, "invalid list offset accepted");
        const int before = poseReads;
        lookupCode = error;
        rejects([&] { collection::readTcpPose(1, "probe-b"); }, "deleted TCP silently fell back");
        require(poseReads == before, "pose queried after tool lookup failure");
        lookupCode = sucess; assigned = makeTool("probe-a", 100);
        rejects([&] { collection::readTcpPose(1, "probe-b"); }, "mismatched tool returned by SDK accepted");
        assigned = makeTool("probe-b", std::numeric_limits<double>::quiet_NaN());
        rejects([&] { collection::readTcpPose(1, "probe-b"); }, "invalid selected offset accepted");
        assigned = makeTool("probe-b", 200); poseCode = error;
        rejects([&] { collection::readTcpPose(1, "probe-b"); }, "pose RPC failure ignored");
        poseCode = sucess; nonFinitePose = true;
        rejects([&] { collection::readTcpPose(1, "probe-b"); }, "invalid actual pose accepted");
        rejects([&] { collection::readTcpPose(1, ""); }, "empty selection accepted");
        std::cout << "TCP list, selected-tool read, fresh offsets and RPC failures passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
