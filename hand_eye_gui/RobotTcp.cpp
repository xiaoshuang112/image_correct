#include "RobotTcp.hpp"
#include <QByteArray>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
void check(CRresult code, const QString& action) {
    if (code != sucess)
        throw std::runtime_error(QString("%1失败（错误码 %2）").arg(action).arg(code).toStdString());
}

QString toolName(const TCPMsg& tcp) {
    const char* end = std::find(tcp.tcpName, tcp.tcpName + sizeof(tcp.tcpName), '\0');
    if (end == tcp.tcpName || end == tcp.tcpName + sizeof(tcp.tcpName))
        throw std::runtime_error("TCP 名称为空或未正确终止");
    const QByteArray bytes(tcp.tcpName, int(end - tcp.tcpName));
    const QString name = QString::fromLocal8Bit(bytes);
    if (name.toLocal8Bit() != bytes || name.trimmed().isEmpty())
        throw std::runtime_error("TCP 名称编码无效");
    return name;
}

void checkOffset(const TCPMsg& tcp) {
    if (!std::all_of(tcp.tcpOffset, tcp.tcpOffset + ROB_AXIS_NUM,
                     [](double value) { return std::isfinite(value); }))
        throw std::runtime_error("TCP 工具偏移包含无效数值");
}
} // namespace

namespace collection {
TcpList readTcpList(RobotHandle handle) {
    std::array<TCPMsg, TCP_MAX_NUM> tools{};
    int count = 0;
    check(cr_get_allTCPmsg(handle, tools.data(), &count), "TCP 列表读取");
    if (count < 0 || count > TCP_MAX_NUM)
        throw std::runtime_error("TCP 列表数量超出 SDK 范围");
    TcpList result;
    for (int i = 0; i < count; ++i) {
        const QString name = toolName(tools[i]);
        checkOffset(tools[i]);
        if (result.names.contains(name)) throw std::runtime_error("TCP 列表包含重名工具");
        result.names.append(name);
    }
    if (result.names.isEmpty()) return result;
    TCPMsg current{};
    const CRresult code = cr_get_currentTCPmsg(handle, &current);
    if (code != sucess) {
        result.warning = QString("当前 TCP 读取失败（错误码 %1），请手动选择工具").arg(code);
    } else {
        try {
            const QString name = toolName(current);
            if (result.names.contains(name)) result.current = name;
            else result.warning = "当前 TCP 不在列表中，请手动选择工具";
        } catch (const std::exception&) {
            result.warning = "当前 TCP 名称无效，请手动选择工具";
        }
    }
    return result;
}

TcpPose readTcpPose(RobotHandle handle, const QString& name) {
    QByteArray bytes = name.toLocal8Bit();
    if (bytes.isEmpty() || bytes.size() >= int(sizeof(TCPMsg::tcpName)) || bytes.contains('\0'))
        throw std::runtime_error("请选择有效的 TCP 工具");
    TCPMsg tcp{};
    check(cr_cfg_tcp_accord_name_get(handle, bytes.data(), &tcp), "所选 TCP 读取");
    if (toolName(tcp) != name) throw std::runtime_error("所选 TCP 已变化，请刷新列表后重选");
    checkOffset(tcp);
    TcpPose result;
    result.name = name;
    std::copy(tcp.tcpOffset, tcp.tcpOffset + ROB_AXIS_NUM, result.offset.begin());
    result.pose.fill(std::numeric_limits<double>::quiet_NaN());
    check(cr_get_AssignTCP_Pose(handle, tcp.tcpOffset, result.pose.data()), "TCP 位姿读取");
    if (!std::all_of(result.pose.begin(), result.pose.end(), [](double v) { return std::isfinite(v); }))
        throw std::runtime_error("TCP 位姿包含无效数值");
    return result;
}
} // namespace collection
