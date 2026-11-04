import QtQuick 2.15
import QtQuick.Layouts 1.15
import "."
ColumnLayout {
    id: pose
    property var controller
    readonly property bool valid: controller.tcpPoseValid && controller.robotConnected && controller.selectedTcp !== "" && controller.tcpPoseTool === controller.selectedTcp
    spacing: 8
    RowLayout {
        Layout.fillWidth: true
        HmiText { text: "当前 TCP · " + (pose.controller.selectedTcp || "未选择工具"); color: Theme.ink; font.weight: Font.DemiBold; Layout.fillWidth: true; elide: Text.ElideRight; wrapMode: Text.NoWrap }
        HmiText { text: pose.valid ? pose.controller.tcpPoseTimestamp : "—"; font.family: Theme.mono; font.pixelSize: Theme.meta }
    }
    PoseValues { objectName: "livePoseValues"; Layout.fillWidth: true; values: pose.controller.tcpPoseValues; valid: pose.valid }
    HmiText {
        objectName: "livePoseStatus"; Layout.fillWidth: true; font.pixelSize: Theme.meta
        text: pose.valid ? "基坐标系 · 平移 mm / 轴角 °" : pose.controller.tcpPoseTool !== pose.controller.selectedTcp && pose.controller.tcpPoseTool !== "" ? "工具已切换，等待所选 TCP 的新位姿" : pose.controller.tcpPoseStatus
        color: pose.valid ? Theme.ink3 : Theme.warning
    }
}
