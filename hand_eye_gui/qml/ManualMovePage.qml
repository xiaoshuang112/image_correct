import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "."
Item {
    id: page
    property var controller
    property bool locked: false
    signal leave()
    property string targetText: ""
    readonly property var parsed: controller.parseTcpTarget(targetText)
    readonly property bool canMove: !locked && controller.robotConnected && controller.selectedTcp !== "" && parsed.valid
    Timer {
        id: measure; interval: 0
        onTriggered: pair.height = Math.max(targetContent.implicitHeight + 40, liveContent.implicitHeight + 40,
            Math.min(Theme.pageCardMinHeight, scroll.availableHeight - 32))
    }
    ColumnLayout {
        anchors.fill: parent; spacing: 0
        Rectangle {
            color: Theme.surface; Layout.fillWidth: true; Layout.preferredHeight: 64
            RowLayout {
                anchors.fill: parent; anchors.margins: 12; spacing: 16
                CollectButton { objectName: "manualBackButton"; text: "← 返回"; quiet: true; enabled: !page.locked; onClicked: page.leave() }
                HmiText { text: "采集 › 手动移动 TCP"; color: Theme.ink; font.pixelSize: Theme.title; Layout.fillWidth: true }
                HmiText { text: "基坐标系"; font.pixelSize: Theme.meta }
            }
        }
        ScrollView {
            id: scroll; Layout.fillWidth: true; Layout.fillHeight: true; clip: true; contentWidth: availableWidth
            onAvailableHeightChanged: measure.restart()
            ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
            Item {
                width: scroll.availableWidth; implicitHeight: Math.max(scroll.availableHeight, pair.height + 32)
                Item {
                    id: pair
                    width: Math.min(parent.width - 32, Theme.pageContentWidth)
                    x: Math.round((parent.width - width) / 2)
                    y: Math.round((parent.height - height) / 2)
                    height: 400
                    onWidthChanged: measure.restart()
                    HmiPanel {
                        objectName: "manualTargetCard"; width: (parent.width - 16) / 2; height: parent.height; x: 0
                        ColumnLayout {
                            id: targetContent; width: parent.width; height: implicitHeight; onImplicitHeightChanged: measure.restart(); spacing: 16
                            HmiText { text: "目标位姿"; color: Theme.ink; font.pixelSize: Theme.appTitle }
                            HmiText { Layout.fillWidth: true; text: "TCP 工具" }
                            TcpSelector { objectName: "manualTcpSelector"; Layout.fillWidth: true; controller: page.controller; enabled: !page.locked && page.controller.robotConnected && page.controller.tcpNames.length > 0 }
                            HmiText { Layout.fillWidth: true; text: "X,Y,Z,Rx,Ry,Rz"; font.family: Theme.mono; color: Theme.ink }
                            CollectField {
                                objectName: "tcpTargetInput"; Layout.fillWidth: true; Layout.minimumWidth: 0
                                text: page.targetText; placeholderText: "300,0,400,0,180,0"
                                enabled: !page.locked; invalid: text.trim() !== "" && !page.parsed.valid
                                onTextChanged: page.targetText = text
                            }
                            HmiText { Layout.fillWidth: true; text: "一次输入六个数值，使用英文或中文逗号分隔。平移 mm，旋转为轴角 °。"; font.pixelSize: Theme.meta }
                            HmiText { objectName: "tcpTargetError"; Layout.fillWidth: true; text: page.targetText === "" ? "请输入目标坐标" : page.parsed.error; color: page.parsed.valid ? Theme.ink3 : Theme.warning; font.pixelSize: Theme.meta }
                            HmiText { text: "解析结果 · 只读"; font.pixelSize: Theme.meta }
                            PoseValues { objectName: "parsedTargetValues"; Layout.fillWidth: true; values: page.parsed.values || []; valid: page.parsed.valid }
                        }
                    }
                    HmiPanel {
                        objectName: "manualLiveCard"; width: (parent.width - 16) / 2; height: parent.height; x: width + 16
                        ColumnLayout {
                            id: liveContent; width: parent.width; height: implicitHeight; onImplicitHeightChanged: measure.restart(); spacing: 24
                            HmiText { text: "移动前确认"; color: Theme.ink; font.pixelSize: Theme.appTitle }
                            LivePose { Layout.fillWidth: true; controller: page.controller }
                            HmiText { Layout.fillWidth: true; text: "确认所选 TCP、基坐标系和路径周围空间。点击底部「移动到目标」才会下发运动。"; font.pixelSize: Theme.body }
                            HmiText { Layout.fillWidth: true; text: "移动中锁定坐标编辑、工具切换与返回。停止未确认时保持锁定，可重试停止。"; font.pixelSize: Theme.meta }
                            HmiText { objectName: "manualMoveProgress"; Layout.fillWidth: true; text: page.controller.manualMoveProgress; color: page.controller.motionFault ? Theme.danger : Theme.ink2 }
                        }
                    }
                }
            }
        }
        Rectangle {
            objectName: "manualFooter"; color: Theme.surface; Layout.fillWidth: true; Layout.preferredHeight: 72
            RowLayout {
                anchors.fill: parent; anchors.margins: 16; spacing: 16
                HmiText {
                    objectName: "manualFeedback"; Layout.fillWidth: true
                    text: page.controller.motionFault ? "停止尚未确认，请重试停止" : page.locked ? page.controller.manualMoveProgress || "操作进行中，请稍候" : !page.controller.robotConnected ? "请先在主页面连接机器人" : page.controller.selectedTcp === "" ? "请选择 TCP 工具" : page.parsed.valid ? "工具：" + page.controller.selectedTcp + " · 目标已解析，等待手动执行" : "输入完整且有效的目标坐标后可移动"
                    color: page.controller.motionFault ? Theme.danger : Theme.ink2
                }
                CollectButton {
                    objectName: "manualMoveButton"; Layout.preferredWidth: 160; Layout.preferredHeight: 44
                    text: page.locked ? page.controller.motionFault ? "重试停止" : "停止移动" : "移动到目标"
                    primary: !page.locked; danger: page.locked
                    enabled: page.locked || page.canMove
                    // Movement is deliberately mouse/touch only; Enter or Space cannot trigger it.
                    focusPolicy: Qt.NoFocus
                    Keys.onReturnPressed: event.accepted = true
                    Keys.onEnterPressed: event.accepted = true
                    Keys.onSpacePressed: event.accepted = true
                    onClicked: page.locked ? page.controller.cancelCollectionOperation() : page.controller.moveTcpTo(page.targetText)
                }
            }
        }
    }
}
