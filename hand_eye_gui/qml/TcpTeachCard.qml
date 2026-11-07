import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "."
CollectCard {
    id: card
    objectName: "tcpTeachCard"
    property var controller
    property bool locked: false
    property bool windowReady: false
    property bool pageVisible: true
    property bool targetDraft: false
    property var draftTexts: []
    property string feedback: ""
    property string lastSelectedTcp: ""
    property int heldAxis: -1
    property int heldSign: 0
    property var lease: 0
    readonly property bool jogHeld: heldAxis >= 0
    readonly property bool narrow: width < 680
    readonly property bool poseReady: controller.robotConnected && controller.selectedTcp !== "" && controller.tcpPoseValid && controller.tcpPoseFresh && controller.tcpPoseTool === controller.selectedTcp && actualValues.length === 6 && actualValues.every(function(v) { return isFinite(Number(v)) })
    readonly property var actualValues: controller.tcpDisplayPoseValues
    readonly property string targetText: draftTexts.join(",")
    readonly property var parsed: controller.parseTcpTarget(targetText)
    readonly property bool speedsValid: linearSpeed.acceptableInput && angularSpeed.acceptableInput && Number(linearSpeed.text) > 0 && Number(angularSpeed.text) > 0
    readonly property bool canJog: poseReady && !targetDraft && !locked && !jogHeld && speedsValid && windowReady && pageVisible
    readonly property string reason: controller.motionFault ? "停止尚未确认，请重试停止" : locked ? controller.manualMoveProgress || "操作进行中" : !controller.robotConnected ? "连接机器人后可示教" : !controller.selectedTcp ? "请选择 TCP 工具" : !poseReady ? controller.tcpPoseStatus || "位姿已过期，等待新数据" : !speedsValid ? "请填写有效的手动速度" : targetDraft ? parsed.valid ? "目标待执行 · 长按微调已暂停" : "请输入六个有效目标数值" : "长按 ± 250 ms 开始微调，松开停止 · BASE"
    padding: 10
    implicitWidth: 480
    implicitHeight: 210
    function format(value) { return Number(value).toFixed(2) }
    function beginEdit(axis, value) {
        var next = targetDraft ? draftTexts.slice() : actualValues.map(function(v) { return Number(v).toString() })
        while (next.length < 6) next.push("")
        next[axis] = value
        draftTexts = next; targetDraft = true; feedback = ""
    }
    function useCurrent() { if (locked || jogHeld || !poseReady) return; targetDraft = false; draftTexts = []; feedback = "已使用当前坐标"; card.forceActiveFocus() }
    function changeMode(mode) {
        if (locked || jogHeld || mode === controller.orientationMode) return
        if (targetDraft) {
            var result = controller.convertTcpPose(targetText, controller.orientationMode, mode)
            if (!result.valid) { feedback = result.error; return }
            draftTexts = result.values.map(function(v) { return Number(v).toString() })
        }
        controller.orientationMode = mode
        feedback = ""
    }
    function pressJog(axis, sign) {
        if (!canJog) return
        controller.setManualSpeeds(Number(linearSpeed.text), Number(angularSpeed.text))
        heldAxis = axis; heldSign = sign; feedback = "继续按住以开始微调…"; holdDelay.restart()
    }
    function stopJog() {
        holdDelay.stop()
        var token = lease
        lease = 0; heldAxis = -1; heldSign = 0
        if (token > 0) controller.stopTcpJog(token)
    }
    function moveTarget() {
        if (locked || jogHeld || !targetDraft || !parsed.valid || !poseReady || !speedsValid) return
        controller.setManualSpeeds(Number(linearSpeed.text), Number(angularSpeed.text))
        controller.moveTcpToDisplay(targetText, controller.orientationMode)
        if (controller.manualMoveActive) { targetDraft = false; draftTexts = [] }
    }
    onWindowReadyChanged: if (!windowReady) stopJog()
    onPageVisibleChanged: if (!pageVisible) stopJog()
    Component.onCompleted: lastSelectedTcp = controller.selectedTcp
    Component.onDestruction: stopJog()
    Timer {
        id: holdDelay; interval: 250
        onTriggered: {
            if (!card.jogHeld || !card.windowReady || !card.pageVisible || !card.poseReady || card.targetDraft || card.locked) { card.stopJog(); return }
            card.lease = card.controller.beginTcpJog(card.heldAxis, card.heldSign, card.controller.orientationMode)
            if (card.lease <= 0) card.stopJog()
        }
    }
    Timer {
        interval: 100; running: card.jogHeld; repeat: true
        onTriggered: {
            if (!card.windowReady || !card.pageVisible || !card.poseReady) { card.stopJog(); return }
            if (card.lease > 0) card.controller.renewTcpJog(card.lease)
        }
    }
    Connections {
        target: card.controller
        function onCoordinatesImported(coordinates, tcpName, error) {
            if (error) { card.feedback = error; return }
            var converted = card.controller.convertTcpPose(coordinates, "axis", card.controller.orientationMode)
            if (!converted.valid) { card.feedback = converted.error; return }
            card.draftTexts = converted.values.map(function(v) { return Number(v).toString() })
            card.targetDraft = true
            card.feedback = "已导入目标" + (tcpName ? " · 文件 TCP：" + tcpName : "") + "，等待手动执行"
        }
        function onTcpToolsChanged() {
            if (card.lastSelectedTcp !== card.controller.selectedTcp) { card.stopJog(); card.targetDraft = false; card.draftTexts = []; card.lastSelectedTcp = card.controller.selectedTcp }
        }
        function onRobotConnectedChanged() { if (!card.controller.robotConnected) card.stopJog() }
    }
    ColumnLayout {
        anchors.fill: parent; spacing: 6
        RowLayout {
            objectName: "teachSpeedRow"; Layout.fillWidth: true; Layout.preferredHeight: 32; spacing: card.narrow ? 2 : 8
            HmiText { text: "平移"; font.pixelSize: Theme.meta }
            CollectField {
                id: linearSpeed; objectName: "manualLinearSpeedInput"; Layout.preferredWidth: card.narrow ? 46 : 62; implicitHeight: 32; font.pixelSize: Theme.meta; leftPadding: 4; rightPadding: 4
                text: String(card.controller.manualLinearSpeedMmS); enabled: !card.locked && !card.jogHeld
                validator: DoubleValidator { bottom: 1; top: 1000; decimals: 2; locale: "C" }
                invalid: !acceptableInput
                onEditingFinished: if (card.speedsValid) card.controller.setManualSpeeds(Number(text), Number(angularSpeed.text))
            }
            HmiText { text: "mm/s"; font.pixelSize: Theme.meta }
            HmiText { text: "角度步进"; font.pixelSize: Theme.meta }
            CollectField {
                id: angularSpeed; objectName: "manualAngularSpeedInput"; Layout.preferredWidth: card.narrow ? 40 : 56; implicitHeight: 32; font.pixelSize: Theme.meta; leftPadding: 4; rightPadding: 4
                text: String(card.controller.manualAngularSpeedDegS); enabled: !card.locked && !card.jogHeld
                validator: DoubleValidator { bottom: 0.1; top: 180; decimals: 2; locale: "C" }
                invalid: !acceptableInput
                ToolTip.visible: hovered; ToolTip.text: "用于长按旋转微调；移动到输入坐标时使用平移速度，由机器人规划姿态变化"
                onEditingFinished: if (card.speedsValid) card.controller.setManualSpeeds(Number(linearSpeed.text), Number(text))
            }
            HmiText { text: "°/s"; font.pixelSize: Theme.meta }
            CompactSelector {
                objectName: "orientationModeSelector"; Layout.preferredWidth: card.narrow ? 104 : 130; Layout.preferredHeight: 32; font.family: Theme.sans; font.pixelSize: Theme.meta
                model: ["轴角", "欧拉角 ZYX"]; currentIndex: card.controller.orientationMode === "euler" ? 1 : 0
                enabled: !card.locked && !card.jogHeld && (!card.targetDraft || card.parsed.valid)
                onActivated: {
                    card.changeMode(currentIndex === 1 ? "euler" : "axis")
                    // A rejected conversion does not change the controller, so
                    // its NOTIFY cannot restore a user-modified ComboBox index.
                    currentIndex = Qt.binding(function() { return card.controller.orientationMode === "euler" ? 1 : 0 })
                }
            }
            Item { Layout.fillWidth: true; Layout.minimumWidth: 0 }
            CollectButton { objectName: "copyPoseButton"; text: "复制当前"; small: true; leftPadding: 8; rightPadding: 8; enabled: card.poseReady; onClicked: card.controller.copyCurrentTcpCoordinates(card.controller.orientationMode) }
            HmiText { visible: !card.narrow; text: card.targetDraft ? "目标待执行" : "实时 · BASE"; font.pixelSize: Theme.meta }
        }
        GridLayout {
            objectName: "teachAxes"; columns: 3; Layout.fillWidth: true; Layout.fillHeight: true; columnSpacing: card.narrow ? 6 : 12; rowSpacing: 6
            Repeater {
                model: ["X", "Y", "Z", "Rx", "Ry", "Rz"]
                RowLayout {
                    id: axisGroup; property int axis: index
                    Layout.fillWidth: true; Layout.minimumWidth: 0; Layout.preferredHeight: 44; spacing: card.narrow ? 2 : 4
                    HmiText { text: modelData; Layout.preferredWidth: card.narrow ? 18 : 24; color: Theme.ink; font.family: Theme.mono; font.pixelSize: Theme.meta }
                    Repeater {
                        model: 3
                        Loader {
                            property int slot: index
                            Layout.fillWidth: slot === 1; Layout.preferredWidth: slot === 1 ? -1 : card.narrow ? 28 : 36; Layout.minimumWidth: slot === 1 ? 32 : card.narrow ? 28 : 36; Layout.preferredHeight: 44
                            sourceComponent: slot === 1 ? valueField : jogButton
                            Component {
                                id: valueField
                                CollectField {
                                    objectName: "tcpAxis" + axisGroup.axis
                                    text: card.targetDraft ? (card.draftTexts[axisGroup.axis] || "") : card.poseReady ? card.format(card.actualValues[axisGroup.axis]) : "—"
                                    horizontalAlignment: TextInput.AlignHCenter; font.pixelSize: card.narrow ? Theme.meta : Theme.body; leftPadding: 2; rightPadding: 2
                                    enabled: !card.locked && !card.jogHeld && card.poseReady
                                    invalid: card.targetDraft && (text.trim() === "" || !isFinite(Number(text)))
                                    onTextEdited: card.beginEdit(axisGroup.axis, text)
                                    Keys.onReturnPressed: event.accepted = true
                                    Keys.onEnterPressed: event.accepted = true
                                    Keys.onSpacePressed: event.accepted = true
                                    ToolTip.visible: hovered; ToolTip.text: card.targetDraft ? "输入目标 · " + (axisGroup.axis < 3 ? "mm" : "°") : "当前坐标 · 编辑后冻结六个目标值"
                                }
                            }
                            Component {
                                id: jogButton
                                Rectangle {
                                    id: jog; objectName: "jog" + axisGroup.axis + (slot === 0 ? "Minus" : "Plus")
                                    property bool ownHold: card.heldAxis === axisGroup.axis && card.heldSign === (slot === 0 ? -1 : 1)
                                    enabled: card.canJog || ownHold
                                    opacity: enabled ? 1 : 0.4; radius: Theme.radiusControl
                                    color: ownHold ? Theme.accentSoft : jogArea.containsMouse ? Theme.raisedHover : Theme.raised
                                    border.color: ownHold ? Theme.accent : Theme.hairline
                                    HmiText { anchors.centerIn: parent; text: slot === 0 ? "−" : "+"; font.pixelSize: 20; color: Theme.ink }
                                    MouseArea {
                                        id: jogArea; objectName: "jogSurface" + axisGroup.axis + (slot === 0 ? "Minus" : "Plus"); anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.LeftButton
                                        onPressed: card.pressJog(axisGroup.axis, slot === 0 ? -1 : 1)
                                        onReleased: if (jog.ownHold) card.stopJog()
                                        onCanceled: if (jog.ownHold) card.stopJog()
                                        onExited: if (jog.ownHold) card.stopJog()
                                    }
                                    ToolTip.visible: jogArea.containsMouse; ToolTip.text: card.canJog ? "长按微调，松开停止" : card.reason
                                }
                            }
                        }
                    }
                }
            }
        }
        RowLayout {
            objectName: "teachActionRow"; Layout.fillWidth: true; Layout.preferredHeight: 32; spacing: card.narrow ? 4 : 8
            CollectButton { objectName: "useCurrentPoseButton"; text: "使用当前坐标"; small: true; leftPadding: 8; rightPadding: 8; enabled: card.poseReady && !card.locked && !card.jogHeld; onClicked: card.useCurrent() }
            CollectButton { objectName: "importCoordinatesButton"; text: "导入 JSON"; small: true; leftPadding: 8; rightPadding: 8; enabled: !card.locked && !card.jogHeld && !card.controller.coordinatesImporting; onClicked: card.controller.chooseTcpCoordinateFile() }
            Item { Layout.fillWidth: true; Layout.minimumWidth: 0 }
            CollectButton { objectName: "teachStopButton"; text: card.controller.motionFault ? "重试停止" : "停止"; small: true; danger: true; leftPadding: 8; rightPadding: 8; enabled: card.locked || card.jogHeld; onClicked: { card.stopJog(); card.controller.cancelCollectionOperation() } }
            CollectButton {
                objectName: "manualMoveButton"; text: "移动到输入坐标"; small: true; primary: true; leftPadding: 8; rightPadding: 8
                enabled: !card.locked && !card.jogHeld && card.targetDraft && card.parsed.valid && card.poseReady && card.speedsValid
                focusPolicy: Qt.NoFocus
                Keys.onReturnPressed: event.accepted = true
                Keys.onEnterPressed: event.accepted = true
                Keys.onSpacePressed: event.accepted = true
                onClicked: card.moveTarget()
            }
        }
    }
}
