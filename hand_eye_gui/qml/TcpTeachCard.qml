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
    property bool tcpReady: false
    signal savePoseRequested()
    property bool targetDraft: false
    property var draftTexts: []
    property string feedback: ""
    property string lastSelectedTcp: ""
    property int heldAxis: -1
    property int heldSign: 0
    property var lease: 0
    readonly property bool jogHeld: heldAxis >= 0
    readonly property bool narrow: width < 680
    readonly property bool wide: width >= 800
    readonly property bool tcpSelectorActive: tcpSelector.activeFocus || tcpSelector.popup.visible
    readonly property bool poseReady: controller.robotConnected && controller.selectedTcp !== "" && controller.tcpPoseValid && controller.tcpPoseFresh && controller.tcpPoseTool === controller.selectedTcp && actualValues.length === 6 && actualValues.every(function(v) { return isFinite(Number(v)) })
    readonly property var actualValues: controller.tcpDisplayPoseValues
    readonly property string targetText: draftTexts.join(",")
    readonly property var parsed: controller.parseTcpTarget(targetText)
    readonly property bool speedsValid: linearSpeed.acceptableInput && angularSpeed.acceptableInput && Number(linearSpeed.text) > 0 && Number(angularSpeed.text) > 0
    readonly property bool canJog: poseReady && !targetDraft && !locked && !jogHeld && speedsValid && windowReady && pageVisible
    readonly property string reason: controller.motionFault ? "停止尚未确认，请重试停止" : locked ? controller.manualMoveProgress || "操作进行中" : !controller.robotConnected ? "连接机器人后可示教" : !controller.selectedTcp ? "请选择 TCP 工具" : !poseReady ? controller.tcpPoseStatus || "位姿已过期，等待新数据" : !speedsValid ? "请填写有效的手动速度" : targetDraft ? parsed.valid ? "目标待执行 · 长按微调已暂停" : "请输入六个有效目标数值" : "长按 ± 250 ms 开始微调，松开停止 · BASE"
    padding: 8
    implicitWidth: 480
    implicitHeight: 208
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
    function applyLinearPreset(speed) {
        if (locked || jogHeld) return
        linearSpeed.text = String(speed)
        if (card.speedsValid) controller.setManualSpeeds(Number(speed), Number(angularSpeed.text))
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
        anchors.fill: parent; spacing: 5
        // Row 1: Unified TCP selector + Save + Speed & presets + Orientation mode + Live/Draft badge
        RowLayout {
            objectName: "teachSpeedRow"; Layout.fillWidth: true; Layout.preferredHeight: 30; spacing: card.narrow ? 4 : 6
            TcpSelector {
                id: tcpSelector; objectName: "tcpSelector"; controller: card.controller
                Layout.preferredWidth: card.narrow ? 86 : 122; Layout.preferredHeight: 30; implicitHeight: 30
                enabled: !card.locked && !card.jogHeld && card.controller.robotConnected && card.controller.tcpNames.length > 0
            }
            CollectButton {
                objectName: "refreshTcpButton"; text: card.narrow ? "↻" : "刷新"; small: true; quiet: true
                Layout.preferredHeight: 30; implicitHeight: 30; leftPadding: card.narrow ? 6 : 8; rightPadding: card.narrow ? 6 : 8
                enabled: !card.locked && !card.jogHeld && card.controller.robotConnected
                ToolTip.visible: hovered; ToolTip.text: "刷新 TCP 工具列表"
                onClicked: card.controller.refreshTcpTools()
            }
            CollectButton {
                objectName: "savePoseButton"; text: "保存 TCP"; small: true
                Layout.preferredHeight: 30; implicitHeight: 30; leftPadding: card.narrow ? 6 : 10; rightPadding: card.narrow ? 6 : 10
                enabled: !card.locked && !card.jogHeld && card.tcpReady
                onClicked: card.savePoseRequested()
            }
            Rectangle { visible: !card.narrow; Layout.preferredWidth: 1; Layout.preferredHeight: 16; color: Theme.hairline }
            HmiText { visible: !card.narrow; text: "平移"; font.pixelSize: Theme.meta }
            CollectField {
                id: linearSpeed; objectName: "manualLinearSpeedInput"
                Layout.preferredWidth: card.narrow ? 42 : 52; Layout.preferredHeight: 30; implicitHeight: 30
                font.pixelSize: Theme.meta; leftPadding: 4; rightPadding: 4; horizontalAlignment: TextInput.AlignHCenter
                text: String(card.controller.manualLinearSpeedMmS); enabled: !card.locked && !card.jogHeld
                validator: DoubleValidator { bottom: 1; top: 1000; decimals: 2; locale: "C" }
                invalid: !acceptableInput
                onEditingFinished: if (card.speedsValid) card.controller.setManualSpeeds(Number(text), Number(angularSpeed.text))
            }
            HmiText { text: "mm/s"; font.pixelSize: Theme.micro; color: Theme.ink3 }
            // Quick speed presets (visible on spacious screens)
            Rectangle {
                visible: card.wide
                Layout.preferredHeight: 26; implicitWidth: presetRow.implicitWidth + 6
                radius: 6; color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                Row {
                    id: presetRow; anchors.centerIn: parent; spacing: 2
                    Repeater {
                        model: [{label: "慢", spd: 2}, {label: "中", spd: 10}, {label: "快", spd: 50}]
                        Rectangle {
                            property bool active: Number(linearSpeed.text) === modelData.spd
                            width: 24; height: 20; radius: 4
                            color: active ? Theme.accentSoft : presetMouse.containsMouse ? Theme.raisedHover : "transparent"
                            border.width: active ? 1 : 0; border.color: Theme.accentLine
                            HmiText { anchors.centerIn: parent; text: modelData.label; font.pixelSize: Theme.micro; color: parent.active ? Theme.ink : Theme.ink2 }
                            MouseArea {
                                id: presetMouse; anchors.fill: parent; hoverEnabled: true
                                enabled: !card.locked && !card.jogHeld
                                onClicked: card.applyLinearPreset(modelData.spd)
                            }
                            ToolTip.visible: presetMouse.containsMouse; ToolTip.text: "设为 " + modelData.spd + " mm/s"
                        }
                    }
                }
            }
            HmiText { visible: card.width >= 740; text: "角步进"; font.pixelSize: Theme.meta }
            CollectField {
                id: angularSpeed; objectName: "manualAngularSpeedInput"
                Layout.preferredWidth: card.narrow ? 36 : 46; Layout.preferredHeight: 30; implicitHeight: 30
                font.pixelSize: Theme.meta; leftPadding: 4; rightPadding: 4; horizontalAlignment: TextInput.AlignHCenter
                text: String(card.controller.manualAngularSpeedDegS); enabled: !card.locked && !card.jogHeld
                validator: DoubleValidator { bottom: 0.1; top: 180; decimals: 2; locale: "C" }
                invalid: !acceptableInput
                ToolTip.visible: hovered; ToolTip.text: "用于长按旋转微调；移动到输入坐标时使用平移速度，由机器人规划姿态变化"
                onEditingFinished: if (card.speedsValid) card.controller.setManualSpeeds(Number(linearSpeed.text), Number(text))
            }
            HmiText { text: "°/s"; font.pixelSize: Theme.micro; color: Theme.ink3 }
            Item { Layout.fillWidth: true; Layout.minimumWidth: 0 }
            CompactSelector {
                objectName: "orientationModeSelector"
                Layout.preferredWidth: card.narrow ? 82 : 104; Layout.preferredHeight: 30; implicitHeight: 30
                font.family: Theme.sans; font.pixelSize: Theme.meta
                model: ["轴角", "欧拉角 ZYX"]; currentIndex: card.controller.orientationMode === "euler" ? 1 : 0
                enabled: !card.locked && !card.jogHeld && (!card.targetDraft || card.parsed.valid)
                onActivated: {
                    card.changeMode(currentIndex === 1 ? "euler" : "axis")
                    // A rejected conversion does not change the controller, so
                    // its NOTIFY cannot restore a user-modified ComboBox index.
                    currentIndex = Qt.binding(function() { return card.controller.orientationMode === "euler" ? 1 : 0 })
                }
            }
            Rectangle {
                visible: !card.narrow
                Layout.preferredHeight: 26; implicitWidth: modePillText.implicitWidth + 16; radius: 13
                color: card.targetDraft ? Theme.warningSoft : card.poseReady ? Theme.okSoft : Theme.raised
                border.width: 1
                border.color: card.targetDraft ? Theme.warningLine : card.poseReady ? Theme.okLine : Theme.hairline
                HmiText {
                    id: modePillText; anchors.centerIn: parent; font.pixelSize: Theme.micro; font.weight: Font.DemiBold
                    text: card.targetDraft ? "草稿待执行" : "实时 · BASE"
                    color: card.targetDraft ? Theme.warningText : card.poseReady ? Theme.okText : Theme.ink3
                }
            }
        }

        // Row 2 & 3: Full-width 3x2 Axis Cards with RGB accent bars, unit labels, inline actual readout, and 250ms hold fill
        GridLayout {
            objectName: "teachAxes"; columns: 3; Layout.fillWidth: true; Layout.fillHeight: true
            columnSpacing: card.narrow ? 5 : 8; rowSpacing: 5
            Repeater {
                model: [
                    {name: "X", unit: "mm"}, {name: "Y", unit: "mm"}, {name: "Z", unit: "mm"},
                    {name: "Rx", unit: "°"}, {name: "Ry", unit: "°"}, {name: "Rz", unit: "°"}
                ]
                Rectangle {
                    id: axisCard; property int axis: index
                    Layout.fillWidth: true; Layout.minimumWidth: 0; Layout.preferredHeight: 58; Layout.fillHeight: true
                    radius: Theme.radiusControl
                    color: Theme.surfaceAlt
                    border.width: 1
                    border.color: card.targetDraft ? Theme.warningLine : Theme.hairline
                    clip: true
                    // Left 3px semantic axis color bar
                    Rectangle {
                        anchors.left: parent.left; anchors.top: parent.top; anchors.bottom: parent.bottom
                        anchors.topMargin: 3; anchors.bottomMargin: 3
                        width: 3; radius: 1.5
                        color: Theme.axisColors[axisCard.axis]
                    }
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 7; anchors.rightMargin: 4; anchors.topMargin: 2; anchors.bottomMargin: 2
                        spacing: 1
                        RowLayout {
                            Layout.fillWidth: true; Layout.preferredHeight: 13; spacing: 4
                            HmiText { text: modelData.name; color: Theme.ink; font.family: Theme.mono; font.pixelSize: Theme.micro; font.weight: Font.DemiBold }
                            HmiText { text: modelData.unit; color: Theme.ink3; font.family: Theme.mono; font.pixelSize: Theme.micro }
                            Item { Layout.fillWidth: true }
                            HmiText {
                                visible: card.targetDraft && card.poseReady
                                text: "实际 " + (card.poseReady ? card.format(card.actualValues[axisCard.axis]) : "—")
                                color: Theme.ink3; font.family: Theme.mono; font.pixelSize: Theme.micro
                                elide: Text.ElideRight; wrapMode: Text.NoWrap
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true; Layout.preferredHeight: 40; spacing: card.narrow ? 2 : 4
                            Repeater {
                                model: 3
                                Loader {
                                    property int slot: index
                                    Layout.fillWidth: slot === 1
                                    Layout.preferredWidth: slot === 1 ? -1 : card.narrow ? 26 : 32
                                    Layout.minimumWidth: slot === 1 ? 34 : card.narrow ? 26 : 32
                                    Layout.preferredHeight: 40
                                    sourceComponent: slot === 1 ? valueField : jogButton
                                    Component {
                                        id: valueField
                                        CollectField {
                                            objectName: "tcpAxis" + axisCard.axis
                                            implicitHeight: 40
                                            text: card.targetDraft ? (card.draftTexts[axisCard.axis] || "") : card.poseReady ? card.format(card.actualValues[axisCard.axis]) : "—"
                                            horizontalAlignment: TextInput.AlignHCenter
                                            font.pixelSize: card.narrow ? Theme.meta : Theme.body
                                            leftPadding: 2; rightPadding: 2
                                            enabled: !card.locked && !card.jogHeld && card.poseReady
                                            draft: card.targetDraft
                                            invalid: card.targetDraft && (text.trim() === "" || !isFinite(Number(text)))
                                            onTextEdited: card.beginEdit(axisCard.axis, text)
                                            Keys.onReturnPressed: event.accepted = true
                                            Keys.onEnterPressed: event.accepted = true
                                            Keys.onSpacePressed: event.accepted = true
                                            ToolTip.visible: hovered
                                            ToolTip.text: card.targetDraft ? "输入目标 · " + (axisCard.axis < 3 ? "mm" : "°") : "当前坐标 · 编辑后冻结六个目标值"
                                        }
                                    }
                                    Component {
                                        id: jogButton
                                        Rectangle {
                                            id: jog; objectName: "jog" + axisCard.axis + (slot === 0 ? "Minus" : "Plus")
                                            property bool ownHold: card.heldAxis === axisCard.axis && card.heldSign === (slot === 0 ? -1 : 1)
                                            property bool joggingNow: ownHold && card.lease > 0
                                            enabled: card.canJog || ownHold
                                            opacity: enabled ? 1 : 0.38; radius: Theme.radiusControl; clip: true
                                            color: joggingNow ? Theme.accentSoft : jogArea.containsMouse ? Theme.raisedHover : Theme.raised
                                            border.width: 1
                                            border.color: ownHold ? Theme.accent : jogArea.containsMouse ? Theme.hairlineStrong : Theme.hairline
                                            // 250ms press-and-hold charge fill feedback
                                            Rectangle {
                                                id: chargeFill
                                                anchors.left: parent.left; anchors.top: parent.top; anchors.bottom: parent.bottom
                                                width: jog.joggingNow ? parent.width : 0
                                                color: Theme.accentSoft
                                                visible: jog.ownHold
                                                PropertyAnimation {
                                                    id: chargeAnim
                                                    target: chargeFill; property: "width"
                                                    from: 0; to: jog.width; duration: 250
                                                }
                                            }
                                            onOwnHoldChanged: {
                                                if (ownHold) chargeAnim.restart()
                                                else { chargeAnim.stop(); chargeFill.width = 0 }
                                            }
                                            HmiText { anchors.centerIn: parent; text: slot === 0 ? "−" : "+"; font.pixelSize: 18; color: jog.ownHold ? Theme.white : Theme.ink }
                                            MouseArea {
                                                id: jogArea; objectName: "jogSurface" + axisCard.axis + (slot === 0 ? "Minus" : "Plus"); anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.LeftButton
                                                onPressed: card.pressJog(axisCard.axis, slot === 0 ? -1 : 1)
                                                onReleased: if (jog.ownHold) card.stopJog()
                                                onCanceled: if (jog.ownHold) card.stopJog()
                                                onExited: if (jog.ownHold) card.stopJog()
                                            }
                                            ToolTip.visible: jogArea.containsMouse; ToolTip.text: card.canJog ? "长按 250ms 微调，松开立即停止" : card.reason
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Row 4: Action bar + inline actual pose summary when editing
        RowLayout {
            objectName: "teachActionRow"; Layout.fillWidth: true; Layout.preferredHeight: 30; spacing: card.narrow ? 4 : 6
            CollectButton {
                objectName: "copyPoseButton"; text: "复制当前"; small: true
                Layout.preferredHeight: 30; implicitHeight: 30; leftPadding: 8; rightPadding: 8
                enabled: card.poseReady
                onClicked: card.controller.copyCurrentTcpCoordinates(card.controller.orientationMode)
            }
            CollectButton {
                objectName: "useCurrentPoseButton"; text: card.narrow ? "使用当前" : "使用当前坐标"; small: true
                warningTone: card.targetDraft
                Layout.preferredHeight: 30; implicitHeight: 30; leftPadding: 8; rightPadding: 8
                enabled: card.poseReady && !card.locked && !card.jogHeld
                onClicked: card.useCurrent()
            }
            CollectButton {
                objectName: "importCoordinatesButton"; text: "导入 JSON"; small: true
                Layout.preferredHeight: 30; implicitHeight: 30; leftPadding: 8; rightPadding: 8
                enabled: !card.locked && !card.jogHeld && !card.controller.coordinatesImporting
                onClicked: card.controller.chooseTcpCoordinateFile()
            }
            HmiText {
                objectName: "actualPoseWhileEditing"
                visible: card.targetDraft
                Layout.fillWidth: true; Layout.minimumWidth: 40
                font.pixelSize: Theme.micro; font.family: Theme.mono; color: Theme.ink2
                elide: Text.ElideRight; wrapMode: Text.NoWrap
                text: "当前(" + (card.controller.orientationMode === "euler" ? "ZYX" : "轴角") + ") " +
                      (card.poseReady ? card.actualValues.map(card.format).join(", ") : "等待位姿")
            }
            Item { visible: !card.targetDraft; Layout.fillWidth: true; Layout.minimumWidth: 0 }
            CollectButton {
                objectName: "teachStopButton"
                text: card.controller.motionFault ? "重试停止" : "停止"; small: true
                danger: true; solidDanger: card.locked || card.jogHeld || card.controller.motionFault
                Layout.preferredHeight: 30; implicitHeight: 30; leftPadding: 10; rightPadding: 10
                enabled: card.locked || card.jogHeld
                onClicked: { card.stopJog(); card.controller.cancelCollectionOperation() }
            }
            CollectButton {
                objectName: "manualMoveButton"; text: "移动到输入坐标"; small: true; primary: true
                Layout.preferredHeight: 30; implicitHeight: 30; leftPadding: 10; rightPadding: 10
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
