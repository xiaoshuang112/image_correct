import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "."
ApplicationWindow {
    id: root
    visible: true; width: 1280; height: 850; minimumWidth: 760; minimumHeight: 600
    title: "COLLECT · 图像采集 · " + appVersion
    color: Theme.bg; font.family: Theme.sans; font.pixelSize: Theme.body
    property var controller: collectionController
    property string page: "main"
    property string captureMode: "single"
    property bool showCrosshair: true
    readonly property bool ninePointMode: captureMode === "nine"
    readonly property bool configuredPointMode: captureMode === "configured"
    readonly property bool dualMode: controller.dualCameraMode
    readonly property int activeCam: dualMode ? controller.activePreviewCamera : 1
    property var configuredPoints: ({count: 0, error: ""})
    property bool gridAnglesValid: false
    property bool poseEditorOpen: false
    readonly property bool busy: controller.collectionBusy || controller.manualMoveActive || controller.jogActive || controller.motionFault
    readonly property bool anyCameraOpen: controller.cameraOpened || controller.camera2Opened
    readonly property bool camerasHealthy: dualMode ? (controller.cameraOpened && controller.camera2Opened && controller.camera1Healthy && controller.camera2Healthy)
                                                    : (controller.cameraOpened && (!controller.previewActive || controller.camera1Healthy))
    readonly property bool ready: dualMode ? controller.dualCameraReady && !busy
                                           : controller.cameraOpened && controller.previewActive && !busy
    readonly property bool compact: width < 960 || height < 680
    readonly property bool tcpReady: controller.robotConnected && controller.selectedTcp !== ""
    readonly property real diagonalOffsetMm: offsetX.acceptableInput && offsetY.acceptableInput ? Math.sqrt(Math.pow(Number(offsetX.text), 2) + Math.pow(Number(offsetY.text), 2)) : NaN
    readonly property bool gridOffsetValid: offsetX.acceptableInput && offsetY.acceptableInput && Number(offsetX.text) >= 0.1 && Number(offsetY.text) >= 0.1 && diagonalOffsetMm <= 50
    readonly property bool editing: activeFocusItem !== null && (activeFocusItem.cursorPosition !== undefined || activeFocusItem.hasOwnProperty("currentIndex"))
    readonly property bool shortcutReady: page === "main" && ready && captureMode === "single" && !editing && !poseEditorOpen && !teach.targetDraft && !teach.jogHeld && !teach.tcpSelectorActive
    readonly property bool switchShortcutReady: page === "main" && dualMode && !editing && !poseEditorOpen && !teach.targetDraft && !teach.jogHeld && !teach.tcpSelectorActive
    readonly property bool activeUndistortEnabled: (dualMode && activeCam === 2) ? controller.undistortion2Enabled : controller.undistortionEnabled
    readonly property bool activeUndistortReady: (dualMode && activeCam === 2)
        ? (controller.camera2Opened && (controller.undistortion2Ready || controller.undistortion2Enabled))
        : (controller.cameraOpened && (controller.undistortionReady || controller.undistortionEnabled))
    readonly property string activeUndistortStatus: (dualMode && activeCam === 2) ? controller.undistortion2Status : controller.undistortionStatus
    readonly property bool crosshairSupported: !root.dualMode || root.activeCam === 1
    readonly property bool crosshairVisibleOnPreview: root.showCrosshair && root.crosshairSupported
    readonly property string progressText: controller.ninePointProgress || ""
    readonly property var pointStageMatch: /第\s*(\d+)\s*\/\s*(\d+)\s*点/.exec(progressText)
    readonly property var savedStageMatch: /已保存\s*(\d+)\s*\/\s*(\d+)/.exec(progressText)
    readonly property int progressTotal: pointStageMatch ? Number(pointStageMatch[2]) : savedStageMatch ? Number(savedStageMatch[2]) : 0
    readonly property int activeStep: pointStageMatch ? Number(pointStageMatch[1]) : 0
    readonly property int completedSteps: pointStageMatch ? Math.max(0, Number(pointStageMatch[1]) - 1)
                                        : savedStageMatch ? Number(savedStageMatch[1])
                                        : 0
    readonly property bool progressFinished: progressText.indexOf("完成") >= 0 && progressTotal > 0 && completedSteps >= progressTotal && !busy
    readonly property real parsedProgressRatio: {
        if (progressFinished) return 1.0
        if (pointStageMatch && progressTotal > 0) {
            var capturing = progressText.indexOf("采集") >= 0
            var units = Math.max(0, activeStep - (capturing ? 0.5 : 1.0))
            return Math.min(0.95, Math.max(0.04, units / (progressTotal + 1)))
        }
        if (savedStageMatch && progressTotal > 0) {
            return Math.min(0.95, Math.max(0, completedSteps / (progressTotal + 1)))
        }
        return -1
    }
    property real latchedProgressRatio: 0
    onParsedProgressRatioChanged: if (parsedProgressRatio >= 0) latchedProgressRatio = parsedProgressRatio
    onProgressTextChanged: if (progressText === "" || progressText.indexOf("基准位稳定") >= 0) latchedProgressRatio = 0
    readonly property real progressRatio: {
        if (parsedProgressRatio >= 0) return parsedProgressRatio
        if (busy) return Math.max(0.08, latchedProgressRatio)
        return 0
    }
    property string frameMetadata: ""
    property bool hasFrame: false
    property int seenCount: controller.savedImageCount
    property bool historyReady: false
    readonly property bool interactionLocked: busy || teach.jogHeld
    function takePhoto() { if (ready && page === "main" && !teach.jogHeld) controller.capturePhoto() }
    function formatPlanNumber(value) {
        var scaled = Math.round(Number(value) * 100) / 100
        if (!isFinite(scaled)) return ""
        return String(scaled)
    }
    function ninePointAngles() {
        if (!angleInputs || angleInputs.count !== 9) return []
        var values = []
        for (var i = 0; i < 9; ++i) {
            var field = angleInputs.itemAt(i)
            if (!field || !field.acceptableInput) return []
            values.push(Number(field.text))
        }
        return values
    }
    function resetNinePointAngles() {
        if (!angleInputs || angleInputs.count !== 9 || interactionLocked) return
        var defaults = controller.ninePointDefaultYawOffsets || [0, 15, -15, -20, 0, 20, 15, -15, 0]
        for (var i = 0; i < 9 && i < defaults.length; ++i) {
            var field = angleInputs.itemAt(i)
            if (field) field.text = (defaults[i] > 0 ? "+" : "") + defaults[i]
        }
        gridAnglesValid = ninePointAngles().length === 9
    }
    function syncHistory() {
        var count = controller.savedImageCount
        if (historyReady && count > seenCount) { shutterFlash.restart() }
        else if (count < seenCount) { shutterFlash.stop(); flash.opacity = 0 }
        seenCount = count; historyReady = true
    }
    function savePose() { pointName.text = ""; poseEditorOpen = true; poseDialog.open(); pointName.forceActiveFocus() }
    function confirmPose() { if (tcpReady && !busy) controller.saveTcpPose(pointName.text) }
    onPageChanged: teach.stopJog()
    Binding {
        target: root.controller; property: "previewVisible"
        value: root.page === "main" && root.visible && root.visibility !== Window.Minimized
    }
    Binding {
        target: root.controller; property: "previewDisplaySize"
        value: Qt.size(Math.max(1, Math.round(cameraImage.width * root.screen.devicePixelRatio)),
                       Math.max(1, Math.round(cameraImage.height * root.screen.devicePixelRatio)))
    }
    function updateFrameMetadataText() {
        const cam = root.dualMode ? root.controller.activePreviewCamera : 1
        if (!root.dualMode) {
            root.frameMetadata = root.controller.cameraModel + " · " + root.controller.cameraActualWidth + " × " + root.controller.cameraActualHeight + " · " + root.controller.cameraActualExposureUs + " μs" + (root.controller.undistortionEnabled ? " · 去畸变" : "")
        } else if (cam === 2) {
            root.frameMetadata = "相机 2 · " + root.controller.camera2Model + " · " + root.controller.camera2ActualWidth + " × " + root.controller.camera2ActualHeight + " · " + root.controller.camera2ActualExposureUs + " μs" + (root.controller.undistortion2Enabled ? " · 去畸变" : "")
        } else {
            root.frameMetadata = "相机 1 · " + root.controller.cameraModel + " · " + root.controller.cameraActualWidth + " × " + root.controller.cameraActualHeight + " · " + root.controller.cameraActualExposureUs + " μs" + (root.controller.undistortionEnabled ? " · 去畸变" : "")
        }
    }
    onControllerChanged: { historyReady = false; seenCount = controller.savedImageCount; shutterFlash.stop(); flash.opacity = 0; historyReady = true; hasFrame = false; frameMetadata = ""; latchedProgressRatio = 0 }
    onClosing: function(event) {
        teach.stopJog()
        if (busy) { event.accepted = false; controller.cancelCollectionOperation() }
        else if (page === "settings" && settings.dirty) { event.accepted = false; settings.requestLeave() }
    }
    Component.onCompleted: { seenCount = controller.savedImageCount; historyReady = true }
    Shortcut { sequence: "Escape"; onActivated: { teach.stopJog(); if (root.busy) root.controller.cancelCollectionOperation() } }
    Shortcut { sequence: "Space"; autoRepeat: false; enabled: root.shortcutReady; onActivated: root.takePhoto() }
    Shortcut { sequence: "S"; autoRepeat: false; enabled: root.shortcutReady; onActivated: root.takePhoto() }
    Shortcut { sequence: "Ctrl+S"; autoRepeat: false; enabled: root.shortcutReady; onActivated: root.takePhoto() }
    Shortcut { sequence: "Tab"; autoRepeat: false; enabled: root.switchShortcutReady; onActivated: root.controller.togglePreviewCamera() }
    Connections {
        target: root.controller
        function onCollectionChanged() {
            root.syncHistory()
            if (root.hasFrame) root.updateFrameMetadataText()
        }
        function onFrameUpdated() {
            if (root.page !== "main") return
            cameraImage.source = "image://camera/current#" + Date.now()
            root.hasFrame = true
            root.updateFrameMetadataText()
        }
        function onFrameCleared() {
            root.hasFrame = false
            root.frameMetadata = ""
        }
        function onCameraOpenedChanged() {
            if (!root.controller.cameraOpened && !root.controller.camera2Opened) {
                root.hasFrame = false
                root.frameMetadata = ""
            }
        }
        function onDualCameraModeChanged() {
            root.hasFrame = false
            root.frameMetadata = ""
        }
        function onActivePreviewCameraChanged() {
            if (root.hasFrame) root.updateFrameMetadataText()
        }
        function onStatusMessageChanged() {
            if (root.poseEditorOpen && root.controller.statusMessage.indexOf("TCP 坐标已保存：") === 0) poseDialog.close()
        }
        function onRobotConnectedChanged() { if (!root.controller.robotConnected) poseDialog.close() }
    }

    // ==================== MAIN PAGE ====================
    Item {
        anchors.fill: parent; visible: root.page === "main"
        ColumnLayout {
            anchors.fill: parent; spacing: 0
            // TOP HEADER BAR
            Rectangle {
                color: Theme.surface; Layout.fillWidth: true; Layout.preferredHeight: 54
                Rectangle { anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom; height: 1; color: Theme.hairline }
                RowLayout {
                    anchors.fill: parent; anchors.leftMargin: 14; anchors.rightMargin: 14; spacing: 10
                    // Brand Emblem + Name
                    Rectangle {
                        Layout.preferredWidth: 32; Layout.preferredHeight: 32; radius: 8
                        color: Theme.accentSoft; border.width: 1; border.color: Theme.accentLine
                        CollectIcon { anchors.centerIn: parent; width: 20; height: 20; ink: Theme.accent }
                    }
                    HmiText {
                        visible: root.width >= 880; text: "COLLECT"; color: Theme.ink
                        font.pixelSize: Theme.title; font.weight: Font.Bold; font.letterSpacing: 1
                    }
                    // Global Status Pill
                    Rectangle {
                        Layout.fillWidth: true; Layout.minimumWidth: 80; Layout.preferredHeight: 30; radius: 15
                        readonly property color tone: root.controller.motionFault || /失败|异常|故障|意外变化|过慢|超时/.test(root.controller.statusMessage) ? Theme.danger
                                                    : root.busy ? Theme.accent
                                                    : root.anyCameraOpen ? Theme.ok
                                                    : Theme.warning
                        color: Qt.rgba(tone.r, tone.g, tone.b, 0.12)
                        border.width: 1; border.color: Qt.rgba(tone.r, tone.g, tone.b, 0.36)
                        RowLayout {
                            anchors.fill: parent; anchors.leftMargin: 10; anchors.rightMargin: 12; spacing: 7
                            Rectangle { width: 7; height: 7; radius: 3.5; color: parent.parent.tone }
                            HmiText {
                                objectName: "globalStatus"; Layout.fillWidth: true
                                text: root.controller.statusMessage; color: parent.parent.tone
                                font.pixelSize: Theme.meta; elide: Text.ElideRight; wrapMode: Text.NoWrap; verticalAlignment: Text.AlignVCenter
                            }
                        }
                    }
                    // Single / Dual Camera Segmented Trough
                    Rectangle {
                        Layout.preferredHeight: 34; implicitWidth: camModeRow.implicitWidth + 6
                        radius: 8; color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                        RowLayout {
                            id: camModeRow; anchors.centerIn: parent; spacing: 2
                            CollectButton {
                                objectName: "singleCameraModeButton"; small: true; segmented: true; text: "单相机"
                                implicitHeight: 28; leftPadding: 10; rightPadding: 10
                                selected: !root.dualMode
                                enabled: !root.interactionLocked && !root.anyCameraOpen
                                ToolTip.visible: hovered && root.anyCameraOpen; ToolTip.text: "请先断开相机再切换单/双相机模式"
                                onClicked: root.controller.dualCameraMode = false
                            }
                            CollectButton {
                                objectName: "dualCameraModeButton"; small: true; segmented: true; text: "双相机"
                                implicitHeight: 28; leftPadding: 10; rightPadding: 10
                                selected: root.dualMode
                                enabled: !root.interactionLocked && !root.anyCameraOpen
                                ToolTip.visible: hovered && root.anyCameraOpen; ToolTip.text: "请先断开相机再切换单/双相机模式"
                                onClicked: root.controller.dualCameraMode = true
                            }
                        }
                    }
                    // Device Connection Status Pills
                    CollectButton {
                        objectName: "openCameraButton"; small: true; pill: true; statusDot: true
                        implicitHeight: 32; leftPadding: 12; rightPadding: 12
                        positive: root.anyCameraOpen && root.camerasHealthy
                        danger: root.anyCameraOpen && !root.camerasHealthy
                        dotColor: !root.controller.cameraOpened ? Theme.ink3 : root.controller.camera1Healthy ? Theme.ok : Theme.danger
                        dot2Visible: root.dualMode
                        dot2Color: !root.controller.camera2Opened ? Theme.ink3 : root.controller.camera2Healthy ? Theme.ok : Theme.danger
                        text: root.anyCameraOpen ? "相机已连" : "连接相机"
                        enabled: !root.interactionLocked
                        ToolTip.visible: hovered; ToolTip.text: root.anyCameraOpen ? "点击断开相机连接" : "点击连接相机并开启预览"
                        onClicked: {
                            if (root.anyCameraOpen) {
                                root.controller.closeCamera()
                            } else {
                                root.controller.openCamera()
                                const linksReady = root.dualMode
                                    ? root.controller.cameraOpened && root.controller.camera2Opened
                                    : root.controller.cameraOpened
                                if (linksReady) root.controller.startPreview()
                            }
                        }
                    }
                    CollectButton {
                        objectName: "connectRobotButton"; small: true; pill: true; statusDot: true
                        implicitHeight: 32; leftPadding: 12; rightPadding: 12
                        positive: root.controller.robotConnected
                        dotColor: root.controller.robotConnected ? Theme.ok : Theme.ink3
                        text: root.controller.robotConnected ? "机器人已连" : "连接机器人"
                        enabled: !root.interactionLocked
                        ToolTip.visible: hovered; ToolTip.text: root.controller.robotConnected ? "点击断开机器人连接" : "点击连接机器人控制器"
                        onClicked: root.controller.robotConnected ? root.controller.disconnectRobot() : root.controller.connectRobot()
                    }
                    CollectButton {
                        objectName: "settingsButton"; small: true; text: "设置"
                        implicitHeight: 32; leftPadding: 12; rightPadding: 12
                        enabled: !root.interactionLocked
                        onClicked: { settings.reload(); root.page = "settings" }
                    }
                }
            }

            // MAIN WORKSPACE DECK
            RowLayout {
                objectName: "mainDeck"
                Layout.fillWidth: true; Layout.fillHeight: true
                Layout.margins: root.compact ? 10 : 14
                spacing: root.compact ? 10 : 12

                // LEFT MISSION SIDEBAR (Capture Mode + 9-Point Rz Matrix + Primary CTA + Progress + Stat Cards)
                CollectCard {
                    objectName: "mainControls"
                    Layout.preferredWidth: root.width < 920 ? 222 : 284
                    Layout.fillHeight: true
                    padding: root.compact ? 8 : 12
                    ColumnLayout {
                        anchors.fill: parent; spacing: root.compact ? 6 : 10
                        // Mode Segmented Switcher
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 34; radius: 8
                            color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                            RowLayout {
                                anchors.fill: parent; anchors.margins: 3; spacing: 2
                                CollectButton { objectName: "singleModeButton"; text: "单次"; small: true; segmented: true; implicitHeight: 28; selected: root.captureMode === "single"; Layout.fillWidth: true; enabled: !root.interactionLocked; onClicked: root.captureMode = "single" }
                                CollectButton { objectName: "ninePointModeButton"; text: "九点"; small: true; segmented: true; implicitHeight: 28; selected: root.ninePointMode; Layout.fillWidth: true; enabled: !root.interactionLocked; onClicked: root.captureMode = "nine" }
                                CollectButton {
                                    objectName: "configuredModeButton"; text: "按配置"; small: true; segmented: true; implicitHeight: 28; selected: root.configuredPointMode
                                    Layout.fillWidth: true; enabled: !root.interactionLocked
                                    onClicked: {
                                        root.captureMode = "configured"
                                        root.configuredPoints = root.controller.configuredPointSummary()
                                    }
                                }
                            }
                        }

                        // SINGLE MODE BRIEF PANEL
                        Rectangle {
                            visible: root.captureMode === "single"
                            Layout.fillWidth: true
                            implicitHeight: singleInfoCol.implicitHeight + 18
                            radius: Theme.radiusControl; color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                            ColumnLayout {
                                id: singleInfoCol
                                anchors.fill: parent; anchors.margins: 9; spacing: 6
                                RowLayout {
                                    Layout.fillWidth: true
                                    HmiText { text: "融合曝光序列 (μs)"; font.pixelSize: Theme.meta; color: Theme.ink2 }
                                    Item { Layout.fillWidth: true }
                                    HmiText {
                                        readonly property var expVals: ((root.dualMode && root.activeCam === 2 ? root.controller.exposure2List : root.controller.exposureList) || "").split(/[,，;\s]+/).filter(function(v){ return v !== "" })
                                        text: expVals.length + " 档"; font.pixelSize: Theme.micro; font.family: Theme.mono; color: Theme.accent
                                    }
                                }
                                Row {
                                    Layout.fillWidth: true; spacing: 4; clip: true
                                    Repeater {
                                        model: ((root.dualMode && root.activeCam === 2 ? root.controller.exposure2List : root.controller.exposureList) || "").split(/[,，;\s]+/).filter(function(v){ return v !== "" })
                                        Rectangle {
                                            width: chipTxt.implicitWidth + 10; height: 22; radius: 4
                                            color: Theme.raised; border.width: 1; border.color: Theme.hairline
                                            HmiText { id: chipTxt; anchors.centerIn: parent; text: modelData; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink }
                                        }
                                    }
                                }
                                HmiText {
                                    Layout.fillWidth: true; font.pixelSize: Theme.micro; color: Theme.ink3
                                    text: "Space / S 快捷拍照 · 同步记录法兰 6D 位姿"
                                }
                            }
                        }

                        // NINE-POINT OFFSET & RZ MATRIX PANEL
                        ColumnLayout {
                            visible: root.ninePointMode; Layout.fillWidth: true; spacing: root.compact ? 4 : 7
                            RowLayout {
                                Layout.fillWidth: true; spacing: 6
                                CollectField {
                                    id: offsetX; objectName: "ninePointOffsetXInput"
                                    implicitHeight: root.compact ? 32 : 36; leftPadding: 26; rightPadding: 26
                                    font.pixelSize: Theme.meta; horizontalAlignment: TextInput.AlignHCenter
                                    text: root.formatPlanNumber(root.controller.ninePointOffsetXMm); placeholderText: "10"
                                    Layout.fillWidth: true; Layout.minimumWidth: 0; enabled: !root.interactionLocked
                                    invalid: !acceptableInput || !root.gridOffsetValid
                                    validator: DoubleValidator { bottom: 0.1; top: 50; decimals: 2; locale: "C" }
                                    HmiText { anchors.left: parent.left; anchors.leftMargin: 6; anchors.verticalCenter: parent.verticalCenter; text: "ΔX"; font.pixelSize: Theme.micro; color: Theme.ink2 }
                                    HmiText { anchors.right: parent.right; anchors.rightMargin: 6; anchors.verticalCenter: parent.verticalCenter; text: "mm"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                }
                                CollectField {
                                    id: offsetY; objectName: "ninePointOffsetYInput"
                                    implicitHeight: root.compact ? 32 : 36; leftPadding: 26; rightPadding: 26
                                    font.pixelSize: Theme.meta; horizontalAlignment: TextInput.AlignHCenter
                                    text: root.formatPlanNumber(root.controller.ninePointOffsetYMm); placeholderText: "10"
                                    Layout.fillWidth: true; Layout.minimumWidth: 0; enabled: !root.interactionLocked
                                    invalid: !acceptableInput || !root.gridOffsetValid
                                    validator: DoubleValidator { bottom: 0.1; top: 50; decimals: 2; locale: "C" }
                                    HmiText { anchors.left: parent.left; anchors.leftMargin: 6; anchors.verticalCenter: parent.verticalCenter; text: "ΔY"; font.pixelSize: Theme.micro; color: Theme.ink2 }
                                    HmiText { anchors.right: parent.right; anchors.rightMargin: 6; anchors.verticalCenter: parent.verticalCenter; text: "mm"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                }
                            }
                            RowLayout {
                                visible: !root.compact || !root.gridOffsetValid
                                Layout.fillWidth: true
                                HmiText {
                                    text: "对角距基准"; font.pixelSize: Theme.micro; color: Theme.ink3
                                    visible: root.gridOffsetValid
                                }
                                HmiText {
                                    objectName: "ninePointOffsetError"; visible: !root.gridOffsetValid; Layout.fillWidth: true
                                    text: "偏移须 0.1～50 mm，对角 ≤50 mm"; color: Theme.warning; font.pixelSize: Theme.micro
                                }
                                Item { Layout.fillWidth: root.gridOffsetValid }
                                HmiText {
                                    visible: root.gridOffsetValid && isFinite(root.diagonalOffsetMm)
                                    text: root.diagonalOffsetMm.toFixed(1) + " / 50 mm"
                                    font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.okText
                                }
                            }

                            // Explicit Rz Angle Reminder + Reset
                            RowLayout {
                                Layout.fillWidth: true
                                HmiText {
                                    text: "各点 Rz 角度 · °（±20°）"
                                    font.pixelSize: Theme.micro; font.weight: Font.DemiBold; color: Theme.ink2
                                }
                                Item { Layout.fillWidth: true }
                                HmiText {
                                    text: "恢复默认"; font.pixelSize: Theme.micro
                                    color: resetMouse.containsMouse && !root.interactionLocked ? Theme.ink : Theme.accent
                                    opacity: root.interactionLocked ? 0.4 : 1
                                    MouseArea {
                                        id: resetMouse; anchors.fill: parent; hoverEnabled: true
                                        enabled: !root.interactionLocked
                                        cursorShape: Qt.PointingHandCursor
                                        onClicked: root.resetNinePointAngles()
                                    }
                                }
                            }

                            // 3x3 Visual Matrix with -ΔX/0/+ΔX and +ΔY/0/-ΔY Coordinate Rulers
                            Rectangle {
                                objectName: "ninePointPlan"
                                Layout.fillWidth: true
                                implicitHeight: matrixCol.implicitHeight + 12
                                radius: Theme.radiusControl; color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                                ColumnLayout {
                                    id: matrixCol
                                    anchors.fill: parent; anchors.margins: 6; spacing: 3
                                    // Top X-axis ruler
                                    RowLayout {
                                        Layout.fillWidth: true; Layout.leftMargin: 24; spacing: 4
                                        HmiText { Layout.fillWidth: true; horizontalAlignment: Text.AlignHCenter; text: "−ΔX"; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                        HmiText { Layout.fillWidth: true; horizontalAlignment: Text.AlignHCenter; text: "0"; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                        HmiText { Layout.fillWidth: true; horizontalAlignment: Text.AlignHCenter; text: "+ΔX"; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                    }
                                    RowLayout {
                                        Layout.fillWidth: true; spacing: 4
                                        // Left Y-axis ruler
                                        ColumnLayout {
                                            Layout.preferredWidth: 20; Layout.preferredHeight: angleGrid.implicitHeight; spacing: 4
                                            HmiText { Layout.fillHeight: true; Layout.fillWidth: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; text: "+Y"; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                            HmiText { Layout.fillHeight: true; Layout.fillWidth: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; text: "0"; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                            HmiText { Layout.fillHeight: true; Layout.fillWidth: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; text: "−Y"; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                        }
                                        GridLayout {
                                            id: angleGrid
                                            Layout.fillWidth: true; columns: 3; columnSpacing: 4; rowSpacing: 4
                                            Repeater {
                                                id: angleInputs; model: root.controller.ninePointYawOffsets
                                                onItemAdded: root.gridAnglesValid = root.ninePointAngles().length === 9
                                                onItemRemoved: root.gridAnglesValid = root.ninePointAngles().length === 9
                                                CollectField {
                                                    id: angleField; objectName: "ninePointAngle" + index
                                                    readonly property bool stepActive: root.busy && root.progressTotal === 9 && root.activeStep === (index + 1)
                                                    readonly property bool stepDone: root.progressTotal === 9 && root.completedSteps >= (index + 1)
                                                    Layout.fillWidth: true; Layout.minimumWidth: 0; implicitHeight: root.compact ? 28 : 34
                                                    text: (modelData > 0 ? "+" : "") + modelData; enabled: !root.interactionLocked
                                                    leftPadding: 14; rightPadding: 11; font.pixelSize: root.compact ? Theme.micro : Theme.meta
                                                    horizontalAlignment: TextInput.AlignHCenter; selectByMouse: true
                                                    invalid: !acceptableInput
                                                    validator: DoubleValidator { bottom: -20; top: 20; decimals: 2; notation: DoubleValidator.StandardNotation; locale: "C" }
                                                    onTextChanged: root.gridAnglesValid = root.ninePointAngles().length === 9
                                                    onAcceptableInputChanged: root.gridAnglesValid = root.ninePointAngles().length === 9
                                                    background: Rectangle {
                                                        radius: Theme.radiusSmall
                                                        color: angleField.stepActive ? Theme.accentSoft
                                                             : angleField.stepDone ? Theme.okSoft
                                                             : index === 4 ? Theme.accentSoft
                                                             : Theme.surface
                                                        border.width: angleField.stepActive || angleField.activeFocus || angleField.invalid ? 1.5 : 1
                                                        border.color: angleField.invalid ? Theme.danger
                                                                    : angleField.stepActive ? Theme.accent
                                                                    : angleField.stepDone ? Theme.okLine
                                                                    : angleField.activeFocus ? Theme.accent
                                                                    : index === 4 ? Theme.accentLine
                                                                    : Theme.hairline
                                                    }
                                                    HmiText { anchors.left: parent.left; anchors.leftMargin: 4; anchors.top: parent.top; anchors.topMargin: 2; text: index + 1; font.pixelSize: 10; font.family: Theme.mono; color: angleField.stepActive ? Theme.accent : angleField.stepDone ? Theme.okText : Theme.ink3 }
                                                    HmiText { anchors.right: parent.right; anchors.rightMargin: 4; anchors.verticalCenter: parent.verticalCenter; text: "°"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        // CONFIGURED POINTS PANEL
                        Rectangle {
                            visible: root.configuredPointMode; Layout.fillWidth: true
                            implicitHeight: cfgCol.implicitHeight + 18
                            radius: Theme.radiusControl; color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                            ColumnLayout {
                                id: cfgCol; anchors.fill: parent; anchors.margins: 9; spacing: 6
                                RowLayout {
                                    Layout.fillWidth: true
                                    HmiText { text: "move_tcp_points.json"; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink }
                                    Item { Layout.fillWidth: true }
                                    Rectangle {
                                        implicitWidth: cfgSummary.implicitWidth + 12; implicitHeight: 20; radius: 10
                                        color: root.configuredPoints.error ? Theme.warningSoft : Theme.accentSoft
                                        HmiText {
                                            id: cfgSummary; objectName: "configuredPointSummary"; anchors.centerIn: parent
                                            font.pixelSize: Theme.micro; font.family: Theme.mono
                                            text: root.configuredPoints.error || root.configuredPoints.count + " 个点位"
                                            color: root.configuredPoints.error ? Theme.warningText : Theme.ink
                                        }
                                    }
                                }
                                CheckBox {
                                    id: saveConfiguredRaw; objectName: "saveConfiguredRaw"
                                    Layout.fillWidth: true; Layout.minimumWidth: 0; Layout.preferredHeight: root.compact ? 26 : 30
                                    text: "同时保存原始融合图（RAW）"
                                    enabled: !root.interactionLocked && (root.dualMode
                                        ? (root.controller.undistortionEnabled || root.controller.undistortion2Enabled)
                                        : root.controller.undistortionEnabled)
                                    font.pixelSize: Theme.micro; palette.windowText: Theme.ink
                                }
                            }
                        }

                        // PRIMARY CTA BUTTON + SLIM PROGRESS BAR
                        CollectButton {
                            objectName: root.busy ? "stopButton" : root.ninePointMode ? "ninePointButton" : root.configuredPointMode ? "configuredCaptureButton" : "captureButton"
                            Layout.fillWidth: true; Layout.preferredHeight: root.compact ? 42 : 48
                            text: root.busy ? (root.controller.motionFault ? "重试停止" : "停止采集")
                                  : root.ninePointMode ? (root.dualMode ? "开始双相机九点" : "开始九点采集")
                                  : root.configuredPointMode ? (root.dualMode ? "开始双相机按配置" : "开始按配置采图")
                                  : (root.dualMode ? "采集双相机图片" : "采集图片")
                            primary: !root.busy; danger: root.busy; solidDanger: root.busy
                            enabled: root.busy || root.ready && !teach.jogHeld &&
                                     (root.captureMode === "single" || root.controller.robotConnected &&
                                      (!root.ninePointMode || root.gridOffsetValid && root.gridAnglesValid))
                            onClicked: {
                                if (root.busy) root.controller.cancelCollectionOperation()
                                else if (root.ninePointMode) root.controller.captureNinePoints(Number(offsetX.text), Number(offsetY.text), root.ninePointAngles())
                                else if (root.configuredPointMode) root.controller.captureConfiguredPoints(saveConfiguredRaw.checked && (root.dualMode ? (root.controller.undistortionEnabled || root.controller.undistortion2Enabled) : root.controller.undistortionEnabled))
                                else root.takePhoto()
                            }
                        }

                        // Slim Progress Bar
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 4; radius: 2
                            color: Theme.raised; clip: true
                            Rectangle {
                                objectName: "captureProgressFill"
                                height: parent.height; radius: 2
                                width: Math.round(parent.width * root.progressRatio)
                                color: root.controller.motionFault ? Theme.danger : root.progressFinished ? Theme.ok : Theme.accent
                            }
                        }

                        Item { Layout.fillHeight: true; Layout.minimumHeight: 0 }

                        // OPERATION BRIEF: Task + Progress + 2 Stat Cards + Feedback
                        ColumnLayout {
                            objectName: "operationBrief"; Layout.fillWidth: true; spacing: root.compact ? 4 : 6
                            RowLayout {
                                Layout.fillWidth: true; spacing: 6
                                HmiText {
                                    objectName: "taskSummary"; font.pixelSize: Theme.micro; color: Theme.ink2
                                    text: "状态 · " + (root.controller.motionFault ? "等待停止确认" : root.controller.jogActive ? "长按微调" : root.controller.manualMoveActive ? "目标移动" : root.controller.collectionBusy ? "采集中" : "待机")
                                }
                                HmiText {
                                    objectName: "ninePointProgress"; Layout.fillWidth: true; Layout.minimumWidth: 0
                                    horizontalAlignment: Text.AlignRight
                                    font.pixelSize: Theme.micro; font.family: Theme.mono; color: Theme.ink3
                                    elide: Text.ElideRight; wrapMode: Text.NoWrap
                                    text: root.controller.ninePointProgress || "—"
                                }
                            }
                            RowLayout {
                                Layout.fillWidth: true; spacing: 6
                                // Stat Card 1: Saved count
                                Rectangle {
                                    Layout.fillWidth: true; Layout.preferredHeight: root.compact ? 46 : 54
                                    radius: Theme.radiusControl; color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                                    ColumnLayout {
                                        anchors.fill: parent; anchors.margins: 7; spacing: 1
                                        HmiText { text: "已保存"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                        HmiText {
                                            Layout.fillWidth: true; color: Theme.ink; font.family: Theme.mono; font.pixelSize: root.compact ? Theme.body : Theme.title; font.weight: Font.DemiBold
                                            elide: Text.ElideRight; wrapMode: Text.NoWrap
                                            text: root.dualMode && root.controller.savedGroupCount > 0
                                                ? root.controller.savedGroupCount + " 组"
                                                : root.controller.savedImageCount + " 张"
                                        }
                                    }
                                }
                                // Stat Card 2: Last Capture Time
                                Rectangle {
                                    Layout.fillWidth: true; Layout.preferredHeight: root.compact ? 46 : 54
                                    radius: Theme.radiusControl; color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                                    ColumnLayout {
                                        anchors.fill: parent; anchors.margins: 7; spacing: 1
                                        HmiText { text: "最近采集"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                        HmiText {
                                            objectName: "savedCountSummary"
                                            Layout.fillWidth: true; color: Theme.ink; font.family: Theme.mono; font.pixelSize: root.compact ? Theme.meta : Theme.body; font.weight: Font.DemiBold
                                            elide: Text.ElideRight; wrapMode: Text.NoWrap
                                            text: root.controller.lastCaptureTime || "—"
                                        }
                                    }
                                }
                            }
                            HmiText {
                                objectName: "latestResult"; Layout.fillWidth: true; font.pixelSize: Theme.micro
                                maximumLineCount: root.compact ? 1 : 2; elide: Text.ElideRight
                                text: !teach.poseReady || root.busy || teach.targetDraft && !teach.parsed.valid ? teach.reason : teach.feedback || teach.reason
                                color: root.controller.motionFault ? Theme.danger : teach.targetDraft ? Theme.warningText : Theme.ink3
                            }
                        }
                    }
                }

                // RIGHT MAIN COLUMN: HERO VIEWFINDER + UNIFIED TCP TEACHING DECK
                ColumnLayout {
                    objectName: "mainRightColumn"; Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumWidth: 0; spacing: root.compact ? 10 : 12
                    // UPPER CARD: VIEWFINDER
                    CollectCard {
                        Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumHeight: 200; padding: 0
                        ColumnLayout {
                            anchors.fill: parent; spacing: 0
                            // Viewfinder Top Toolbar (High-frequency left, low-frequency right)
                            Rectangle {
                                id: viewfinderBar
                                readonly property bool tight: width < 580
                                Layout.fillWidth: true; Layout.preferredHeight: 42; color: Theme.surface
                                Rectangle { anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom; height: 1; color: Theme.hairline }
                                RowLayout {
                                    anchors.fill: parent; anchors.leftMargin: 10; anchors.rightMargin: 10; spacing: viewfinderBar.tight ? 4 : 8
                                    // Dual-camera switcher (visible in dual mode)
                                    Rectangle {
                                        visible: root.dualMode; Layout.preferredHeight: 30; implicitWidth: dualSwitchRow.implicitWidth + 6
                                        radius: 6; color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                                        RowLayout {
                                            id: dualSwitchRow; anchors.centerIn: parent; spacing: 2
                                            CollectButton {
                                                objectName: "previewCam1Button"; small: true; segmented: true; statusDot: true
                                                implicitHeight: 24; leftPadding: 8; rightPadding: 8
                                                text: "相机 1"; selected: root.activeCam === 1
                                                dotColor: !root.controller.cameraOpened ? Theme.ink3 : root.controller.camera1Healthy ? Theme.ok : Theme.danger
                                                onClicked: root.controller.activePreviewCamera = 1
                                            }
                                            CollectButton {
                                                objectName: "previewCam2Button"; small: true; segmented: true; statusDot: true
                                                implicitHeight: 24; leftPadding: 8; rightPadding: 8
                                                text: "相机 2"; selected: root.activeCam === 2
                                                dotColor: !root.controller.camera2Opened ? Theme.ink3 : root.controller.camera2Healthy ? Theme.ok : Theme.danger
                                                onClicked: root.controller.activePreviewCamera = 2
                                            }
                                        }
                                    }
                                    // Pause / Resume Preview (Moved to viewfinder toolbar!)
                                    CollectButton {
                                        objectName: "previewButton"; small: true
                                        implicitHeight: 28; leftPadding: viewfinderBar.tight ? 6 : 10; rightPadding: viewfinderBar.tight ? 6 : 10
                                        text: viewfinderBar.tight ? (root.controller.previewActive ? "暂停" : "恢复") : (root.controller.previewActive ? "暂停预览" : "恢复预览")
                                        enabled: root.anyCameraOpen && !root.interactionLocked
                                        onClicked: root.controller.previewActive ? root.controller.stopPreview() : root.controller.startPreview()
                                    }
                                    // Quick Undistortion Toggle (One-click directly on preview bar!)
                                    CollectButton {
                                        objectName: "quickUndistortButton"; small: true
                                        implicitHeight: 28; leftPadding: viewfinderBar.tight ? 6 : 10; rightPadding: viewfinderBar.tight ? 6 : 10
                                        positive: root.activeUndistortEnabled
                                        text: viewfinderBar.tight ? (root.activeUndistortEnabled ? "去畸变✓" : "去畸变")
                                                                  : (root.activeUndistortEnabled ? "去畸变: 开" : "去畸变: 关")
                                        enabled: !root.interactionLocked && root.activeUndistortReady
                                        ToolTip.visible: hovered; ToolTip.text: root.activeUndistortStatus || "切换实时预览与采图去畸变"
                                        onClicked: {
                                            if (root.dualMode && root.activeCam === 2) root.controller.toggleUndistortion2()
                                            else root.controller.toggleUndistortion()
                                        }
                                    }
                                    // Crosshair Switch (Camera 1 overlay only in dual mode)
                                    RowLayout {
                                        id: crosshairControl
                                        objectName: "crosshairControl"
                                        readonly property bool hoveredForTooltip: crosshairHover.hovered
                                        spacing: 5
                                        opacity: root.crosshairSupported ? 1 : 0.4
                                        ToolTip.visible: crosshairHover.hovered
                                        ToolTip.text: root.crosshairSupported ? "显示或隐藏预览中心十字线（仅预览有效）" : "相机 2 预览不显示中心十字线（仅相机 1 支持）"
                                        HoverHandler { id: crosshairHover }
                                        Rectangle {
                                            objectName: "crosshairToggle"
                                            enabled: root.crosshairSupported
                                            Layout.preferredWidth: 30; Layout.preferredHeight: 17; radius: 8.5
                                            color: root.crosshairVisibleOnPreview ? Theme.accent : Theme.raised
                                            border.width: 1; border.color: root.crosshairVisibleOnPreview ? Theme.accent : Theme.hairlineStrong
                                            Rectangle {
                                                width: 13; height: 13; radius: 6.5; anchors.verticalCenter: parent.verticalCenter
                                                x: root.crosshairVisibleOnPreview ? parent.width - width - 2 : 2
                                                color: Theme.white
                                            }
                                            MouseArea {
                                                anchors.fill: parent; enabled: root.crosshairSupported; cursorShape: Qt.PointingHandCursor
                                                onClicked: root.showCrosshair = !root.showCrosshair
                                            }
                                        }
                                        HmiText {
                                            visible: !viewfinderBar.tight
                                            text: "十字线"; font.pixelSize: Theme.meta; color: root.crosshairVisibleOnPreview ? Theme.ink : Theme.ink3
                                            MouseArea { anchors.fill: parent; enabled: root.crosshairSupported; cursorShape: Qt.PointingHandCursor; onClicked: root.showCrosshair = !root.showCrosshair }
                                        }
                                    }
                                    Item { Layout.fillWidth: true }
                                    // Direction Markers (Right side, RGB colored labels)
                                    HmiText { text: "X+"; font.family: Theme.mono; font.pixelSize: Theme.meta; font.weight: Font.DemiBold; color: Theme.axisX }
                                    DirectionSelector { objectName: "previewDirectionX"; direction: root.controller.previewPositiveX; onChosen: root.controller.setPreviewDirections(value, root.controller.previewPositiveY) }
                                    HmiText { text: "Y+"; font.family: Theme.mono; font.pixelSize: Theme.meta; font.weight: Font.DemiBold; color: Theme.axisY }
                                    DirectionSelector { objectName: "previewDirectionY"; direction: root.controller.previewPositiveY; perpendicularTo: root.controller.previewPositiveX; restrictPerpendicular: true; onChosen: root.controller.setPreviewDirections(root.controller.previewPositiveX, value) }
                                }
                            }

                            // Pitch-Black Camera Canvas
                            Rectangle {
                                objectName: "captureImageArea"; Layout.fillWidth: true; Layout.fillHeight: true; color: Theme.viewfinder; clip: true
                                property int seenCount: root.seenCount
                                Image { id: cameraImage; anchors.fill: parent; fillMode: Image.PreserveAspectFit; cache: false; asynchronous: false; visible: root.hasFrame }
                                // Toggleable Crosshair (only on Cam 1 in dual mode)
                                Rectangle { visible: root.crosshairVisibleOnPreview; anchors.verticalCenter: parent.verticalCenter; width: parent.width; height: 1; color: "#28FFFFFF" }
                                Rectangle { visible: root.crosshairVisibleOnPreview; anchors.horizontalCenter: parent.horizontalCenter; height: parent.height; width: 1; color: "#28FFFFFF" }
                                Rectangle { visible: root.crosshairVisibleOnPreview; anchors.centerIn: parent; width: 120; height: 3; color: Theme.danger; opacity: 0.85 }
                                Rectangle { visible: root.crosshairVisibleOnPreview; anchors.centerIn: parent; width: 3; height: 120; color: Theme.danger; opacity: 0.85 }
                                // Centered Empty State Prompt
                                Column {
                                    visible: !root.hasFrame
                                    anchors.centerIn: parent; spacing: 8
                                    CollectIcon { anchors.horizontalCenter: parent.horizontalCenter; width: 32; height: 32; ink: Theme.ink3; opacity: 0.5 }
                                    HmiText {
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        horizontalAlignment: Text.AlignHCenter; wrapMode: Text.WordWrap
                                        text: !root.dualMode ? "等待相机画面 · 点击顶部「连接相机」"
                                              : root.activeCam === 2 ? root.controller.camera2StatusMessage
                                              : (root.controller.cameraOpened && root.controller.previewActive && !root.controller.camera1Healthy
                                                  ? "相机 1 等待画面或取帧异常，请检查连接"
                                                  : "等待相机 1 画面 · 点击顶部「连接相机」")
                                        font.pixelSize: Theme.meta; color: Theme.ink3
                                    }
                                }
                                // Floating Metadata Pill
                                Rectangle {
                                    visible: root.hasFrame; x: 12; y: 10; height: 26
                                    width: Math.min(parent.width - 24, frameLabel.implicitWidth + 22); radius: 13
                                    color: Theme.chip; border.width: 1
                                    border.color: root.dualMode && root.activeCam === 2 ? Theme.warningLine : Theme.accentLine
                                    HmiText {
                                        id: frameLabel; objectName: "frameMetadata"
                                        anchors.fill: parent; anchors.leftMargin: 11; anchors.rightMargin: 11
                                        text: root.frameMetadata; color: Theme.ink
                                        font.pixelSize: Theme.micro; font.family: Theme.mono
                                        verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight; wrapMode: Text.NoWrap
                                    }
                                }
                                // Floating RGB Direction Gizmo
                                DirectionGizmo {
                                    objectName: "previewDirectionGizmo"
                                    x: (cameraImage.width - cameraImage.paintedWidth) / 2 + 10
                                    y: (cameraImage.height + cameraImage.paintedHeight) / 2 - height - 10
                                    visible: (!root.dualMode || root.activeCam === 1) && root.hasFrame && root.controller.previewPositiveX !== "none"
                                    directionX: root.controller.previewPositiveX; directionY: root.controller.previewPositiveY
                                    width: Math.max(0, Math.min(96, cameraImage.paintedWidth - 16, cameraImage.paintedHeight - 16)); height: width
                                }
                                Rectangle { id: flash; objectName: "shutterFlashOverlay"; anchors.fill: parent; color: Theme.white; opacity: 0 }
                                SequentialAnimation { id: shutterFlash; objectName: "shutterFlashAnimation"; NumberAnimation { target: flash; property: "opacity"; from: 0.5; to: 0; duration: 240 } }
                            }
                        }
                    }

                    // LOWER CARD: UNIFIED FULL-WIDTH 3x2 TCP TEACHING DECK
                    TcpTeachCard {
                        id: teach; Layout.fillWidth: true; Layout.preferredHeight: 208; Layout.minimumHeight: 208
                        controller: root.controller; locked: root.busy; tcpReady: root.tcpReady
                        windowReady: root.active && root.visible && root.visibility !== Window.Minimized
                        pageVisible: root.page === "main"
                        onSavePoseRequested: root.savePose()
                    }
                }
            }
        }
    }

    // ==================== SETTINGS PAGE ====================
    SettingsPage { id: settings; objectName: "settingsPage"; anchors.fill: parent; visible: root.page === "settings"; controller: root.controller; locked: root.busy; onLeave: root.page = "main" }

    // ==================== SAVE TCP POSE DIALOG ====================
    Dialog {
        id: poseDialog; objectName: "savePoseDialog"; anchors.centerIn: parent; width: Math.min(480, root.width - 32); modal: true; title: "保存 TCP 坐标"; closePolicy: Popup.CloseOnEscape
        header: HmiText { objectName: "poseDialogTitle"; text: poseDialog.title; color: Theme.ink; font.pixelSize: Theme.appTitle; font.weight: Font.DemiBold; padding: 18 }
        onClosed: root.poseEditorOpen = false
        background: Rectangle { color: Theme.surface; border.color: Theme.hairlineStrong; radius: Theme.radiusCard }
        contentItem: ColumnLayout {
            spacing: 14
            HmiText { text: "当前工具：" + root.controller.selectedTcp; Layout.fillWidth: true; color: Theme.ink }
            HmiText { text: "点位名称 · 留空使用时间戳"; Layout.fillWidth: true; font.pixelSize: Theme.meta }
            CollectField { id: pointName; objectName: "pointNameInput"; mono: false; Layout.fillWidth: true; enabled: root.tcpReady && !root.interactionLocked; onAccepted: root.confirmPose(); Keys.onEscapePressed: { poseDialog.close(); event.accepted = true } }
            HmiText { Layout.fillWidth: true; text: root.controller.statusMessage; font.pixelSize: Theme.meta }
            RowLayout { Item { Layout.fillWidth: true } CollectButton { text: "返回"; quiet: true; onClicked: poseDialog.close() } CollectButton { objectName: "confirmPoseButton"; text: "保存点位"; primary: true; enabled: root.tcpReady && !root.interactionLocked; onClicked: root.confirmPose() } }
        }
    }
}
