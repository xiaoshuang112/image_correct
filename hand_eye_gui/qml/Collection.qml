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
    readonly property bool ninePointMode: captureMode === "nine"
    readonly property bool configuredPointMode: captureMode === "configured"
    readonly property bool dualMode: controller.dualCameraMode
    readonly property int activeCam: dualMode ? controller.activePreviewCamera : 1
    property var configuredPoints: ({count: 0, error: ""})
    property bool gridAnglesValid: false
    property bool poseEditorOpen: false
    readonly property bool busy: controller.collectionBusy || controller.manualMoveActive || controller.jogActive || controller.motionFault
    readonly property bool ready: dualMode ? controller.dualCameraReady && !busy
                                          : controller.cameraOpened && controller.previewActive && !busy
    readonly property bool compact: width < 1100 || height < (ninePointMode ? 840 : 740) || teach.targetDraft
    readonly property bool tcpReady: controller.robotConnected && controller.selectedTcp !== ""
    readonly property bool gridOffsetValid: offsetX.acceptableInput && offsetY.acceptableInput && Number(offsetX.text) >= 0.1 && Number(offsetY.text) >= 0.1 && Math.sqrt(Math.pow(Number(offsetX.text), 2) + Math.pow(Number(offsetY.text), 2)) <= 50
    readonly property bool editing: activeFocusItem !== null && (activeFocusItem.cursorPosition !== undefined || activeFocusItem.hasOwnProperty("currentIndex"))
    readonly property bool shortcutReady: page === "main" && ready && captureMode === "single" && !editing && !poseEditorOpen && !teach.targetDraft && !teach.jogHeld && !tcpSelector.activeFocus && !tcpSelector.popup.visible
    readonly property bool switchShortcutReady: page === "main" && dualMode && !editing && !poseEditorOpen && !teach.targetDraft && !teach.jogHeld && !tcpSelector.activeFocus && !tcpSelector.popup.visible
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
    onControllerChanged: { historyReady = false; seenCount = controller.savedImageCount; shutterFlash.stop(); flash.opacity = 0; historyReady = true; hasFrame = false; frameMetadata = "" }
    onClosing: function(event) {
        teach.stopJog()
        if (busy) { event.accepted = false; controller.cancelCollectionOperation() }
        else if (page === "settings" && settings.dirty) { event.accepted = false; settings.requestLeave() }
    }
    Component.onCompleted: { seenCount = controller.savedImageCount; historyReady = true;  }
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
    Item {
        anchors.fill: parent; visible: root.page === "main"
        ColumnLayout {
            anchors.fill: parent; spacing: 0
            Rectangle {
                color: Theme.surface; Layout.fillWidth: true; Layout.preferredHeight: 56
                RowLayout {
                    anchors.fill: parent; anchors.leftMargin: 16; anchors.rightMargin: 16; spacing: 12
                    CollectIcon { ink: Theme.accent; Layout.preferredWidth: 24; Layout.preferredHeight: 24 }
                    HmiText { visible: root.width >= 1100; text: "图像采集"; color: Theme.ink; font.pixelSize: Theme.title; font.weight: Font.DemiBold }
                    Rectangle {
                        Layout.fillWidth: true; Layout.minimumWidth: 80; Layout.preferredHeight: 28; radius: 14
                        readonly property color tone: root.controller.motionFault || /失败|异常|故障|意外变化|过慢|超时/.test(root.controller.statusMessage) ? Theme.danger : root.busy ? Theme.accent : root.controller.cameraOpened ? Theme.ok : Theme.warning
                        color: Qt.rgba(tone.r,tone.g,tone.b,0.12); border.color: Qt.rgba(tone.r,tone.g,tone.b,0.35)
                        HmiText { objectName: "globalStatus"; anchors.fill: parent; anchors.leftMargin: 12; anchors.rightMargin: 12; text: root.controller.statusMessage; color: parent.tone; elide: Text.ElideRight; wrapMode: Text.NoWrap; verticalAlignment: Text.AlignVCenter }
                    }
                    RowLayout {
                        spacing: 4
                        CollectButton {
                            objectName: "singleCameraModeButton"; small: true; text: "单相机"
                            selected: !root.dualMode
                            enabled: !root.interactionLocked && !root.controller.cameraOpened && !root.controller.camera2Opened
                            onClicked: root.controller.dualCameraMode = false
                        }
                        CollectButton {
                            objectName: "dualCameraModeButton"; small: true; text: "双相机"
                            selected: root.dualMode
                            enabled: !root.interactionLocked && !root.controller.cameraOpened && !root.controller.camera2Opened
                            onClicked: root.controller.dualCameraMode = true
                        }
                    }
                    CollectButton {
                        objectName: "openCameraButton"; small: true
                        text: (root.controller.cameraOpened || root.controller.camera2Opened) ? "断开相机" : "连接相机"
                        enabled: !root.interactionLocked
                        onClicked: {
                            if (root.controller.cameraOpened || root.controller.camera2Opened) {
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
                    CollectButton { objectName: "connectRobotButton"; small: true; text: root.controller.robotConnected ? "断开机器人" : "连接机器人"; enabled: !root.interactionLocked; onClicked: root.controller.robotConnected ? root.controller.disconnectRobot() : root.controller.connectRobot() }
                    HmiText { visible: root.width >= 1100; text: appGitCommit; font.family: Theme.mono; font.pixelSize: Theme.meta }
                    CollectButton { objectName: "settingsButton"; small: true; text: "设置"; quiet: true; enabled: !root.interactionLocked; onClicked: { settings.reload(); root.page = "settings" } }
                }
            }
            RowLayout {
                objectName: "mainDeck"; Layout.fillWidth: true; Layout.fillHeight: true; Layout.margins: root.width < 900 ? 12 : 16; spacing: root.width < 900 ? 12 : 16
                CollectCard {
                    objectName: "mainControls"; Layout.preferredWidth: root.width < 900 ? 200 : root.compact ? 232 : 264; Layout.fillHeight: true; padding: root.compact ? 10 : 16
                    ColumnLayout {
                        anchors.fill: parent; spacing: root.compact ? 4 : 10
                        HmiText { visible: !root.compact; text: "采集模式"; font.pixelSize: Theme.meta; font.letterSpacing: 1.5 }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 4
                            CollectButton { objectName: "singleModeButton"; text: "单次"; small: root.compact; selected: root.captureMode === "single"; Layout.fillWidth: true; enabled: !root.interactionLocked; onClicked: root.captureMode = "single" }
                            CollectButton { objectName: "ninePointModeButton"; text: "九点"; small: root.compact; selected: root.ninePointMode; Layout.fillWidth: true; enabled: !root.interactionLocked; onClicked: root.captureMode = "nine" }
                            CollectButton {
                                objectName: "configuredModeButton"; text: "按配置"; small: root.compact; selected: root.configuredPointMode
                                Layout.fillWidth: true; enabled: !root.interactionLocked
                                onClicked: {
                                    root.captureMode = "configured"
                                    root.configuredPoints = root.controller.configuredPointSummary()
                                }
                            }
                        }
                        RowLayout {
                            visible: root.ninePointMode; Layout.fillWidth: true; spacing: 8
                            CollectField { id: offsetX; objectName: "ninePointOffsetXInput"; implicitHeight: root.compact ? 32 : 44; leftPadding: 28; text: root.formatPlanNumber(root.controller.ninePointOffsetXMm); placeholderText: "ΔX mm"; Layout.fillWidth: true; Layout.minimumWidth: 0; enabled: !root.interactionLocked; validator: DoubleValidator { bottom: 0.1; top: 50; decimals: 2; locale: "C" } HmiText { anchors.left: parent.left; anchors.leftMargin: 5; anchors.verticalCenter: parent.verticalCenter; text: "ΔX"; font.pixelSize: Theme.meta } }
                            CollectField { id: offsetY; objectName: "ninePointOffsetYInput"; implicitHeight: root.compact ? 32 : 44; leftPadding: 28; text: root.formatPlanNumber(root.controller.ninePointOffsetYMm); placeholderText: "ΔY mm"; Layout.fillWidth: true; Layout.minimumWidth: 0; enabled: !root.interactionLocked; validator: DoubleValidator { bottom: 0.1; top: 50; decimals: 2; locale: "C" } HmiText { anchors.left: parent.left; anchors.leftMargin: 5; anchors.verticalCenter: parent.verticalCenter; text: "ΔY"; font.pixelSize: Theme.meta } }
                        }
                        HmiText { visible: root.ninePointMode && !root.compact; text: "X / Y 偏移 · mm（0.1～50）"; font.pixelSize: Theme.meta }
                        GridLayout {
                            objectName: "ninePointPlan"; visible: root.ninePointMode; Layout.fillWidth: true
                            columns: 3; columnSpacing: 4; rowSpacing: 4
                            Repeater {
                                id: angleInputs; model: root.controller.ninePointYawOffsets
                                onItemAdded: root.gridAnglesValid = root.ninePointAngles().length === 9
                                onItemRemoved: root.gridAnglesValid = root.ninePointAngles().length === 9
                                CollectField {
                                    id: angleField; objectName: "ninePointAngle" + index
                                    Layout.fillWidth: true; Layout.minimumWidth: 0; implicitHeight: root.compact ? 28 : 32
                                    text: (modelData > 0 ? "+" : "") + modelData; enabled: !root.interactionLocked
                                    leftPadding: 16; rightPadding: 12; font.pixelSize: root.compact ? Theme.meta : Theme.body
                                    horizontalAlignment: TextInput.AlignHCenter; selectByMouse: true
                                    invalid: !acceptableInput
                                    validator: DoubleValidator { bottom: -20; top: 20; decimals: 2; notation: DoubleValidator.StandardNotation; locale: "C" }
                                    onTextChanged: root.gridAnglesValid = root.ninePointAngles().length === 9
                                    onAcceptableInputChanged: root.gridAnglesValid = root.ninePointAngles().length === 9
                                    background: Rectangle {
                                        radius: Theme.radiusSmall; color: index === 4 ? Theme.accentSoft : Theme.bg
                                        border.color: angleField.invalid ? Theme.danger : angleField.activeFocus ? Theme.accent : index === 4 ? Theme.accentLine : Theme.hairline
                                    }
                                    HmiText { anchors.left: parent.left; anchors.leftMargin: 5; anchors.verticalCenter: parent.verticalCenter; text: index + 1; font.pixelSize: Theme.meta; color: Theme.ink3 }
                                    HmiText { anchors.right: parent.right; anchors.rightMargin: 5; anchors.verticalCenter: parent.verticalCenter; text: "°"; font.pixelSize: Theme.meta; color: Theme.ink3 }
                                }
                            }
                        }
                        HmiText { visible: root.ninePointMode && !root.compact; text: "Rz 可编辑 · 相对基准（±20°）"; font.pixelSize: Theme.meta }
                        HmiText { objectName: "ninePointOffsetError"; visible: root.ninePointMode && !root.gridOffsetValid; Layout.fillWidth: true; text: "偏移须为 0.1～50 mm，对角距基准 ≤50 mm"; color: Theme.warning; font.pixelSize: Theme.meta }
                        ColumnLayout {
                            visible: root.configuredPointMode; Layout.fillWidth: true; spacing: 2
                            HmiText { text: "move_tcp_points.json"; font.family: Theme.mono; font.pixelSize: Theme.meta }
                            HmiText {
                                objectName: "configuredPointSummary"; Layout.fillWidth: true; font.pixelSize: Theme.meta
                                text: root.configuredPoints.error || root.configuredPoints.count + " 个点位"
                                color: root.configuredPoints.error ? Theme.warning : Theme.ink2
                            }
                            CheckBox {
                                id: saveConfiguredRaw; objectName: "saveConfiguredRaw"
                                Layout.fillWidth: true; Layout.minimumWidth: 0; Layout.preferredHeight: root.compact ? 28 : 34
                                text: "同时保存原始融合图（RAW）"
                                enabled: !root.interactionLocked && (root.dualMode
                                    ? (root.controller.undistortionEnabled || root.controller.undistortion2Enabled)
                                    : root.controller.undistortionEnabled)
                                font.pixelSize: Theme.meta; palette.windowText: Theme.ink
                            }
                        }
                        CollectButton {
                            objectName: root.busy ? "stopButton" : root.ninePointMode ? "ninePointButton" : root.configuredPointMode ? "configuredCaptureButton" : "captureButton"
                            Layout.fillWidth: true; Layout.preferredHeight: root.compact ? 44 : 56
                            text: root.busy ? root.controller.motionFault ? "重试停止" : "停止采集"
                                  : root.ninePointMode ? (root.dualMode ? "开始双相机九点" : "开始九点采集")
                                  : root.configuredPointMode ? (root.dualMode ? "开始双相机按配置" : "开始按配置采图")
                                  : (root.dualMode ? "采集双相机图片" : "采集图片")
                            primary: !root.interactionLocked; danger: root.busy
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
                        Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: Theme.hairline }
                        HmiText { visible: !root.compact; text: "TCP 工具"; font.pixelSize: Theme.meta; font.letterSpacing: 1.5 }
                        TcpSelector { id: tcpSelector; implicitHeight: root.compact ? 32 : 44; objectName: "tcpSelector"; controller: root.controller; Layout.fillWidth: true; enabled: !root.interactionLocked && root.controller.robotConnected && root.controller.tcpNames.length > 0 }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 8
                            CollectButton { objectName: "refreshTcpButton"; text: "刷新"; small: root.compact; quiet: true; Layout.fillWidth: true; enabled: !root.interactionLocked && root.controller.robotConnected; onClicked: root.controller.refreshTcpTools() }
                            CollectButton { objectName: "savePoseButton"; text: "保存 TCP"; small: root.compact; Layout.fillWidth: true; enabled: !root.interactionLocked && root.tcpReady; onClicked: root.savePose() }
                        }
                        Item { Layout.fillHeight: true; Layout.minimumHeight: 0 }
                        Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: Theme.hairline }
                        ColumnLayout {
                            objectName: "operationBrief"; Layout.fillWidth: true; spacing: root.compact ? 2 : 6
                            HmiText { visible: !root.compact; text: "运行信息"; font.pixelSize: Theme.meta }
                            HmiText { objectName: "taskSummary"; Layout.fillWidth: true; font.pixelSize: Theme.meta; text: "当前任务  " + (root.controller.motionFault ? "等待停止确认" : root.controller.jogActive ? "长按微调" : root.controller.manualMoveActive ? "目标移动" : root.controller.collectionBusy ? "采集中" : "待机") }
                            HmiText { objectName: "ninePointProgress"; Layout.fillWidth: true; font.pixelSize: Theme.meta; maximumLineCount: root.compact ? 1 : 2; elide: Text.ElideRight; text: "采集进度  " + (root.controller.ninePointProgress || "—") }
                            HmiText {
                                objectName: "savedCountSummary"; Layout.fillWidth: true; font.pixelSize: Theme.meta
                                text: root.dualMode && root.controller.savedGroupCount > 0
                                    ? "已保存  " + root.controller.savedGroupCount + " 组（" + root.controller.savedImageCount + " 张）   ·   " + (root.controller.lastCaptureTime || "—")
                                    : "已保存  " + root.controller.savedImageCount + " 张   ·   " + (root.controller.lastCaptureTime || "—")
                            }
                            HmiText { objectName: "latestResult"; Layout.fillWidth: true; font.pixelSize: Theme.meta; maximumLineCount: root.compact ? 2 : 3; elide: Text.ElideRight; text: !teach.poseReady || root.busy || teach.targetDraft && !teach.parsed.valid ? teach.reason : teach.feedback || teach.reason; color: root.controller.motionFault ? Theme.danger : teach.targetDraft ? Theme.warning : Theme.ink2 }
                            HmiText {
                                objectName: "actualPoseWhileEditing"; visible: teach.targetDraft; Layout.fillWidth: true; font.pixelSize: Theme.meta; font.family: Theme.mono
                                text: "当前 TCP · " + (root.controller.orientationMode === "euler" ? "欧拉角 ZYX" : "轴角") + "\n" + (teach.poseReady ? teach.actualValues.slice(0,3).map(teach.format).join(", ") + "\n" + teach.actualValues.slice(3).map(teach.format).join(", ") : "等待有效位姿")
                            }
                        }
                        CollectButton { objectName: "previewButton"; small: root.compact; Layout.fillWidth: true; text: root.controller.previewActive ? "暂停预览" : "恢复预览"; quiet: true; enabled: root.controller.cameraOpened && !root.interactionLocked; onClicked: root.controller.previewActive ? root.controller.stopPreview() : root.controller.startPreview() }
                    }
                }
                ColumnLayout {
                    objectName: "mainRightColumn"; Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumWidth: 0; spacing: 12
                    CollectCard {
                        Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumHeight: 120; padding: 0
                        ColumnLayout {
                            anchors.fill: parent; spacing: 0
                            RowLayout {
                                Layout.fillWidth: true; Layout.preferredHeight: 44; Layout.leftMargin: 10; Layout.rightMargin: 10; spacing: 6
                                HmiText { text: "X+"; font.family: Theme.mono; font.pixelSize: Theme.meta }
                                DirectionSelector { objectName: "previewDirectionX"; direction: root.controller.previewPositiveX; onChosen: root.controller.setPreviewDirections(value, root.controller.previewPositiveY) }
                                HmiText { text: "Y+"; font.family: Theme.mono; font.pixelSize: Theme.meta }
                                DirectionSelector { objectName: "previewDirectionY"; direction: root.controller.previewPositiveY; perpendicularTo: root.controller.previewPositiveX; restrictPerpendicular: true; onChosen: root.controller.setPreviewDirections(root.controller.previewPositiveX, value) }
                                Item { Layout.fillWidth: true }
                                RowLayout {
                                    visible: root.dualMode; spacing: 4
                                    CollectButton {
                                        objectName: "previewCam1Button"; small: true
                                        text: "相机 1"
                                        selected: root.activeCam === 1
                                        leftPadding: 18; rightPadding: 10
                                        onClicked: root.controller.activePreviewCamera = 1
                                        Rectangle {
                                            anchors.left: parent.left; anchors.leftMargin: 6; anchors.verticalCenter: parent.verticalCenter
                                            width: 7; height: 7; radius: 3.5
                                            color: !root.controller.cameraOpened ? Theme.ink3 : root.controller.camera1Healthy ? Theme.ok : Theme.danger
                                        }
                                    }
                                    CollectButton {
                                        objectName: "previewCam2Button"; small: true
                                        text: "相机 2"
                                        selected: root.activeCam === 2
                                        leftPadding: 18; rightPadding: 10
                                        onClicked: root.controller.activePreviewCamera = 2
                                        Rectangle {
                                            anchors.left: parent.left; anchors.leftMargin: 6; anchors.verticalCenter: parent.verticalCenter
                                            width: 7; height: 7; radius: 3.5
                                            color: !root.controller.camera2Opened ? Theme.ink3 : root.controller.camera2Healthy ? Theme.ok : Theme.danger
                                        }
                                    }
                                }
                                HmiText { visible: root.width >= 1024; text: "相机预览"; color: Theme.ink; font.pixelSize: Theme.body }
                                HmiText { visible: root.width >= 1100; text: "标记仅预览"; font.pixelSize: Theme.meta }
                            }
                            Rectangle {
                                objectName: "captureImageArea"; Layout.fillWidth: true; Layout.fillHeight: true; color: Theme.viewfinder; clip: true
                                property int seenCount: root.seenCount
                                Image { id: cameraImage; anchors.fill: parent; fillMode: Image.PreserveAspectFit; cache: false; asynchronous: false; visible: root.hasFrame }
                                Rectangle { visible: !root.dualMode || root.activeCam === 1; anchors.verticalCenter: parent.verticalCenter; width: parent.width; height: 1; color: Theme.hairline }
                                Rectangle { visible: !root.dualMode || root.activeCam === 1; anchors.horizontalCenter: parent.horizontalCenter; height: parent.height; width: 1; color: Theme.hairline }
                                Rectangle { visible: !root.dualMode || root.activeCam === 1; anchors.centerIn: parent; width: 120; height: 3; color: Theme.danger; opacity: 0.8 }
                                Rectangle { visible: !root.dualMode || root.activeCam === 1; anchors.centerIn: parent; width: 3; height: 120; color: Theme.danger; opacity: 0.8 }
                                HmiText {
                                    visible: !root.hasFrame
                                    anchors.horizontalCenter: parent.horizontalCenter; anchors.bottom: parent.bottom; anchors.bottomMargin: 16
                                    width: parent.width - 32; horizontalAlignment: Text.AlignHCenter; wrapMode: Text.WordWrap
                                    text: !root.dualMode ? "等待画面 · 点击「连接相机」"
                                          : root.activeCam === 2 ? root.controller.camera2StatusMessage
                                          : (root.controller.cameraOpened && root.controller.previewActive && !root.controller.camera1Healthy
                                              ? "相机 1 等待画面或取帧异常，请检查连接"
                                              : "等待相机 1 画面 · 点击「连接相机」")
                                    font.pixelSize: Theme.meta
                                }
                                Rectangle {
                                    visible: root.hasFrame; x: 12; y: 12; height: 28; width: Math.min(parent.width - 24, frameLabel.implicitWidth + 24); radius: 14; color: Theme.chip
                                    border.width: root.dualMode ? 1 : 0
                                    border.color: root.activeCam === 2 ? Theme.warning : Theme.accent
                                    HmiText { id: frameLabel; objectName: "frameMetadata"; anchors.fill: parent; anchors.leftMargin: 12; anchors.rightMargin: 12; text: root.frameMetadata; font.pixelSize: Theme.meta; font.family: Theme.mono; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight; wrapMode: Text.NoWrap }
                                }
                                DirectionGizmo {
                                    objectName: "previewDirectionGizmo"
                                    x: (cameraImage.width - cameraImage.paintedWidth) / 2 + 8
                                    y: (cameraImage.height + cameraImage.paintedHeight) / 2 - height - 8
                                    visible: (!root.dualMode || root.activeCam === 1) && root.hasFrame && root.controller.previewPositiveX !== "none"
                                    directionX: root.controller.previewPositiveX; directionY: root.controller.previewPositiveY
                                    width: Math.max(0, Math.min(112, cameraImage.paintedWidth - 16, cameraImage.paintedHeight - 16)); height: width
                                }
                                Rectangle { id: flash; objectName: "shutterFlashOverlay"; anchors.fill: parent; color: Theme.white; opacity: 0 }
                                SequentialAnimation { id: shutterFlash; objectName: "shutterFlashAnimation"; NumberAnimation { target: flash; property: "opacity"; from: 0.5; to: 0; duration: 240 } }
                            }
                        }
                    }
                    TcpTeachCard {
                        id: teach; Layout.fillWidth: true; Layout.preferredHeight: 210; Layout.minimumHeight: 210
                        controller: root.controller; locked: root.busy
                        windowReady: root.active && root.visible && root.visibility !== Window.Minimized
                        pageVisible: root.page === "main"
                    }
                }
            }
        }
    }
    SettingsPage { id: settings; objectName: "settingsPage"; anchors.fill: parent; visible: root.page === "settings"; controller: root.controller; locked: root.busy; onLeave: root.page = "main" }
    Dialog {
        id: poseDialog; objectName: "savePoseDialog"; anchors.centerIn: parent; width: Math.min(520, root.width - 32); modal: true; title: "保存 TCP 坐标"; closePolicy: Popup.CloseOnEscape
        header: HmiText { objectName: "poseDialogTitle"; text: poseDialog.title; color: Theme.ink; font.pixelSize: Theme.appTitle; padding: 20 }
        onClosed: root.poseEditorOpen = false
        background: Rectangle { color: Theme.surface; border.color: Theme.hairline; radius: Theme.radiusCard }
        contentItem: ColumnLayout {
            spacing: 16
            HmiText { text: "当前工具：" + root.controller.selectedTcp; Layout.fillWidth: true; color: Theme.ink }
            HmiText { text: "点位名称 · 留空使用时间戳"; Layout.fillWidth: true }
            CollectField { id: pointName; objectName: "pointNameInput"; mono: false; Layout.fillWidth: true; enabled: root.tcpReady && !root.interactionLocked; onAccepted: root.confirmPose(); Keys.onEscapePressed: { poseDialog.close(); event.accepted = true } }
            HmiText { Layout.fillWidth: true; text: root.controller.statusMessage; font.pixelSize: Theme.meta }
            RowLayout { Item { Layout.fillWidth: true } CollectButton { text: "返回"; quiet: true; onClicked: poseDialog.close() } CollectButton { objectName: "confirmPoseButton"; text: "保存点位"; primary: true; enabled: root.tcpReady && !root.interactionLocked; onClicked: root.confirmPose() } }
        }
    }
}
