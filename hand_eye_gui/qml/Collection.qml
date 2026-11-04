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
    property bool fivePointMode: false
    property bool poseEditorOpen: false
    readonly property bool busy: controller.collectionBusy || controller.manualMoveActive || controller.motionFault
    readonly property bool ready: controller.cameraOpened && controller.previewActive && !busy
    readonly property bool compact: width < 1100 || height < 740
    readonly property bool tcpReady: controller.robotConnected && controller.selectedTcp !== ""
    readonly property bool editing: activeFocusItem !== null && (activeFocusItem.cursorPosition !== undefined || activeFocusItem.hasOwnProperty("currentIndex"))
    readonly property bool shortcutReady: page === "main" && ready && !fivePointMode && !editing && !poseEditorOpen && !tcpSelector.activeFocus && !tcpSelector.popup.visible
    property var logLines: []
    property string frameMetadata: ""
    property bool hasFrame: false
    property int seenCount: controller.savedImageCount
    property int shutterSerial: 0
    property bool historyReady: false
    function takePhoto() { if (ready && page === "main") controller.capturePhoto() }
    function scrollLogToEnd() {
        Qt.callLater(function() {
            logText.cursorPosition = logText.length
            logScroll.contentItem.contentY = Math.max(0, logText.contentHeight - logScroll.availableHeight)
        })
    }
    function showBackendLog() {
        if (page !== "main") return
        var lines = controller.logText.split(/\r?\n/).filter(function(line) { return line !== "" })
        logLines = lines.slice(-1000)
        scrollLogToEnd()
    }
    function recordStatus(message) {
        if (!message || controller.logText !== "") return
        var next = logLines.slice(); next.push("[" + Qt.formatTime(new Date(), "hh:mm:ss") + "] " + message)
        logLines = next.slice(-1000)
        scrollLogToEnd()
    }
    function syncHistory() {
        var count = controller.savedImageCount
        if (historyReady && count > seenCount) { ++shutterSerial; shutterFlash.restart() }
        else if (count < seenCount) { shutterFlash.stop(); flash.opacity = 0 }
        seenCount = count; historyReady = true
    }
    function savePose() { pointName.text = ""; poseEditorOpen = true; poseDialog.open(); pointName.forceActiveFocus() }
    function confirmPose() { if (tcpReady && !busy) controller.saveTcpPose(pointName.text) }
    onPageChanged: { if (page === "main" && controller.logText) showBackendLog() }
    Binding {
        target: root.controller; property: "previewVisible"
        value: root.page === "main" && root.visible && root.visibility !== Window.Minimized
    }
    Binding {
        target: root.controller; property: "previewDisplaySize"
        value: Qt.size(Math.max(1, Math.round(cameraImage.width * root.screen.devicePixelRatio)),
                       Math.max(1, Math.round(cameraImage.height * root.screen.devicePixelRatio)))
    }
    onControllerChanged: { historyReady = false; seenCount = controller.savedImageCount; shutterFlash.stop(); flash.opacity = 0; historyReady = true; hasFrame = false; frameMetadata = "" }
    onClosing: function(event) {
        if (busy) { event.accepted = false; controller.cancelCollectionOperation() }
        else if (page === "settings" && settings.dirty) { event.accepted = false; settings.requestLeave() }
    }
    Component.onCompleted: { seenCount = controller.savedImageCount; historyReady = true; recordStatus("程序启动 · " + appGitCommit); recordStatus(controller.statusMessage); if (controller.logText) showBackendLog() }
    Shortcut { sequence: "Space"; autoRepeat: false; enabled: root.shortcutReady; onActivated: root.takePhoto() }
    Shortcut { sequence: "S"; autoRepeat: false; enabled: root.shortcutReady; onActivated: root.takePhoto() }
    Shortcut { sequence: "Ctrl+S"; autoRepeat: false; enabled: root.shortcutReady; onActivated: root.takePhoto() }
    Connections {
        target: root.controller
        function onCollectionChanged() { root.syncHistory() }
        function onLogTextChanged() { root.showBackendLog() }
        function onFrameUpdated() {
            if (root.page !== "main") return
            cameraImage.source = "image://camera/current#" + Date.now()
            root.hasFrame = true
            root.frameMetadata = root.controller.cameraModel + " · " + root.controller.cameraActualWidth + " × " + root.controller.cameraActualHeight + " · " + root.controller.cameraActualExposureUs + " μs" + (root.controller.undistortionEnabled ? " · 去畸变" : "")
        }
        function onStatusMessageChanged() {
            root.recordStatus(root.controller.statusMessage)
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
                        readonly property color tone: root.controller.motionFault || /失败[:：]|失败$|异常|故障/.test(root.controller.statusMessage) ? Theme.danger : root.busy ? Theme.accent : root.controller.cameraOpened ? Theme.ok : Theme.warning
                        color: Qt.rgba(tone.r,tone.g,tone.b,0.12); border.color: Qt.rgba(tone.r,tone.g,tone.b,0.35)
                        HmiText { objectName: "globalStatus"; anchors.fill: parent; anchors.leftMargin: 12; anchors.rightMargin: 12; text: root.controller.statusMessage; color: parent.tone; elide: Text.ElideRight; wrapMode: Text.NoWrap; verticalAlignment: Text.AlignVCenter }
                    }
                    CollectButton {
                        objectName: "openCameraButton"; small: true; text: root.controller.cameraOpened ? "断开相机" : "连接相机"; enabled: !root.busy
                        onClicked: { if (root.controller.cameraOpened) root.controller.closeCamera(); else { root.controller.openCamera(); if (root.controller.cameraOpened) root.controller.startPreview() } }
                    }
                    CollectButton { objectName: "connectRobotButton"; small: true; text: root.controller.robotConnected ? "断开机器人" : "连接机器人"; enabled: !root.busy; onClicked: root.controller.robotConnected ? root.controller.disconnectRobot() : root.controller.connectRobot() }
                    HmiText { visible: root.width >= 1100; text: appGitCommit; font.family: Theme.mono; font.pixelSize: Theme.meta }
                    CollectButton { objectName: "settingsButton"; small: true; text: "设置"; quiet: true; enabled: !root.busy; onClicked: { settings.reload(); root.page = "settings" } }
                }
            }
            RowLayout {
                objectName: "mainDeck"; Layout.fillWidth: true; Layout.fillHeight: true; Layout.margins: 16; spacing: 16
                CollectCard {
                    objectName: "mainControls"; Layout.preferredWidth: root.compact ? 248 : 300; Layout.fillHeight: true; padding: root.compact ? 12 : 20
                    ColumnLayout {
                        anchors.fill: parent; spacing: root.compact ? 8 : 12
                        HmiText { visible: !root.compact; text: "采集模式"; font.pixelSize: Theme.meta; font.letterSpacing: 1.5 }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 4
                            CollectButton { objectName: "singleModeButton"; text: "单次"; selected: !root.fivePointMode; Layout.fillWidth: true; enabled: !root.busy; onClicked: root.fivePointMode = false }
                            CollectButton { objectName: "fivePointModeButton"; text: "五点"; selected: root.fivePointMode; Layout.fillWidth: true; enabled: !root.busy; onClicked: root.fivePointMode = true }
                        }
                        RowLayout {
                            visible: root.fivePointMode; Layout.fillWidth: true; spacing: 8
                            CollectField { id: offsetX; objectName: "fivePointOffsetXInput"; text: "10"; placeholderText: "ΔX mm"; Layout.fillWidth: true; Layout.minimumWidth: 0; enabled: !root.busy; validator: DoubleValidator { bottom: 0.1; top: 50; decimals: 2; locale: "C" } }
                            CollectField { id: offsetY; objectName: "fivePointOffsetYInput"; text: "10"; placeholderText: "ΔY mm"; Layout.fillWidth: true; Layout.minimumWidth: 0; enabled: !root.busy; validator: DoubleValidator { bottom: 0.1; top: 50; decimals: 2; locale: "C" } }
                        }
                        HmiText { visible: root.fivePointMode && !root.compact; text: "X / Y 偏移 · mm（0.1～50）"; font.pixelSize: Theme.meta }
                        CollectButton {
                            objectName: root.busy ? "stopButton" : root.fivePointMode ? "fivePointButton" : "captureButton"
                            Layout.fillWidth: true; Layout.preferredHeight: 56
                            text: root.busy ? root.controller.motionFault ? "重试停止" : "停止采集" : root.fivePointMode ? "开始五点采集" : "采集图片"
                            primary: !root.busy; danger: root.busy
                            enabled: root.busy || root.ready && (!root.fivePointMode || root.controller.robotConnected && offsetX.acceptableInput && offsetY.acceptableInput && Number(offsetX.text) >= 0.1 && Number(offsetY.text) >= 0.1)
                            onClicked: { if (root.busy) root.controller.cancelCollectionOperation(); else if (root.fivePointMode) root.controller.captureFivePoints(Number(offsetX.text), Number(offsetY.text)); else root.takePhoto() }
                        }
                        HmiText {
                            objectName: "fivePointProgress"; Layout.fillWidth: true; Layout.preferredHeight: root.compact ? 32 : 44
                            text: root.controller.motionFault ? "停止尚未确认，操作保持锁定" : root.busy ? root.controller.fivePointProgress || "采集进行中，请稍候" : !root.controller.cameraOpened ? "连接相机后开始预览与采集" : !root.controller.previewActive ? "请先恢复预览" : root.fivePointMode ? "确认路径无障碍后开始五点采集" : "空格 / Ctrl+S 采集 · 多曝光融合"
                            color: root.controller.motionFault ? Theme.danger : Theme.ink3; font.pixelSize: Theme.meta
                        }
                        Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: Theme.hairline }
                        HmiText { visible: !root.compact; text: "TCP 工具"; font.pixelSize: Theme.meta; font.letterSpacing: 1.5 }
                        TcpSelector { id: tcpSelector; objectName: "tcpSelector"; controller: root.controller; Layout.fillWidth: true; enabled: !root.busy && root.controller.robotConnected && root.controller.tcpNames.length > 0 }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 8
                            CollectButton { objectName: "refreshTcpButton"; text: "刷新"; quiet: true; Layout.fillWidth: true; enabled: !root.busy && root.controller.robotConnected; onClicked: root.controller.refreshTcpTools() }
                            CollectButton { objectName: "savePoseButton"; text: "保存 TCP"; Layout.fillWidth: true; enabled: !root.busy && root.tcpReady; onClicked: root.savePose() }
                        }
                        CollectButton { objectName: "manualPageButton"; text: "手动移动 TCP →"; quiet: true; Layout.fillWidth: true; enabled: !root.busy && root.tcpReady; onClicked: root.page = "manual" }
                        Item { Layout.fillHeight: true; Layout.minimumHeight: 0 }
                        HmiText { Layout.fillWidth: true; text: root.busy ? "操作进行中，设备与工具编辑已锁定" : !root.controller.robotConnected ? "连接机器人后可使用 TCP" : !root.controller.selectedTcp ? "选择 TCP 后可保存或手动移动" : "工具：" + root.controller.selectedTcp; font.pixelSize: Theme.meta; color: Theme.ink3; maximumLineCount: 2; elide: Text.ElideRight }
                        CollectButton { objectName: "previewButton"; small: root.compact; Layout.fillWidth: true; text: root.controller.previewActive ? "暂停预览" : "恢复预览"; quiet: true; enabled: root.controller.cameraOpened && !root.busy; onClicked: root.controller.previewActive ? root.controller.stopPreview() : root.controller.startPreview() }
                    }
                }
                ColumnLayout {
                    objectName: "mainRightColumn"; Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumWidth: 0; spacing: 12
                    CollectCard {
                        Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumHeight: 120; padding: 0
                        ColumnLayout {
                            anchors.fill: parent; spacing: 0
                            RowLayout {
                                Layout.fillWidth: true; Layout.preferredHeight: 44; Layout.leftMargin: 16; Layout.rightMargin: 16
                                HmiText { text: "相机预览"; color: Theme.ink; font.pixelSize: Theme.title; Layout.fillWidth: true }
                                HmiText { text: "十字线仅预览，不写入图片"; font.pixelSize: Theme.meta }
                            }
                            Rectangle {
                                objectName: "captureImageArea"; Layout.fillWidth: true; Layout.fillHeight: true; color: Theme.viewfinder; clip: true
                                property int seenCount: root.seenCount
                                Image { id: cameraImage; anchors.fill: parent; fillMode: Image.PreserveAspectFit; cache: false; asynchronous: false; visible: root.hasFrame }
                                Rectangle { anchors.verticalCenter: parent.verticalCenter; width: parent.width; height: 1; color: Theme.hairline }
                                Rectangle { anchors.horizontalCenter: parent.horizontalCenter; height: parent.height; width: 1; color: Theme.hairline }
                                Rectangle { anchors.centerIn: parent; width: 120; height: 3; color: Theme.danger; opacity: 0.8 }
                                Rectangle { anchors.centerIn: parent; width: 3; height: 120; color: Theme.danger; opacity: 0.8 }
                                HmiText { visible: !root.hasFrame; anchors.horizontalCenter: parent.horizontalCenter; anchors.bottom: parent.bottom; anchors.bottomMargin: 16; text: "等待画面 · 点击「连接相机」"; font.pixelSize: Theme.meta }
                                Rectangle {
                                    visible: root.hasFrame; x: 12; y: 12; height: 28; width: Math.min(parent.width - 24, frameLabel.implicitWidth + 24); radius: 14; color: Theme.chip
                                    HmiText { id: frameLabel; objectName: "frameMetadata"; anchors.fill: parent; anchors.leftMargin: 12; anchors.rightMargin: 12; text: root.frameMetadata; font.pixelSize: Theme.meta; font.family: Theme.mono; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight; wrapMode: Text.NoWrap }
                                }
                                Rectangle { id: flash; objectName: "shutterFlashOverlay"; anchors.fill: parent; color: Theme.white; opacity: 0 }
                                SequentialAnimation { id: shutterFlash; objectName: "shutterFlashAnimation"; NumberAnimation { target: flash; property: "opacity"; from: 0.5; to: 0; duration: 240 } }
                            }
                            HmiText { Layout.fillWidth: true; Layout.margins: 12; text: "已保存 " + root.controller.savedImageCount + " 张 · TCP 点位 " + root.controller.savedPoseCount + (root.controller.previewActive ? " · 预览中" : " · 预览已暂停"); font.family: Theme.mono; font.pixelSize: Theme.meta }
                        }
                    }
                    CollectCard { objectName: "livePoseCard"; Layout.fillWidth: true; Layout.preferredHeight: livePose.implicitHeight + 24; padding: 12; LivePose { id: livePose; width: parent.width; controller: root.controller } }
                    CollectCard {
                        objectName: "logCard"; Layout.fillWidth: true; Layout.preferredHeight: root.height < 740 ? 104 : 150; padding: 12
                        ColumnLayout {
                            anchors.fill: parent; spacing: 4
                            RowLayout { Layout.fillWidth: true; HmiText { text: "运行日志"; color: Theme.ink; Layout.fillWidth: true } HmiText { text: "最近 " + root.logLines.length + " / 1000 行 · 可选择复制"; font.pixelSize: Theme.meta } }
                            ScrollView {
                                id: logScroll; Layout.fillWidth: true; Layout.fillHeight: true; clip: true; contentWidth: availableWidth
                                TextArea { id: logText; objectName: "operationLog"; text: root.logLines.join("\n"); readOnly: true; selectByMouse: true; wrapMode: TextEdit.WrapAnywhere; color: Theme.ink2; font.family: Theme.mono; font.pixelSize: Theme.meta; background: null; padding: 0 }
                            }
                        }
                    }
                }
            }
        }
    }
    SettingsPage { id: settings; objectName: "settingsPage"; anchors.fill: parent; visible: root.page === "settings"; controller: root.controller; locked: root.busy; onLeave: root.page = "main" }
    ManualMovePage { id: manual; objectName: "manualPage"; anchors.fill: parent; visible: root.page === "manual"; controller: root.controller; locked: root.busy; onLeave: root.page = "main" }
    Dialog {
        id: poseDialog; objectName: "savePoseDialog"; anchors.centerIn: parent; width: Math.min(520, root.width - 32); modal: true; title: "保存 TCP 坐标"; closePolicy: Popup.CloseOnEscape
        header: HmiText { objectName: "poseDialogTitle"; text: poseDialog.title; color: Theme.ink; font.pixelSize: Theme.appTitle; padding: 20 }
        onClosed: root.poseEditorOpen = false
        background: Rectangle { color: Theme.surface; border.color: Theme.hairline; radius: Theme.radiusCard }
        contentItem: ColumnLayout {
            spacing: 16
            HmiText { text: "当前工具：" + root.controller.selectedTcp; Layout.fillWidth: true; color: Theme.ink }
            HmiText { text: "点位名称 · 留空使用时间戳"; Layout.fillWidth: true }
            CollectField { id: pointName; objectName: "pointNameInput"; mono: false; Layout.fillWidth: true; enabled: root.tcpReady && !root.busy; onAccepted: root.confirmPose(); Keys.onEscapePressed: { poseDialog.close(); event.accepted = true } }
            HmiText { Layout.fillWidth: true; text: root.controller.statusMessage; font.pixelSize: Theme.meta }
            RowLayout { Item { Layout.fillWidth: true } CollectButton { text: "返回"; quiet: true; onClicked: poseDialog.close() } CollectButton { objectName: "confirmPoseButton"; text: "保存点位"; primary: true; enabled: root.tcpReady && !root.busy; onClicked: root.confirmPose() } }
        }
    }
}
