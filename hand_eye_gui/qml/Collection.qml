import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: root
    visible: true
    width: 1280
    height: 850
    minimumWidth: 1024
    minimumHeight: 740
    title: "COLLECT · 图像采集 · " + appVersion
    color: "#0d1117"
    font.family: "Noto Sans CJK SC"
    font.pixelSize: 13
    property var controller: calibrationController
    readonly property bool busy: controller.collectionBusy || controller.running
    readonly property bool ready: controller.cameraOpened && controller.previewActive && !busy
    readonly property bool editing: activeFocusItem !== null && activeFocusItem.cursorPosition !== undefined
    readonly property color ink: "#dce6f2"
    readonly property color muted: "#98a9bf"
    readonly property bool compact: width < 1190
    readonly property int infoWidth: compact ? 252 : 264
    readonly property int ipWidth: compact ? 100 : 110
    readonly property int resolutionWidth: compact ? 46 : 58
    readonly property int connectWidth: compact ? 74 : 86
    readonly property color accent: "#78bcff"
    readonly property color surface: "#161c25"
    readonly property color outline: "#2b3441"
    property bool poseEditorOpen: false
    property string today: Qt.formatDate(new Date(), "yyyy-MM-dd")
    readonly property string archiveDate: controller.lastCapturePath
        ? controller.lastCapturePath.split("/").slice(-2, -1).join("") : today
    function saveExposures() {
        var values = []
        for (var i = 0; i < exposureSlots.count; ++i) {
            var value = exposureSlots.get(i).value.trim()
            if (value !== "") values.push(value)
        }
        controller.exposureList = values.join(",")
    }
    function takePhoto() {
        saveExposures()
        controller.capturePhoto()
    }
    function recordStatus(message) {
        if (!message) return
        var follow = operationLog.atYEnd || activityLog.count === 0
        var failed = /失败|异常|无效|超出|超时|不可达|无法/.test(message)
        activityLog.append({stamp: Qt.formatTime(new Date(), "hh:mm:ss"), message: message,
                            tone: failed ? "#cc9191" : "#b7c3d2"})
        if (activityLog.count > 200) activityLog.remove(0, activityLog.count - 200)
        if (follow) Qt.callLater(function() { operationLog.positionViewAtEnd() })
    }
    Component.onCompleted: {
        var values = controller.exposureList.split(/[,，;\s]+/).filter(function(value) { return value !== "" })
        for (var i = 0; i < 5; ++i) exposureSlots.append({value: i < values.length ? values[i] : ""})
        recordStatus(controller.statusMessage)
    }
    Timer { interval: 60000; running: true; repeat: true; onTriggered: root.today = Qt.formatDate(new Date(), "yyyy-MM-dd") }
    ListModel { id: activityLog }
    ListModel { id: exposureSlots }
    Shortcut { sequence: "Space"; autoRepeat: false; context: Qt.WindowShortcut; enabled: root.ready && !root.editing; onActivated: root.takePhoto() }
    Shortcut { sequence: "S"; autoRepeat: false; context: Qt.WindowShortcut; enabled: root.ready && !root.editing; onActivated: root.takePhoto() }
    Shortcut { sequence: "Ctrl+S"; autoRepeat: false; context: Qt.WindowShortcut; enabled: root.ready && !root.editing; onActivated: root.takePhoto() }
    Connections {
        target: root.controller
        function onFrameUpdated() { cameraImage.source = "image://camera/current#" + Date.now() }
        function onStatusMessageChanged() { root.recordStatus(root.controller.statusMessage); if (root.controller.statusMessage.indexOf("点位已保存：") === 0) root.poseEditorOpen = false }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 12

        RowLayout {
            Layout.fillWidth: true; Layout.preferredHeight: 38
            spacing: 10
            Rectangle {
                Layout.preferredWidth: 30; Layout.preferredHeight: 30
                radius: 7; color: "#243d59"
                CollectIcon { anchors.centerIn: parent; width: 19; height: 19; name: "camera"; ink: "#93caff" }
            }
            Text { text: "图像采集"; color: root.ink; font.pixelSize: 16; font.weight: Font.Medium }
            Rectangle { Layout.preferredWidth: 1; Layout.preferredHeight: 16; color: "#354050" }
            Text { text: "多曝光融合采集"; color: root.muted; font.pixelSize: 11 }
            Item { Layout.fillWidth: true }
            Text { text: "COLLECT · " + appVersion; color: root.muted; font.pixelSize: 11 }
        }

        RowLayout {
            id: controlDeck
            objectName: "controlDeck"
            readonly property real availableWidth: width - spacing * 2
            readonly property real contentHeight: Math.max(deviceControls.implicitHeight, calibrationControls.implicitHeight, captureControls.implicitHeight) + 24
            Layout.fillWidth: true
            Layout.minimumHeight: contentHeight; Layout.preferredHeight: contentHeight; Layout.maximumHeight: contentHeight
            spacing: 12
            CollectCard {
                objectName: "deviceCard"
                Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumWidth: 0
                Layout.preferredWidth: controlDeck.availableWidth * (root.compact ? 0.44 : 0.42)
                implicitHeight: deviceControls.implicitHeight + 24
                ColumnLayout {
                    id: deviceControls
                    anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
                    height: implicitHeight; spacing: 8
                    RowLayout {
                        Layout.fillWidth: true; Layout.preferredHeight: 22
                        Rectangle { implicitWidth: 25; implicitHeight: 22; radius: 5; color: root.accent; Text { anchors.centerIn: parent; text: "01"; color: "#12283f"; font.pixelSize: 12; font.weight: Font.Medium } }
                        Text { text: "设备连接"; color: root.ink; font.pixelSize: 13; font.weight: Font.Medium }
                        Item { Layout.fillWidth: true }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 5
                        Text { text: "相机 IP"; color: root.muted; font.pixelSize: 11; Layout.preferredWidth: 44 }
                        CollectField { objectName: "cameraIpInput"; text: root.controller.cameraIp; font.pixelSize: 11; leftPadding: 6; rightPadding: 6; Layout.preferredWidth: root.ipWidth; Layout.maximumWidth: root.ipWidth; enabled: !root.controller.cameraOpened && !root.busy; onEditingFinished: root.controller.cameraIp = text }
                        Text { text: "分辨率"; color: root.muted; font.pixelSize: 11 }
                        CollectField { objectName: "resolutionWidthInput"; text: root.controller.previewWidth; Layout.preferredWidth: root.resolutionWidth; Layout.maximumWidth: root.resolutionWidth; Layout.minimumWidth: 42; leftPadding: 5; rightPadding: 5; font.pixelSize: 11; validator: IntValidator { bottom: 1; top: 32768 } enabled: !root.controller.cameraOpened && !root.busy; onEditingFinished: if (acceptableInput) root.controller.previewWidth = parseInt(text) }
                        Text { text: "×"; color: root.muted; font.pixelSize: 11 }
                        CollectField { objectName: "resolutionHeightInput"; text: root.controller.previewHeight; Layout.preferredWidth: root.resolutionWidth; Layout.maximumWidth: root.resolutionWidth; Layout.minimumWidth: 42; leftPadding: 5; rightPadding: 5; font.pixelSize: 11; validator: IntValidator { bottom: 1; top: 32768 } enabled: !root.controller.cameraOpened && !root.busy; onEditingFinished: if (acceptableInput) root.controller.previewHeight = parseInt(text) }
                        Item { Layout.fillWidth: true; Layout.minimumWidth: 0 }
                        CollectButton { objectName: "openCameraButton"; Layout.preferredWidth: root.connectWidth; Layout.maximumWidth: root.connectWidth; leftPadding: 4; rightPadding: 4; font.pixelSize: 11; text: root.controller.cameraOpened ? "断开相机" : "连接并预览"; enabled: !root.busy; onClicked: { if (root.controller.cameraOpened) root.controller.closeCamera(); else { root.controller.openCamera(); if (root.controller.cameraOpened) root.controller.startPreview() } } }
                    }
                    Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: root.outline }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 5
                        Text { text: "机器人 IP"; color: root.muted; font.pixelSize: 11; Layout.preferredWidth: 44 }
                        CollectField { objectName: "robotIpInput"; text: root.controller.robotIp; font.pixelSize: 11; leftPadding: 6; rightPadding: 6; Layout.preferredWidth: root.ipWidth; Layout.maximumWidth: root.ipWidth; enabled: !root.controller.robotConnected && !root.busy; onEditingFinished: root.controller.robotIp = text }
                        Item { Layout.fillWidth: true; Layout.minimumWidth: 0 }
                        CollectButton { objectName: "levelButton"; text: "一键调平"; leftPadding: 7; rightPadding: 7; font.pixelSize: 11; enabled: root.ready && root.controller.robotConnected; onClicked: root.controller.levelCamera() }
                        CollectButton { objectName: "stopButton"; text: "停止"; leftPadding: 7; rightPadding: 7; font.pixelSize: 11; danger: true; enabled: root.controller.collectionBusy; onClicked: root.controller.cancelCollectionOperation() }
                        CollectButton { objectName: "connectRobotButton"; Layout.preferredWidth: root.connectWidth; Layout.maximumWidth: root.connectWidth; leftPadding: 4; rightPadding: 4; font.pixelSize: 11; text: root.controller.robotConnected ? "断开机器人" : "连接机器人"; enabled: !root.busy; onClicked: root.controller.robotConnected ? root.controller.disconnectRobot() : root.controller.connectRobot() }
                    }
                }
            }
            CollectCard {
                objectName: "calibrationCard"
                Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumWidth: 0
                Layout.preferredWidth: controlDeck.availableWidth * (root.compact ? 0.20 : 0.18)
                implicitHeight: calibrationControls.implicitHeight + 24
                ColumnLayout {
                    id: calibrationControls
                    anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
                    height: implicitHeight; spacing: 8
                    RowLayout {
                        Layout.fillWidth: true; Layout.preferredHeight: 22
                        Rectangle { implicitWidth: 25; implicitHeight: 22; radius: 5; color: root.accent; Text { anchors.centerIn: parent; text: "02"; color: "#12283f"; font.pixelSize: 12; font.weight: Font.Medium } }
                        Text { text: "图像校准"; color: root.ink; font.pixelSize: 13; font.weight: Font.Medium }
                        Item { Layout.fillWidth: true }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 4
                        Text { text: "预览曝光"; color: root.muted; font.pixelSize: 11 }
                        CollectField {
                            id: previewExposure; objectName: "previewExposureInput"
                            text: root.controller.cameraExposureUs; font.pixelSize: 11; leftPadding: 5; rightPadding: 5
                            Layout.preferredWidth: root.compact ? 64 : 76; Layout.maximumWidth: root.compact ? 64 : 76; Layout.minimumWidth: 58
                            validator: IntValidator { bottom: 1; top: 1000000 }
                            enabled: !root.busy
                        }
                        Text { text: "μs"; color: root.muted; font.pixelSize: 11 }
                        Item { Layout.fillWidth: true; Layout.minimumWidth: 0 }
                        CollectButton { objectName: "applyPreviewExposureButton"; text: "应用"; font.pixelSize: 11; leftPadding: 5; rightPadding: 5; Layout.preferredWidth: 36; Layout.maximumWidth: 36; enabled: !root.busy && previewExposure.acceptableInput; onClicked: root.controller.adjustCameraExposure(parseInt(previewExposure.text)) }
                    }
                    Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: root.outline }
                    CollectButton {
                        objectName: "undistortionButton"
                        Layout.fillWidth: true
                        text: root.controller.undistortionEnabled ? "去畸变已启用" : "一键去畸变"
                        iconName: "undistort"; positive: root.controller.undistortionEnabled
                        font.pixelSize: 11
                        enabled: !root.busy && root.controller.cameraOpened && (root.controller.undistortionReady || root.controller.undistortionEnabled)
                        onClicked: root.controller.toggleUndistortion()
                        ToolTip.visible: hovered
                        ToolTip.text: root.controller.undistortionStatus
                    }
                }
            }
            CollectCard {
                objectName: "captureCard"
                Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumWidth: 0
                Layout.preferredWidth: controlDeck.availableWidth * (root.compact ? 0.36 : 0.40)
                implicitHeight: captureControls.implicitHeight + 24
                ColumnLayout {
                    id: captureControls
                    anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
                    height: implicitHeight; spacing: 8
                    RowLayout {
                        Layout.fillWidth: true; Layout.preferredHeight: 22; spacing: 7
                        Rectangle { implicitWidth: 25; implicitHeight: 22; radius: 5; color: root.accent; Text { anchors.centerIn: parent; text: "03"; color: "#12283f"; font.pixelSize: 12; font.weight: Font.Medium } }
                        Text { text: "数据采集"; color: root.ink; font.pixelSize: 13; font.weight: Font.Medium }
                        Item { Layout.fillWidth: true; Layout.minimumWidth: 0 }
                        Text { text: "融合曝光 · μs"; color: root.muted; font.pixelSize: 11 }
                        CollectButton { objectName: "collectionFolderButton"; text: "图片目录"; quiet: true; iconName: "folder"; font.pixelSize: 11; implicitHeight: 22; leftPadding: 0; rightPadding: 0; implicitWidth: contentItem.implicitWidth; onClicked: root.controller.openCollectionFolder() }
                    }
                    RowLayout {
                        objectName: "exposureRow"
                        Layout.fillWidth: true; spacing: 6
                        Repeater {
                            model: exposureSlots
                            CollectField {
                                objectName: index === 0 ? "exposureInput" : "exposureInput" + (index + 1)
                                text: value; placeholderText: "可选"; font.pixelSize: 11; leftPadding: 3; rightPadding: 3
                                horizontalAlignment: TextInput.AlignHCenter
                                Layout.fillWidth: true; Layout.preferredWidth: 1; Layout.minimumWidth: 42
                                validator: IntValidator { bottom: 1; top: 1000000 }
                                enabled: !root.busy
                                onTextEdited: exposureSlots.setProperty(index, "value", text)
                                onEditingFinished: root.saveExposures()
                            }
                        }
                    }
                    Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: root.outline }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 6
                        CollectButton { objectName: "captureButton"; text: "拍照保存"; iconName: "camera"; primary: true; leftPadding: 8; rightPadding: 8; font.pixelSize: 11; Layout.fillWidth: true; Layout.preferredWidth: 1; Layout.minimumWidth: 0; enabled: root.ready; onClicked: root.takePhoto() }
                        CollectButton { objectName: "savePoseButton"; text: "保存法兰位姿"; iconName: "save"; leftPadding: 8; rightPadding: 8; font.pixelSize: 11; Layout.fillWidth: true; Layout.preferredWidth: 1; Layout.minimumWidth: 0; enabled: root.controller.robotConnected && !root.busy; onClicked: { root.poseEditorOpen = true; pointName.text = ""; pointName.forceActiveFocus() } }
                    }
                    Rectangle {
                        visible: root.poseEditorOpen
                        Layout.fillWidth: true; implicitHeight: 52
                        color: "#101923"; border.color: "#354357"; radius: 7
                        RowLayout {
                            anchors.fill: parent; anchors.margins: 7; spacing: 6
                            CollectField { id: pointName; objectName: "pointNameInput"; Layout.fillWidth: true; Layout.minimumWidth: 40; font.pixelSize: 11; leftPadding: 6; rightPadding: 6; enabled: !root.busy; placeholderText: "点位名称（留空按时间戳）"; onAccepted: if (confirmPose.enabled) root.controller.saveFlangePose(text); Keys.onEscapePressed: root.poseEditorOpen = false }
                            CollectButton { id: confirmPose; objectName: "confirmPoseButton"; text: "确认保存"; font.pixelSize: 11; leftPadding: 6; rightPadding: 6; enabled: root.controller.robotConnected && !root.busy; onClicked: root.controller.saveFlangePose(pointName.text) }
                            CollectButton { text: "取消"; font.pixelSize: 11; leftPadding: 6; rightPadding: 6; onClicked: root.poseEditorOpen = false }
                        }
                    }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true; Layout.fillHeight: true; spacing: 12
            Rectangle {
                objectName: "viewfinder"
                Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumWidth: 0
                color: root.surface; radius: 9; border.color: root.outline; clip: true
                ColumnLayout {
                    anchors.fill: parent; anchors.margins: 12; spacing: 10
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        Text { text: "实时预览"; color: root.ink; font.pixelSize: 13; font.weight: Font.Medium }
                        Rectangle { width: 6; height: 6; radius: 3; color: root.controller.previewActive ? "#81caa6" : "#73859c" }
                        Text { text: root.controller.previewActive ? "预览中" : root.controller.cameraOpened ? "已暂停" : "未连接"; color: root.controller.previewActive ? "#93cbb0" : root.muted; font.pixelSize: 11 }
                        Text {
                            objectName: "undistortionStatus"
                            Layout.fillWidth: true; Layout.minimumWidth: 0
                            text: root.controller.undistortionEnabled ? "去畸变已启用 · 预览 / 拍照" : root.controller.undistortionReady ? "去畸变参数已加载 · 未启用" : "去畸变参数不可用"
                            color: root.controller.undistortionEnabled ? "#a0e1bf" : root.muted
                            font.pixelSize: 11; elide: Text.ElideRight
                        }
                        CollectButton { objectName: "previewButton"; text: root.controller.previewActive ? "暂停预览" : "开始预览"; iconName: root.controller.previewActive ? "pause" : "play"; implicitHeight: 28; font.pixelSize: 11; leftPadding: 8; rightPadding: 8; enabled: root.controller.cameraOpened && !root.busy; onClicked: root.controller.previewActive ? root.controller.stopPreview() : root.controller.startPreview() }
                    }
                    Rectangle {
                        id: imageArea
                        Layout.fillWidth: true; Layout.fillHeight: true
                        color: "#090d12"; radius: 6; clip: true
                        Image { id: cameraImage; anchors.fill: parent; fillMode: Image.PreserveAspectFit; cache: false; visible: root.controller.cameraOpened; source: "image://camera/current" }
                        Column {
                            anchors.centerIn: parent; spacing: 12; visible: !root.controller.cameraOpened
                            Rectangle {
                                width: 68; height: 68; radius: 16; color: "#172230"; border.color: "#2b3b4e"
                                anchors.horizontalCenter: parent.horizontalCenter
                                CollectIcon { anchors.centerIn: parent; width: 30; height: 30; name: "camera"; ink: "#7892b0" }
                            }
                            Text { text: "等待相机画面"; color: "#c8d2df"; font.pixelSize: 17; anchors.horizontalCenter: parent.horizontalCenter }
                            Text { text: "连接相机，即可开始预览与拍照"; color: "#8294ab"; font.pixelSize: 11; anchors.horizontalCenter: parent.horizontalCenter }
                        }
                        Rectangle { anchors.centerIn: parent; width: 22; height: 1; color: "#70e5edf7"; visible: root.controller.previewActive }
                        Rectangle { anchors.centerIn: parent; width: 1; height: 22; color: "#70e5edf7"; visible: root.controller.previewActive }
                        Rectangle {
                            anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.margins: 10
                            height: 23; width: modelText.implicitWidth + 16; radius: 4; color: "#dd111820"; visible: root.controller.cameraOpened
                            Text { id: modelText; anchors.centerIn: parent; text: root.controller.cameraModel; color: "#bac7d8"; font.pixelSize: 11 }
                        }
                        Rectangle {
                            anchors.centerIn: parent; width: pausedText.implicitWidth + 24; height: 34; radius: 6
                            color: "#dd111820"; border.color: "#4c5b6e"; visible: root.controller.cameraOpened && !root.controller.previewActive && !root.busy
                            Text { id: pausedText; anchors.centerIn: parent; text: "预览已暂停"; color: root.ink; font.pixelSize: 12 }
                        }
                        Rectangle {
                            anchors.fill: parent; color: "#bb111720"; visible: root.busy
                            Column {
                                anchors.centerIn: parent; spacing: 5
                                BusyIndicator { width: 48; height: 48; running: root.busy; anchors.horizontalCenter: parent.horizontalCenter }
                                Text { text: "正在执行操作…"; color: "#d1dbea"; font.pixelSize: 12 }
                            }
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        Text { text: root.controller.cameraOpened ? root.controller.cameraActualWidth + " × " + root.controller.cameraActualHeight + " px · 适应窗口" : "原始分辨率 · 适应窗口"; color: root.muted; font.pixelSize: 11 }
                        Item { Layout.fillWidth: true }
                        Text { text: "预览曝光 " + root.controller.cameraExposureUs + " μs"; color: root.muted; font.pixelSize: 11 }
                    }
                }
            }
            ColumnLayout {
                Layout.preferredWidth: root.infoWidth; Layout.minimumWidth: root.infoWidth; Layout.maximumWidth: root.infoWidth
                Layout.fillHeight: true; spacing: 12
                CollectCard {
                    objectName: "sessionCard"
                    Layout.fillWidth: true; implicitHeight: sessionInfo.implicitHeight + 24
                    ColumnLayout {
                        id: sessionInfo
                        anchors.fill: parent; spacing: 9
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "采集概况"; color: root.ink; font.pixelSize: 13; font.weight: Font.Medium }
                            Item { Layout.fillWidth: true }
                            Text { text: root.archiveDate; color: root.muted; font.pixelSize: 11 }
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 16
                            ColumnLayout {
                                Layout.fillWidth: true; spacing: 1
                                Text { objectName: "savedImageCount"; text: root.controller.savedImageCount; color: root.ink; font.pixelSize: 28; font.weight: Font.Medium }
                                Text { text: "本次采集 / 张"; color: root.muted; font.pixelSize: 11 }
                            }
                            Rectangle { implicitWidth: 1; Layout.preferredHeight: 44; color: root.outline }
                            ColumnLayout {
                                Layout.fillWidth: true; spacing: 1
                                Text { objectName: "savedPoseCount"; text: root.controller.savedPoseCount; color: root.ink; font.pixelSize: 28; font.weight: Font.Medium }
                                Text { text: "独立点位 / 个"; color: root.muted; font.pixelSize: 11 }
                            }
                        }
                        Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: root.outline }
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "归档目录"; color: root.muted; font.pixelSize: 11 }
                            Text { Layout.fillWidth: true; Layout.minimumWidth: 0; horizontalAlignment: Text.AlignRight; text: root.controller.outputDirectory.split("/").pop() + "/" + root.archiveDate + "/"; color: root.muted; font.pixelSize: 11; elide: Text.ElideLeft }
                        }
                    }
                }
                CollectCard {
                    objectName: "recentPhotoCard"
                    Layout.fillWidth: true; implicitHeight: recentPhoto.implicitHeight + 24
                    ColumnLayout {
                        id: recentPhoto
                        anchors.fill: parent; spacing: 8
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "最近图片"; color: root.ink; font.pixelSize: 13; font.weight: Font.Medium }
                            Item { Layout.fillWidth: true }
                            Text { text: root.controller.lastCapturePath ? "已保存" : "暂无图片"; color: root.muted; font.pixelSize: 11 }
                        }
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: (root.infoWidth - 24) / 2
                            radius: 5; color: "#0f151e"; border.color: root.outline; clip: true
                            Image { objectName: "recentPhotoImage"; anchors.fill: parent; anchors.margins: 1; source: root.controller.lastCapturePath ? "file://" + root.controller.lastCapturePath : ""; fillMode: Image.PreserveAspectFit }
                            Column {
                                anchors.centerIn: parent; spacing: 8; visible: !root.controller.lastCapturePath
                                CollectIcon { anchors.horizontalCenter: parent.horizontalCenter; name: "camera"; ink: "#7892b0"; width: 24; height: 24 }
                                Text { text: "保存后显示缩略图"; color: root.muted; font.pixelSize: 11 }
                            }
                        }
                        Text { Layout.fillWidth: true; Layout.minimumWidth: 0; text: root.controller.lastCapturePath ? root.controller.lastCapturePath.split("/").pop() : "等待第一张图片"; color: "#bbcbde"; font.pixelSize: 11; elide: Text.ElideMiddle }
                        Text { text: "多曝光融合 · PNG"; color: root.muted; font.pixelSize: 11 }
                    }
                }
                CollectCard {
                    objectName: "logCard"
                    Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumHeight: 96
                    ColumnLayout {
                        anchors.fill: parent; spacing: 10
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "操作日志"; color: root.ink; font.pixelSize: 13; font.weight: Font.Medium }
                            Item { Layout.fillWidth: true }
                            Text { text: activityLog.count + " 条"; color: root.muted; font.pixelSize: 11 }
                        }
                        ListView {
                            id: operationLog; objectName: "operationLog"
                            Layout.fillWidth: true; Layout.fillHeight: true
                            clip: true; spacing: 10; boundsBehavior: Flickable.StopAtBounds; model: activityLog
                            ScrollBar.vertical: ScrollBar { width: 4; policy: ScrollBar.AsNeeded; contentItem: Rectangle { radius: 2; color: "#455163"; opacity: parent.active ? 1 : 0.4 } }
                            delegate: Row {
                                width: operationLog.width - 6; spacing: 7
                                Text { width: 54; text: stamp; color: "#8094ad"; font.family: "DejaVu Sans Mono"; font.pixelSize: 11 }
                                Text { width: Math.max(1, parent.width - 61); text: message; color: tone; font.pixelSize: 11; wrapMode: Text.Wrap; lineHeight: 1.25 }
                            }
                        }
                    }
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true; implicitHeight: 28; color: "transparent"
            Rectangle { anchors.top: parent.top; width: parent.width; height: 1; color: root.outline }
            RowLayout {
                anchors.fill: parent; anchors.topMargin: 6; spacing: 12
                RowLayout {
                    spacing: 5
                    Rectangle { width: 6; height: 6; radius: 3; color: root.controller.cameraOpened ? "#81caa6" : "#73859c" }
                    Text { text: root.controller.cameraOpened ? "相机已连接" : "相机未连接"; color: root.muted; font.pixelSize: 11 }
                }
                RowLayout {
                    spacing: 5
                    Rectangle { width: 6; height: 6; radius: 3; color: root.controller.robotConnected ? "#81caa6" : "#73859c" }
                    Text { text: root.controller.robotConnected ? "机器人已连接" : "机器人未连接"; color: root.muted; font.pixelSize: 11 }
                }
                Text { text: "空格 / S / Ctrl+S：拍照保存"; color: root.muted; font.pixelSize: 11 }
                Text { Layout.fillWidth: true; Layout.minimumWidth: 0; horizontalAlignment: Text.AlignRight; text: root.controller.statusMessage; color: root.muted; font.pixelSize: 11; elide: Text.ElideMiddle }
            }
        }
    }
}
