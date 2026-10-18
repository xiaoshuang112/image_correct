import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "."

ApplicationWindow {
    id: root
    visible: true
    width: 1280
    height: 850
    minimumWidth: 1024
    minimumHeight: 740
    title: "COLLECT · 图像采集 · " + appVersion
    color: Theme.bg
    font.family: Theme.sans
    font.pixelSize: Theme.body
    property var controller: collectionController
    readonly property bool busy: controller.collectionBusy
    readonly property bool ready: controller.cameraOpened && controller.previewActive && !busy
    readonly property bool editing: activeFocusItem !== null && activeFocusItem.cursorPosition !== undefined
    readonly property bool compact: width < 1190
    readonly property int infoWidth: compact ? 288 : 320
    readonly property int ipWidth: compact ? 100 : 110
    readonly property int resolutionWidth: compact ? 46 : 58
    readonly property int connectWidth: compact ? 74 : 86
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
        activityLog.append({stamp: Qt.formatTime(new Date(), "hh:mm:ss"), message: message, failed: failed})
        if (activityLog.count > 200) activityLog.remove(0, activityLog.count - 200)
        if (follow) Qt.callLater(function() { operationLog.positionViewAtEnd() })
    }
    Component.onCompleted: {
        var values = controller.exposureList.split(/[,，;\s]+/).filter(function(value) { return value !== "" })
        for (var i = 0; i < 5; ++i) exposureSlots.append({value: i < values.length ? values[i] : ""})
        recordStatus("程序启动 · Git 版本 " + appGitCommit)
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
        anchors.margins: Theme.margin
        spacing: Theme.gap

        RowLayout {
            Layout.fillWidth: true; Layout.preferredHeight: 36
            spacing: 10
            Rectangle {
                Layout.preferredWidth: 28; Layout.preferredHeight: 28
                radius: Theme.radiusControl; color: Theme.accentSoft; border.width: 1; border.color: Theme.accentLine
                CollectIcon { anchors.centerIn: parent; width: 18; height: 18; name: "aperture"; ink: Theme.accent }
            }
            Text { text: "图像采集"; color: Theme.ink; font.pixelSize: Theme.appTitle; font.weight: Font.Medium }
            Rectangle { Layout.preferredWidth: 3; Layout.preferredHeight: 3; radius: 1.5; color: Theme.ink3 }
            Text { text: "多曝光融合采集"; color: Theme.ink2; font.pixelSize: Theme.body }
            Item { Layout.fillWidth: true }
            Text { text: "COLLECT · " + appGitCommit; color: Theme.ink3; font.family: Theme.mono; font.pixelSize: Theme.meta }
        }

        RowLayout {
            id: controlDeck
            objectName: "controlDeck"
            readonly property real availableWidth: width - spacing * 2
            readonly property real contentHeight: Math.max(deviceControls.implicitHeight, calibrationControls.implicitHeight, captureControls.implicitHeight) + Theme.padding * 2
            Layout.fillWidth: true
            Layout.minimumHeight: contentHeight; Layout.preferredHeight: contentHeight; Layout.maximumHeight: contentHeight
            spacing: Theme.gap
            CollectCard {
                objectName: "deviceCard"
                Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumWidth: 0
                Layout.preferredWidth: controlDeck.availableWidth * (root.compact ? 0.44 : 0.42)
                implicitHeight: deviceControls.implicitHeight + Theme.padding * 2
                ColumnLayout {
                    id: deviceControls
                    anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
                    height: implicitHeight; spacing: 8
                    CollectCardHeader { number: "01"; title: "设备连接"; tint: Theme.section1; tintSoft: Theme.section1Soft }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 5
                        Text { text: "相机 IP"; color: Theme.ink2; font.pixelSize: Theme.meta; Layout.preferredWidth: 46 }
                        CollectField { objectName: "cameraIpInput"; text: root.controller.cameraIp; leftPadding: 7; rightPadding: 7; Layout.preferredWidth: root.ipWidth; Layout.maximumWidth: root.ipWidth; enabled: !root.controller.cameraOpened && !root.busy; onEditingFinished: root.controller.cameraIp = text }
                        Text { text: "分辨率"; color: Theme.ink2; font.pixelSize: Theme.meta }
                        CollectField { objectName: "resolutionWidthInput"; text: root.controller.previewWidth; Layout.preferredWidth: root.resolutionWidth; Layout.maximumWidth: root.resolutionWidth; Layout.minimumWidth: 42; leftPadding: 5; rightPadding: 5; validator: IntValidator { bottom: 1; top: 32768 } enabled: !root.controller.cameraOpened && !root.busy; onEditingFinished: if (acceptableInput) root.controller.previewWidth = parseInt(text) }
                        Text { text: "×"; color: Theme.ink3; font.pixelSize: Theme.meta }
                        CollectField { objectName: "resolutionHeightInput"; text: root.controller.previewHeight; Layout.preferredWidth: root.resolutionWidth; Layout.maximumWidth: root.resolutionWidth; Layout.minimumWidth: 42; leftPadding: 5; rightPadding: 5; validator: IntValidator { bottom: 1; top: 32768 } enabled: !root.controller.cameraOpened && !root.busy; onEditingFinished: if (acceptableInput) root.controller.previewHeight = parseInt(text) }
                        Item { Layout.fillWidth: true; Layout.minimumWidth: 0 }
                        CollectButton { objectName: "openCameraButton"; Layout.preferredWidth: root.connectWidth; Layout.maximumWidth: root.connectWidth; leftPadding: 4; rightPadding: 4; positive: root.controller.cameraOpened; text: root.controller.cameraOpened ? "断开相机" : "连接并预览"; enabled: !root.busy; onClicked: { if (root.controller.cameraOpened) root.controller.closeCamera(); else { root.controller.openCamera(); if (root.controller.cameraOpened) root.controller.startPreview() } } }
                    }
                    Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: Theme.hairline }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 5
                        Text { text: "机器人 IP"; color: Theme.ink2; font.pixelSize: Theme.meta; Layout.preferredWidth: 46 }
                        CollectField { objectName: "robotIpInput"; text: root.controller.robotIp; leftPadding: 7; rightPadding: 7; Layout.preferredWidth: root.ipWidth; Layout.maximumWidth: root.ipWidth; enabled: !root.controller.robotConnected && !root.busy; onEditingFinished: root.controller.robotIp = text }
                        Item { Layout.fillWidth: true; Layout.minimumWidth: 0 }
                        CollectButton { objectName: "levelButton"; text: "一键调平"; iconName: "level"; leftPadding: 9; rightPadding: 9; enabled: root.ready && root.controller.robotConnected; onClicked: root.controller.levelCamera() }
                        CollectButton { objectName: "stopButton"; text: "停止"; leftPadding: 9; rightPadding: 9; danger: true; enabled: root.controller.collectionBusy; onClicked: root.controller.cancelCollectionOperation() }
                        CollectButton { objectName: "connectRobotButton"; Layout.preferredWidth: root.connectWidth; Layout.maximumWidth: root.connectWidth; leftPadding: 4; rightPadding: 4; positive: root.controller.robotConnected; text: root.controller.robotConnected ? "断开机器人" : "连接机器人"; enabled: !root.busy; onClicked: root.controller.robotConnected ? root.controller.disconnectRobot() : root.controller.connectRobot() }
                    }
                }
            }
            CollectCard {
                objectName: "calibrationCard"
                Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumWidth: 0
                Layout.preferredWidth: controlDeck.availableWidth * (root.compact ? 0.20 : 0.18)
                implicitHeight: calibrationControls.implicitHeight + Theme.padding * 2
                ColumnLayout {
                    id: calibrationControls
                    anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
                    height: implicitHeight; spacing: 8
                    CollectCardHeader { number: "02"; title: "图像校准"; tint: Theme.section2; tintSoft: Theme.section2Soft }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 4
                        Text { text: "预览曝光"; color: Theme.ink2; font.pixelSize: Theme.meta }
                        CollectField {
                            id: previewExposure; objectName: "previewExposureInput"
                            text: root.controller.cameraExposureUs; leftPadding: 6; rightPadding: 6
                            Layout.preferredWidth: root.compact ? 64 : 76; Layout.maximumWidth: root.compact ? 64 : 76; Layout.minimumWidth: 50
                            validator: IntValidator { bottom: 1; top: 1000000 }
                            enabled: !root.busy
                        }
                        Text { text: "μs"; color: Theme.ink3; font.family: Theme.mono; font.pixelSize: Theme.meta }
                        Item { Layout.fillWidth: true; Layout.minimumWidth: 0 }
                        CollectButton { objectName: "applyPreviewExposureButton"; text: "应用"; leftPadding: 5; rightPadding: 5; Layout.preferredWidth: 36; Layout.maximumWidth: 36; enabled: !root.busy && previewExposure.acceptableInput; onClicked: root.controller.adjustCameraExposure(parseInt(previewExposure.text)) }
                    }
                    Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: Theme.hairline }
                    CollectButton {
                        objectName: "undistortionButton"
                        Layout.fillWidth: true
                        text: root.controller.undistortionEnabled ? "去畸变已启用" : "一键去畸变"
                        iconName: "undistort"; positive: root.controller.undistortionEnabled
                        leftPadding: 8; rightPadding: 8
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
                implicitHeight: captureControls.implicitHeight + Theme.padding * 2
                ColumnLayout {
                    id: captureControls
                    anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
                    height: implicitHeight; spacing: 8
                    CollectCardHeader {
                        number: "03"; title: "数据采集"
                        Text { text: "融合曝光 · μs"; color: Theme.ink3; font.family: Theme.mono; font.pixelSize: Theme.meta }
                        CollectButton { objectName: "collectionFolderButton"; text: "图片目录"; quiet: true; iconName: "folder"; implicitHeight: 20; leftPadding: 0; rightPadding: 0; implicitWidth: contentItem.implicitWidth; onClicked: root.controller.openCollectionFolder() }
                    }
                    RowLayout {
                        objectName: "exposureRow"
                        Layout.fillWidth: true; spacing: 6
                        Repeater {
                            model: exposureSlots
                            CollectField {
                                objectName: index === 0 ? "exposureInput" : "exposureInput" + (index + 1)
                                text: value; placeholderText: "可选"; leftPadding: 3; rightPadding: 3
                                horizontalAlignment: TextInput.AlignHCenter
                                Layout.fillWidth: true; Layout.preferredWidth: 1; Layout.minimumWidth: 42
                                validator: IntValidator { bottom: 1; top: 1000000 }
                                enabled: !root.busy
                                onTextEdited: exposureSlots.setProperty(index, "value", text)
                                onEditingFinished: root.saveExposures()
                            }
                        }
                    }
                    Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: Theme.hairline }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 6
                        CollectButton { objectName: "captureButton"; text: "拍照保存"; iconName: "shutter"; primary: true; leftPadding: 8; rightPadding: 8; Layout.fillWidth: true; Layout.preferredWidth: 1; Layout.minimumWidth: 0; enabled: root.ready; onClicked: root.takePhoto() }
                        CollectButton { objectName: "savePoseButton"; text: "保存法兰位姿"; iconName: "save"; leftPadding: 8; rightPadding: 8; Layout.fillWidth: true; Layout.preferredWidth: 1; Layout.minimumWidth: 0; enabled: root.controller.robotConnected && !root.busy; onClicked: { root.poseEditorOpen = true; pointName.text = ""; pointName.forceActiveFocus() } }
                    }
                    Rectangle {
                        visible: root.poseEditorOpen
                        Layout.fillWidth: true; implicitHeight: 48
                        color: Theme.sunken; border.width: 1; border.color: Theme.accentLine; radius: 8
                        RowLayout {
                            anchors.fill: parent; anchors.margins: 7; spacing: 6
                            CollectField { id: pointName; objectName: "pointNameInput"; mono: false; Layout.fillWidth: true; Layout.minimumWidth: 40; leftPadding: 8; rightPadding: 8; enabled: !root.busy; placeholderText: "点位名称（留空按时间戳）"; onAccepted: if (confirmPose.enabled) root.controller.saveFlangePose(text); Keys.onEscapePressed: root.poseEditorOpen = false }
                            CollectButton { id: confirmPose; objectName: "confirmPoseButton"; text: "确认保存"; leftPadding: 8; rightPadding: 8; enabled: root.controller.robotConnected && !root.busy; onClicked: root.controller.saveFlangePose(pointName.text) }
                            CollectButton { text: "取消"; leftPadding: 8; rightPadding: 8; onClicked: root.poseEditorOpen = false }
                        }
                    }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true; Layout.fillHeight: true; spacing: Theme.gap
            CollectCard {
                objectName: "viewfinder"
                Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumWidth: 0
                ColumnLayout {
                    anchors.fill: parent; spacing: 10
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        Text { text: "实时预览"; color: Theme.ink; font.pixelSize: Theme.title; font.weight: Font.Medium }
                        Rectangle {
                            id: statusChip
                            readonly property bool live: root.controller.previewActive
                            readonly property bool paused: root.controller.cameraOpened && !live
                            readonly property color tone: live ? Theme.ok : paused ? Theme.accent : Theme.ink2
                            Layout.preferredHeight: 20; Layout.preferredWidth: chipRow.implicitWidth + 16
                            radius: Theme.radiusSmall; color: live ? Theme.okSoft : paused ? Theme.accentSoft : Theme.chip
                            Row {
                                id: chipRow
                                anchors.centerIn: parent; spacing: 6
                                Rectangle { width: 6; height: 6; radius: 3; anchors.verticalCenter: parent.verticalCenter; color: statusChip.live ? Theme.ok : statusChip.paused ? Theme.accent : Theme.ink3 }
                                Text { text: statusChip.live ? "LIVE" : statusChip.paused ? "PAUSED" : "OFFLINE"; color: statusChip.tone; font.family: Theme.mono; font.pixelSize: 10; font.letterSpacing: 1; anchors.verticalCenter: parent.verticalCenter }
                            }
                        }
                        Text {
                            objectName: "undistortionStatus"
                            Layout.fillWidth: true; Layout.minimumWidth: 0
                            text: root.controller.undistortionEnabled ? "去畸变已启用 · 预览 / 拍照" : root.controller.undistortionReady ? "去畸变参数已加载 · 未启用" : "去畸变参数不可用"
                            color: root.controller.undistortionEnabled ? Theme.ok : Theme.ink2
                            font.pixelSize: Theme.meta; elide: Text.ElideRight
                        }
                        CollectButton { objectName: "previewButton"; text: root.controller.previewActive ? "暂停预览" : "开始预览"; iconName: root.controller.previewActive ? "pause" : "play"; implicitHeight: 28; leftPadding: 10; rightPadding: 10; enabled: root.controller.cameraOpened && !root.busy; onClicked: root.controller.previewActive ? root.controller.stopPreview() : root.controller.startPreview() }
                    }
                    Rectangle {
                        id: imageArea
                        objectName: "captureImageArea"
                        // History is assigned explicitly, never bound to the live count.
                        property int seenCount: 0
                        property var countController: null
                        property bool countHistoryReady: false
                        function resetCountHistory() {
                            shutterFlash.stop()
                            flash.opacity = 0
                            countController = root.controller
                            seenCount = root.controller ? root.controller.savedImageCount : 0
                            countHistoryReady = true
                        }
                        function observeSavedImageCount() {
                            if (!countHistoryReady || countController !== root.controller) {
                                resetCountHistory()
                                return
                            }
                            var count = root.controller.savedImageCount
                            if (count < seenCount) {
                                // A reset starts a fresh baseline, with no flash.
                                resetCountHistory()
                                return
                            }
                            if (count > seenCount) shutterFlash.restart()
                            seenCount = count
                        }
                        Component.onCompleted: resetCountHistory()
                        Connections {
                            target: root
                            function onControllerChanged() { imageArea.resetCountHistory() }
                        }
                        Layout.fillWidth: true; Layout.fillHeight: true
                        color: Theme.viewfinder; radius: 6; clip: true
                        Connections {
                            target: root.controller
                            function onSavedImageCountChanged() {
                                imageArea.observeSavedImageCount()
                            }
                        }
                        Image { id: cameraImage; anchors.fill: parent; fillMode: Image.PreserveAspectFit; cache: false; visible: root.controller.cameraOpened; source: "image://camera/current" }
                        Item {
                            id: frame
                            anchors.centerIn: parent
                            width: cameraImage.visible && cameraImage.paintedWidth > 0 ? cameraImage.paintedWidth : parent.width
                            height: cameraImage.visible && cameraImage.paintedHeight > 0 ? cameraImage.paintedHeight : parent.height
                            Repeater {
                                model: 2
                                Rectangle { visible: root.controller.previewActive; x: Math.round(frame.width * (index + 1) / 3); width: 1; height: frame.height; color: Theme.gridLine }
                            }
                            Repeater {
                                model: 2
                                Rectangle { visible: root.controller.previewActive; y: Math.round(frame.height * (index + 1) / 3); width: frame.width; height: 1; color: Theme.gridLine }
                            }
                            Repeater {
                                model: 4
                                Item {
                                    visible: root.controller.cameraOpened
                                    width: 18; height: 18
                                    x: index % 2 === 0 ? 12 : frame.width - 30
                                    y: index < 2 ? 12 : frame.height - 30
                                    Rectangle { y: index < 2 ? 0 : 16; width: 18; height: 2; color: Theme.bracket }
                                    Rectangle { x: index % 2 === 0 ? 0 : 16; width: 2; height: 18; color: Theme.bracket }
                                }
                            }
                            Item {
                                anchors.centerIn: parent; width: 24; height: 24; visible: root.controller.previewActive
                                Rectangle { x: 0; y: 11.5; width: 9; height: 1; color: Theme.reticle }
                                Rectangle { x: 15; y: 11.5; width: 9; height: 1; color: Theme.reticle }
                                Rectangle { x: 11.5; y: 0; width: 1; height: 9; color: Theme.reticle }
                                Rectangle { x: 11.5; y: 15; width: 1; height: 9; color: Theme.reticle }
                            }
                            Rectangle {
                                anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.leftMargin: 20; anchors.bottomMargin: 20
                                height: 22; width: modelText.implicitWidth + 16; radius: Theme.radiusSmall; color: Theme.overlay; visible: root.controller.cameraOpened
                                Text { id: modelText; anchors.centerIn: parent; text: root.controller.cameraModel; color: Theme.ink2; font.family: Theme.mono; font.pixelSize: Theme.meta }
                            }
                            Rectangle {
                                anchors.right: parent.right; anchors.bottom: parent.bottom; anchors.rightMargin: 20; anchors.bottomMargin: 20
                                height: 22; width: infoText.implicitWidth + 16; radius: Theme.radiusSmall; color: Theme.overlay
                                Text {
                                    id: infoText; anchors.centerIn: parent; color: Theme.ink2; font.family: Theme.mono; font.pixelSize: Theme.meta
                                    text: root.controller.cameraOpened ? root.controller.cameraActualWidth + "×" + root.controller.cameraActualHeight + "  ·  " + root.controller.cameraExposureUs + " μs"
                                        : "EXP " + root.controller.cameraExposureUs + " μs"
                                }
                            }
                        }
                        Rectangle {
                            anchors.centerIn: parent; width: pausedRow.implicitWidth + 28; height: 34; radius: 6
                            color: Theme.overlayStrong; border.width: 1; border.color: Theme.line; visible: root.controller.cameraOpened && !root.controller.previewActive && !root.busy
                            Row {
                                id: pausedRow
                                anchors.centerIn: parent; spacing: 8
                                CollectIcon { width: 16; height: 16; name: "pause"; ink: Theme.accent; anchors.verticalCenter: parent.verticalCenter }
                                Text { text: "预览已暂停"; color: Theme.ink; font.pixelSize: Theme.body; anchors.verticalCenter: parent.verticalCenter }
                            }
                        }
                        Column {
                            anchors.centerIn: parent; spacing: 10; visible: !root.controller.cameraOpened
                            Rectangle {
                                width: 64; height: 64; radius: 32; color: "transparent"; border.width: 1; border.color: Theme.line
                                anchors.horizontalCenter: parent.horizontalCenter
                                CollectIcon { anchors.centerIn: parent; width: 30; height: 30; name: "aperture"; ink: Theme.ink3 }
                            }
                            Item { width: 1; height: 2 }
                            Text { text: "等待相机画面"; color: Theme.ink; font.pixelSize: 16; anchors.horizontalCenter: parent.horizontalCenter }
                            Text { text: "连接相机，即可开始预览与拍照"; color: Theme.ink2; font.pixelSize: Theme.body; anchors.horizontalCenter: parent.horizontalCenter }
                            Text { text: "TARGET " + root.controller.cameraIp; color: Theme.ink3; font.family: Theme.mono; font.pixelSize: Theme.meta; anchors.horizontalCenter: parent.horizontalCenter }
                        }
                        Rectangle {
                            anchors.fill: parent; color: Theme.overlay; visible: root.busy
                            Column {
                                anchors.centerIn: parent; spacing: 10
                                CollectIcon {
                                    width: 36; height: 36; name: "spinner"; ink: Theme.accent; anchors.horizontalCenter: parent.horizontalCenter
                                    NumberAnimation on rotation { from: 0; to: 360; duration: 900; loops: Animation.Infinite; running: root.busy }
                                }
                                Text { text: "正在执行操作…"; color: Theme.ink; font.pixelSize: Theme.body; anchors.horizontalCenter: parent.horizontalCenter }
                            }
                        }
                        Rectangle {
                            id: flash
                            objectName: "shutterFlashOverlay"
                            anchors.fill: parent; radius: 6; color: "white"; opacity: 0
                            NumberAnimation { id: shutterFlash; objectName: "shutterFlashAnimation"; target: flash; property: "opacity"; from: 0.22; to: 0; duration: 260; easing.type: Easing.OutCubic }
                        }
                    }
                }
            }
            ColumnLayout {
                id: sidebar
                Layout.preferredWidth: root.infoWidth; Layout.minimumWidth: root.infoWidth; Layout.maximumWidth: root.infoWidth
                Layout.fillHeight: true; spacing: Theme.gap
                CollectCard {
                    id: sessionCard
                    objectName: "sessionCard"
                    Layout.fillWidth: true; implicitHeight: sessionInfo.implicitHeight + Theme.padding * 2
                    ColumnLayout {
                        id: sessionInfo
                        anchors.fill: parent; spacing: 8
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "采集概况"; color: Theme.ink; font.pixelSize: Theme.title; font.weight: Font.Medium }
                            Item { Layout.fillWidth: true }
                            Text { text: root.archiveDate; color: Theme.ink3; font.family: Theme.mono; font.pixelSize: Theme.meta }
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 16
                            ColumnLayout {
                                Layout.fillWidth: true; spacing: 0
                                Text { objectName: "savedImageCount"; text: root.controller.savedImageCount; color: Theme.ink; font.family: Theme.mono; font.pixelSize: Theme.stat; font.weight: Font.Light }
                                Text { text: "本次采集 / 张"; color: Theme.ink2; font.pixelSize: Theme.meta }
                            }
                            Rectangle { implicitWidth: 1; Layout.preferredHeight: 44; color: Theme.hairline }
                            ColumnLayout {
                                Layout.fillWidth: true; spacing: 0
                                Text { objectName: "savedPoseCount"; text: root.controller.savedPoseCount; color: Theme.ink; font.family: Theme.mono; font.pixelSize: Theme.stat; font.weight: Font.Light }
                                Text { text: "独立点位 / 个"; color: Theme.ink2; font.pixelSize: Theme.meta }
                            }
                        }
                        Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: Theme.hairline }
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "归档目录"; color: Theme.ink2; font.pixelSize: Theme.meta }
                            Text { Layout.fillWidth: true; Layout.minimumWidth: 0; horizontalAlignment: Text.AlignRight; text: root.controller.outputDirectory.split("/").pop() + "/" + root.archiveDate + "/"; color: Theme.ink3; font.family: Theme.mono; font.pixelSize: Theme.meta; elide: Text.ElideLeft }
                        }
                    }
                }
                CollectCard {
                    objectName: "recentPhotoCard"
                    Layout.fillWidth: true; implicitHeight: recentPhoto.implicitHeight + Theme.padding * 2
                    ColumnLayout {
                        id: recentPhoto
                        anchors.fill: parent; spacing: 8
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "最近图片"; color: Theme.ink; font.pixelSize: Theme.title; font.weight: Font.Medium }
                            Item { Layout.fillWidth: true }
                            Text { text: root.controller.lastCapturePath ? "已保存" : "暂无图片"; color: Theme.ink3; font.pixelSize: Theme.meta }
                        }
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: Math.max(80, Math.min((root.infoWidth - Theme.padding * 2) * 0.56, root.height - Theme.margin * 2 - 36 - controlDeck.contentHeight - 30 - Theme.gap * 3 - sessionCard.implicitHeight - Theme.gap * 2 - 96 - 106))
                            radius: 6; color: Theme.sunken; border.width: 1; border.color: Theme.hairline; clip: true
                            Image { objectName: "recentPhotoImage"; anchors.fill: parent; anchors.margins: 1; source: root.controller.lastCapturePath ? "file://" + root.controller.lastCapturePath : ""; fillMode: Image.PreserveAspectFit }
                            Column {
                                anchors.centerIn: parent; spacing: 8; visible: !root.controller.lastCapturePath
                                CollectIcon { anchors.horizontalCenter: parent.horizontalCenter; name: "aperture"; ink: Theme.ink3; width: 22; height: 22 }
                                Text { text: "保存后显示缩略图"; color: Theme.ink3; font.pixelSize: Theme.meta }
                            }
                        }
                        Text { Layout.fillWidth: true; Layout.minimumWidth: 0; text: root.controller.lastCapturePath ? root.controller.lastCapturePath.split("/").pop() : "等待第一张图片"; color: Theme.ink2; font.family: Theme.mono; font.pixelSize: Theme.meta; elide: Text.ElideMiddle }
                        Rectangle {
                            Layout.preferredHeight: 18; Layout.preferredWidth: tagText.implicitWidth + 14
                            radius: Theme.radiusSmall; color: Theme.chip
                            Text { id: tagText; anchors.centerIn: parent; text: "MERTENS · PNG"; color: Theme.ink2; font.family: Theme.mono; font.pixelSize: 10 }
                        }
                    }
                }
                CollectCard {
                    objectName: "logCard"
                    Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumHeight: 96
                    ColumnLayout {
                        anchors.fill: parent; spacing: 10
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "操作日志"; color: Theme.ink; font.pixelSize: Theme.title; font.weight: Font.Medium }
                            Item { Layout.fillWidth: true }
                            Text { text: activityLog.count + " 条"; color: Theme.ink3; font.family: Theme.mono; font.pixelSize: Theme.meta }
                        }
                        ListView {
                            id: operationLog; objectName: "operationLog"
                            Layout.fillWidth: true; Layout.fillHeight: true
                            clip: true; spacing: 8; boundsBehavior: Flickable.StopAtBounds; model: activityLog
                            ScrollBar.vertical: ScrollBar { width: 3; policy: ScrollBar.AsNeeded; contentItem: Rectangle { radius: 1.5; color: Theme.line; opacity: parent.active ? 1 : 0.5 } }
                            delegate: Row {
                                width: operationLog.width - 6; spacing: 6
                                Text { width: 58; text: stamp; color: Theme.ink3; font.family: Theme.mono; font.pixelSize: Theme.meta }
                                Text { width: Math.max(1, parent.width - 64); text: message; color: failed ? Theme.danger : index === activityLog.count - 1 ? Theme.ink : Theme.ink2; font.pixelSize: Theme.meta; wrapMode: Text.Wrap; lineHeight: 1.3 }
                            }
                        }
                    }
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true; implicitHeight: 30; color: "transparent"
            Rectangle { anchors.top: parent.top; width: parent.width; height: 1; color: Theme.hairline }
            RowLayout {
                anchors.fill: parent; anchors.topMargin: 1; spacing: 14
                RowLayout {
                    spacing: 6
                    Rectangle { width: 6; height: 6; radius: 3; color: root.controller.cameraOpened ? Theme.ok : Theme.ink3 }
                    Text { text: root.controller.cameraOpened ? "相机已连接" : "相机未连接"; color: Theme.ink2; font.pixelSize: Theme.meta }
                }
                RowLayout {
                    spacing: 6
                    Rectangle { width: 6; height: 6; radius: 3; color: root.controller.robotConnected ? Theme.ok : Theme.ink3 }
                    Text { text: root.controller.robotConnected ? "机器人已连接" : "机器人未连接"; color: Theme.ink2; font.pixelSize: Theme.meta }
                }
                RowLayout {
                    spacing: 5
                    Repeater {
                        model: ["Space", "S", "Ctrl+S"]
                        Row {
                            spacing: 5
                            Rectangle {
                                height: 18; width: keyText.implicitWidth + 12; radius: Theme.radiusSmall; color: Theme.raised; border.width: 1; border.color: Theme.line
                                Text { id: keyText; anchors.centerIn: parent; text: modelData; color: Theme.ink2; font.family: Theme.mono; font.pixelSize: 10 }
                            }
                            Text { visible: index < 2; text: "/"; color: Theme.ink3; font.pixelSize: Theme.meta; anchors.verticalCenter: parent.verticalCenter }
                        }
                    }
                    Text { text: "拍照保存"; color: Theme.ink2; font.pixelSize: Theme.meta; Layout.leftMargin: 3 }
                }
                Text { Layout.fillWidth: true; Layout.minimumWidth: 0; horizontalAlignment: Text.AlignRight; text: root.controller.statusMessage; color: Theme.ink2; font.pixelSize: Theme.meta; elide: Text.ElideMiddle }
            }
        }
    }
}
