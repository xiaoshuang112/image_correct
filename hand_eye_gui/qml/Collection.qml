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
    color: "#0d0e12"
    font.family: "Noto Sans CJK SC"
    font.pixelSize: 13
    property var controller: calibrationController
    readonly property bool busy: controller.collectionBusy || controller.running
    readonly property bool ready: controller.cameraOpened && controller.previewActive && !busy
    readonly property bool editing: activeFocusItem !== null && activeFocusItem.cursorPosition !== undefined
    readonly property color ink: "#dce3ed"
    readonly property color muted: "#9ba6b6"
    readonly property int infoWidth: width < 1100 ? 270 : 290
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
        anchors.margins: 18
        spacing: 12

        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: 38
            spacing: 13
            Rectangle {
                Layout.preferredWidth: 30; Layout.preferredHeight: 30
                radius: 8; color: "#172331"
                CollectIcon { anchors.centerIn: parent; width: 20; height: 20; name: "shutter"; ink: "#809dbc" }
            }
            RowLayout {
                spacing: 9
                Text { text: "COLLECT"; color: root.ink; font.pixelSize: 21; font.weight: Font.DemiBold; font.letterSpacing: 1.8 }
                Text { text: "图像采集工作台"; color: root.muted; font.pixelSize: 12 }
            }
            Item { Layout.fillWidth: true }
            Repeater {
                model: 2
                Rectangle {
                    implicitWidth: chipLabel.implicitWidth + 18; implicitHeight: 28; radius: 6
                    color: index === 0 ? "#122019" : "#14202d"
                    border.color: index === 0 ? "#263e30" : "#2a3c51"
                    Text { id: chipLabel; anchors.centerIn: parent; font.pixelSize: 11; color: "#c0cbd8"; text: index === 0 ? "● 相机 · " + (root.controller.previewActive ? "预览中" : root.controller.cameraOpened ? "已暂停" : "未连接") : "● 机器人 · " + (root.controller.robotConnected ? "已连接" : "未连接") }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 12
            CollectCard {
                Layout.preferredWidth: (root.width - 48) * 0.44
                Layout.fillHeight: true
                implicitHeight: deviceControls.implicitHeight + 28
                ColumnLayout {
                    id: deviceControls
                    anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
                    height: implicitHeight; spacing: 8
                    RowLayout {
                        Layout.fillWidth: true
                        Layout.minimumHeight: 28; Layout.preferredHeight: 28; Layout.maximumHeight: 28
                        Rectangle { implicitWidth: 27; implicitHeight: 21; radius: 5; color: "#69acfa"; Text { anchors.centerIn: parent; text: "01"; color: "#101820"; font.pixelSize: 11; font.bold: true } }
                        Text { text: "设备连接"; color: root.ink; font.pixelSize: 14; font.bold: true }
                        Item { Layout.fillWidth: true }
                        Text { text: "HARDWARE"; color: root.muted; font.pixelSize: 10 }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 6
                        Text { text: "相机 IP"; color: root.muted; font.pixelSize: 11; Layout.preferredWidth: 45 }
                        CollectField { text: root.controller.cameraIp; font.pixelSize: 11; leftPadding: 5; rightPadding: 5; Layout.preferredWidth: 110; enabled: !root.controller.cameraOpened && !root.busy; onEditingFinished: root.controller.cameraIp = text }
                        Text { text: "分辨率"; color: root.muted; font.pixelSize: 11 }
                        CollectField { text: root.controller.previewWidth; Layout.fillWidth: true; Layout.minimumWidth: 39; leftPadding: 3; rightPadding: 3; font.pixelSize: 11; validator: IntValidator { bottom: 1; top: 32768 } enabled: !root.controller.cameraOpened && !root.busy; onEditingFinished: if (acceptableInput) root.controller.previewWidth = parseInt(text) }
                        Text { text: "×"; color: root.muted }
                        CollectField { text: root.controller.previewHeight; Layout.fillWidth: true; Layout.minimumWidth: 39; leftPadding: 3; rightPadding: 3; font.pixelSize: 11; validator: IntValidator { bottom: 1; top: 32768 } enabled: !root.controller.cameraOpened && !root.busy; onEditingFinished: if (acceptableInput) root.controller.previewHeight = parseInt(text) }
                        CollectButton { objectName: "openCameraButton"; Layout.preferredWidth: 86; leftPadding: 4; rightPadding: 4; font.pixelSize: 11; text: root.controller.cameraOpened ? "断开相机" : "连接并预览"; enabled: !root.busy; onClicked: { if (root.controller.cameraOpened) root.controller.closeCamera(); else { root.controller.openCamera(); if(root.controller.cameraOpened) root.controller.startPreview() } } }
                    }
                    Rectangle { Layout.fillWidth: true; height: 1; color: "#252a34" }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 6
                        Text { text: "机器人 IP"; color: root.muted; font.pixelSize: 11; Layout.preferredWidth: 45 }
                        CollectField { text: root.controller.robotIp; Layout.preferredWidth: 110; leftPadding: 5; rightPadding: 5; font.pixelSize: 11; enabled: !root.controller.robotConnected && !root.busy; onEditingFinished: root.controller.robotIp = text }
                        Item { Layout.fillWidth: true }
                        CollectButton { objectName: "levelButton"; text: "一键调平"; leftPadding: 6; rightPadding: 6; font.pixelSize: 11; enabled: root.ready && root.controller.robotConnected; onClicked: root.controller.levelCamera() }
                        CollectButton { objectName: "stopButton"; text: "停止"; leftPadding: 6; rightPadding: 6; font.pixelSize: 11; danger: true; enabled: root.controller.collectionBusy; onClicked: root.controller.cancelCollectionOperation() }
                        CollectButton { objectName: "connectRobotButton"; Layout.preferredWidth: 86; leftPadding: 4; rightPadding: 4; font.pixelSize: 11; text: root.controller.robotConnected ? "断开机器人" : "连接机器人"; enabled: !root.busy; onClicked: root.controller.robotConnected ? root.controller.disconnectRobot() : root.controller.connectRobot() }
                    }
                }
            }
            CollectCard {
                Layout.fillWidth: true; Layout.fillHeight: true
                implicitHeight: captureControls.implicitHeight + 28
                ColumnLayout {
                    id: captureControls
                    anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
                    height: implicitHeight; spacing: 8
                    RowLayout {
                        Layout.fillWidth: true
                        Layout.minimumHeight: 28; Layout.preferredHeight: 28; Layout.maximumHeight: 28
                        Rectangle { implicitWidth: 27; implicitHeight: 21; radius: 5; color: "#e8c969"; Text { anchors.centerIn: parent; text: "02"; color: "#101820"; font.pixelSize: 11; font.bold: true } }
                        Text { text: "数据采集"; color: root.ink; font.pixelSize: 14; font.bold: true }
                        Item { Layout.fillWidth: true }
                        CollectButton { text: "图片目录"; quiet: true; iconName: "folder"; font.pixelSize: 11; implicitHeight: 28; onClicked: root.controller.openCollectionFolder() }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        ColumnLayout {
                            spacing: 5
                            Text { text: "融合曝光序列 · μs"; color: root.muted; font.pixelSize: 11 }
                            RowLayout {
                                spacing: 4
                                Repeater {
                                    model: exposureSlots
                                    CollectField {
                                        objectName: index === 0 ? "exposureInput" : "exposureInput" + (index + 1)
                                        text: value; placeholderText: "可选"; font.pixelSize: 11; leftPadding: 2; rightPadding: 2
                                        horizontalAlignment: TextInput.AlignHCenter
                                        Layout.preferredWidth: 48; Layout.minimumWidth: 48; Layout.maximumWidth: 48
                                        validator: IntValidator { bottom: 1; top: 1000000 }
                                        enabled: !root.busy
                                        onTextEdited: exposureSlots.setProperty(index, "value", text)
                                        onEditingFinished: root.saveExposures()
                                    }
                                }
                            }
                        }
                        Item { Layout.fillWidth: true }
                        CollectButton { objectName: "captureButton"; text: "拍照保存"; iconName: "shutter"; primary: true; leftPadding: 9; rightPadding: 9; font.pixelSize: 12; enabled: root.ready; Layout.alignment: Qt.AlignBottom; onClicked: root.takePhoto() }
                        CollectButton { objectName: "savePoseButton"; text: "保存法兰位姿"; iconName: "save"; leftPadding: 9; rightPadding: 9; font.pixelSize: 12; enabled: root.controller.robotConnected && !root.busy; Layout.alignment: Qt.AlignBottom; onClicked: { root.poseEditorOpen = true; pointName.text = ""; pointName.forceActiveFocus() } }
                    }
                    Rectangle {
                        visible: root.poseEditorOpen
                        Layout.alignment: Qt.AlignRight; Layout.preferredWidth: Math.min(410, captureControls.width); implicitHeight: 52
                        color: "#101923"; border.color: "#30445b"; radius: 8
                        RowLayout {
                            anchors.fill: parent; anchors.margins: 8; spacing: 6
                            CollectField { id: pointName; objectName: "pointNameInput"; Layout.fillWidth: true; enabled: !root.busy; placeholderText: "点位名称（留空按时间戳）"; onAccepted: if (confirmPose.enabled) root.controller.saveFlangePose(text); Keys.onEscapePressed: root.poseEditorOpen = false }
                            CollectButton { id: confirmPose; text: "确认保存"; font.pixelSize: 11; leftPadding: 8; rightPadding: 8; enabled: root.controller.robotConnected && !root.busy; onClicked: root.controller.saveFlangePose(pointName.text) }
                            CollectButton { text: "取消"; font.pixelSize: 11; leftPadding: 8; rightPadding: 8; onClicked: root.poseEditorOpen = false }
                        }
                    }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 14
            Rectangle {
                id: viewfinder
                Layout.fillWidth: true; Layout.fillHeight: true
                color: "#151820"; radius: 10; border.color: "#252a34"; clip: true
                ColumnLayout {
                    anchors.fill: parent; anchors.margins: 16; spacing: 12
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        CollectIcon { name: "camera"; ink: "#98a5b6"; Layout.preferredWidth: 18; Layout.preferredHeight: 18 }
                        Rectangle { implicitWidth: 27; implicitHeight: 21; radius: 5; color: "#e99088"; Text { anchors.centerIn: parent; text: "03"; color: "#101820"; font.pixelSize: 11; font.bold: true } }
                            Text { text: "实时预览"; color: root.ink; font.pixelSize: 13; font.weight: Font.Medium }
                        Item { Layout.fillWidth: true }
                        CollectButton { text: root.controller.previewActive ? "暂停预览" : "开始预览"; implicitHeight: 28; font.pixelSize: 11; enabled: root.controller.cameraOpened && !root.busy; onClicked: root.controller.previewActive ? root.controller.stopPreview() : root.controller.startPreview() }
                        Text {
                            text: root.controller.cameraOpened ? root.controller.cameraActualWidth + " × " + root.controller.cameraActualHeight : "LIVE VIEW"
                            color: root.muted; font.family: "DejaVu Sans Mono"; font.pixelSize: 10; font.letterSpacing: 0.7
                        }
                    }
                    Rectangle {
                        id: imageArea
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        color: "#090c10"; radius: 7; clip: true
                        Image {
                            id: cameraImage
                            anchors.fill: parent
                            fillMode: Image.PreserveAspectFit
                            cache: false
                            visible: root.controller.cameraOpened
                            source: "image://camera/current"
                        }
                        // Geometry belongs to the viewfinder and is never saved into the image.
                        Repeater {
                            model: root.controller.cameraOpened ? 0 : 8
                            Rectangle { width: 1; height: imageArea.height; x: (index + 1) * imageArea.width / 9; color: "#181f28" }
                        }
                        Repeater {
                            model: root.controller.cameraOpened ? 0 : 5
                            Rectangle { height: 1; width: imageArea.width; y: (index + 1) * imageArea.height / 6; color: "#181f28" }
                        }
                        Column {
                            anchors.centerIn: parent
                            spacing: 12
                            visible: !root.controller.cameraOpened
                            Rectangle {
                                width: 80; height: 80; radius: 40
                                anchors.horizontalCenter: parent.horizontalCenter
                                color: "#1b2533"; border.color: "#2b3747"
                                CollectIcon { anchors.centerIn: parent; width: 35; height: 35; name: "shutter"; ink: "#6c83a5" }
                            }
                            Text { text: "等待相机画面"; color: "#c8d2df"; font.pixelSize: 17; anchors.horizontalCenter: parent.horizontalCenter }
                            Text { text: "连接相机，即可开始预览与拍照"; color: "#65758b"; font.pixelSize: 11; anchors.horizontalCenter: parent.horizontalCenter }
                        }
                        Rectangle { anchors.centerIn: parent; width: 22; height: 1; color: "#70e5edf7"; visible: root.controller.previewActive }
                        Rectangle { anchors.centerIn: parent; width: 1; height: 22; color: "#70e5edf7"; visible: root.controller.previewActive }
                        Rectangle {
                            anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.margins: 12
                            height: 23; width: modelText.implicitWidth + 16; radius: 4
                            color: "#b31b2330"; visible: root.controller.cameraOpened
                            Text { id: modelText; anchors.centerIn: parent; text: root.controller.cameraModel; color: "#bac7d8"; font.pixelSize: 10 }
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
                        Layout.fillWidth: true
                        Text { text: "保持静止，自动融合多曝光"; color: root.muted; font.pixelSize: 10 }
                        Item { Layout.fillWidth: true }
                        Text { text: root.controller.cameraBackend.toUpperCase() + "  /  原始分辨率"; color: "#6f7e92"; font.pixelSize: 10 }
                    }
                }
            }

            ColumnLayout {
                id: infoSidebar
                Layout.preferredWidth: root.infoWidth
                Layout.minimumWidth: root.infoWidth
                Layout.maximumWidth: root.infoWidth
                Layout.fillHeight: true
                spacing: 12
                CollectCard {
                    id: statusCard
                    Layout.fillWidth: true
                    Layout.minimumWidth: 0
                    Layout.maximumWidth: root.infoWidth
                    implicitHeight: sessionInfo.implicitHeight + 28
                    ColumnLayout {
                        id: sessionInfo
                        anchors.fill: parent; spacing: 7
                        RowLayout {
                            Layout.fillWidth: true
                            Rectangle { implicitWidth: 27; implicitHeight: 21; radius: 5; color: "#69acfa"; Text { anchors.centerIn: parent; text: "04"; color: "#101820"; font.pixelSize: 11; font.bold: true } }
                            Text { text: "采集状态"; color: root.ink; font.pixelSize: 13; font.weight: Font.DemiBold }
                            Item { Layout.fillWidth: true }
                            Text { text: "PNG"; color: root.muted; font.family: "DejaVu Sans Mono"; font.pixelSize: 10 }
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 8
                            Rectangle {
                                Layout.fillWidth: true; implicitHeight: 78; radius: 7; color: "#15271f"; border.color: "#2d4c39"
                                Column {
                                    anchors.fill: parent; anchors.margins: 10; spacing: 3
                                    Text { text: "本次已保存 · 张"; color: "#9ba6b6"; font.pixelSize: 10 }
                                    Text { text: root.controller.savedImageCount < 1000 ? ("000" + root.controller.savedImageCount).slice(-3) : root.controller.savedImageCount; color: "#8be0aa"; font.family: "DejaVu Sans Mono"; font.pixelSize: 31 }
                                }
                            }
                            Rectangle {
                                Layout.fillWidth: true; implicitHeight: 78; radius: 7; color: "#252117"; border.color: "#4b422c"
                                Column {
                                    anchors.fill: parent; anchors.margins: 10; spacing: 13
                                    Text { text: "按日期自动归档"; color: "#b7aa8b"; font.pixelSize: 10 }
                                    Text { text: root.archiveDate; color: "#e3c780"; font.family: "DejaVu Sans Mono"; font.pixelSize: 11 }
                                }
                            }
                        }
                        Rectangle { Layout.fillWidth: true; height: 1; color: "#252a34" }
                        Text { text: "独立点位  " + root.controller.savedPoseCount + " 个"; color: root.muted; font.pixelSize: 11 }
                    }
                }
                CollectCard {
                    Layout.fillWidth: true
                    implicitHeight: exposureControls.implicitHeight + 28
                    ColumnLayout {
                        id: exposureControls
                        anchors.fill: parent; spacing: 10
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "预览曝光"; color: root.ink; font.pixelSize: 13; font.weight: Font.DemiBold }
                            Item { Layout.fillWidth: true }
                            Text { text: "μs"; color: root.muted; font.pixelSize: 11 }
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 8
                            CollectField {
                                id: previewExposure
                                objectName: "previewExposureInput"
                                text: root.controller.cameraExposureUs; mono: true; font.pixelSize: 12
                                Layout.fillWidth: true; Layout.minimumWidth: 0
                                validator: IntValidator { bottom: 1; top: 1000000 }
                                enabled: !root.busy
                            }
                            CollectButton { text: "应用"; font.pixelSize: 12; enabled: !root.busy && previewExposure.acceptableInput; onClicked: root.controller.adjustCameraExposure(parseInt(previewExposure.text)) }
                        }
                    }
                }
                CollectCard {
                    Layout.fillWidth: true
                    Layout.minimumWidth: 0
                    Layout.maximumWidth: root.infoWidth
                    implicitHeight: recentPhoto.implicitHeight + 28
                    ColumnLayout {
                        id: recentPhoto
                        anchors.fill: parent; spacing: 8
                        RowLayout {
                            Layout.fillWidth: true
                            Rectangle { implicitWidth: 27; implicitHeight: 21; radius: 5; color: "#e8c969"; Text { anchors.centerIn: parent; text: "05"; color: "#101820"; font.pixelSize: 11; font.bold: true } }
                            Text { text: "最近图片"; color: root.ink; font.pixelSize: 13; font.weight: Font.DemiBold }
                            Item { Layout.fillWidth: true }
                            Text { text: "多曝光融合"; color: root.muted; font.pixelSize: 10 }
                        }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 10
                            Rectangle {
                                Layout.preferredWidth: 52; Layout.preferredHeight: 52
                                radius: 7; color: "#10151d"; border.color: "#3b4552"; clip: true
                                Image { anchors.fill: parent; anchors.margins: 2; source: root.controller.lastCapturePath ? "file://" + root.controller.lastCapturePath : ""; fillMode: Image.PreserveAspectFit }
                                CollectIcon { anchors.centerIn: parent; visible: !root.controller.lastCapturePath; name: "camera"; ink: "#627184" }
                            }
                            ColumnLayout {
                                Layout.fillWidth: true; Layout.minimumWidth: 0; spacing: 6
                                Text { Layout.fillWidth: true; text: root.controller.lastCapturePath ? root.controller.lastCapturePath.split("/").pop() : "等待第一张图片"; color: "#c5d2e1"; font.pixelSize: 11; elide: Text.ElideMiddle }
                                Text { Layout.fillWidth: true; text: root.controller.lastCapturePath ? "PNG · 多曝光融合" : "保存后显示缩略图"; color: "#93a1b3"; font.pixelSize: 11; wrapMode: Text.Wrap }
                            }
                        }
                    }
                }
                CollectCard {
                    surfaceColor: "#131d19"; outlineColor: "#2b4134"
                    Layout.fillWidth: true
                    Layout.minimumWidth: 0
                    Layout.maximumWidth: root.infoWidth
                    Layout.fillHeight: true
                    Layout.minimumHeight: 96
                    ColumnLayout {
                        anchors.fill: parent; spacing: 10
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "操作日志"; color: root.ink; font.pixelSize: 13; font.weight: Font.DemiBold }
                            Item { Layout.fillWidth: true }
                            Text { text: activityLog.count + " 条"; color: root.muted; font.pixelSize: 10 }
                        }
                        ListView {
                            id: operationLog
                            objectName: "operationLog"
                            Layout.fillWidth: true; Layout.fillHeight: true
                            clip: true; spacing: 10; boundsBehavior: Flickable.StopAtBounds
                            model: activityLog
                            ScrollBar.vertical: ScrollBar {
                                width: 4; policy: ScrollBar.AsNeeded
                                contentItem: Rectangle { radius: 2; color: "#455163"; opacity: parent.active ? 1 : 0.4 }
                            }
                            delegate: Row {
                                width: operationLog.width - 6; spacing: 8
                                Text { width: 54; text: stamp; color: "#879c91"; font.family: "DejaVu Sans Mono"; font.pixelSize: 11 }
                                Text { width: Math.max(1, parent.width - 62); text: message; color: tone; font.pixelSize: 12; wrapMode: Text.Wrap; lineHeight: 1.25 }
                            }
                        }
                    }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true; spacing: 7
            Rectangle { width: 5; height: 5; radius: 2.5; color: root.busy ? "#809dbc" : "#789789" }
            Text { Layout.fillWidth: true; text: root.controller.statusMessage; color: "#8a97a8"; font.pixelSize: 11; elide: Text.ElideMiddle }
            Text { text: "空格 / S / Ctrl+S 拍照"; color: "#748194"; font.pixelSize: 10 }
        }
    }
}
