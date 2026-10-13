import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: root
    visible: true
    width: 1280
    height: 850
    minimumWidth: 1000
    minimumHeight: 740
    title: "COLLECT · 图像采集 · " + appVersion
    color: "#12161c"
    font.family: "Noto Sans CJK SC"
    font.pixelSize: 13
    property var controller: calibrationController
    readonly property bool busy: controller.collectionBusy || controller.running
    readonly property bool ready: controller.cameraOpened && controller.previewActive && !busy
    readonly property bool editing: activeFocusItem !== null && activeFocusItem.cursorPosition !== undefined
    readonly property color ink: "#dce3ed"
    readonly property color muted: "#8693a4"
    readonly property bool wideExposureRow: width >= 1280
    readonly property int infoWidth: width < 1100 ? 270 : 290
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
        for (var i = 0; i < 8; ++i) exposureSlots.append({value: i < values.length ? values[i] : ""})
        recordStatus(controller.statusMessage)
    }
    Timer { interval: 60000; running: true; repeat: true; onTriggered: root.today = Qt.formatDate(new Date(), "yyyy-MM-dd") }
    ListModel { id: activityLog }
    ListModel { id: exposureSlots }
    Shortcut { sequence: "Space"; autoRepeat: false; context: Qt.WindowShortcut; enabled: root.ready && !root.editing; onActivated: root.takePhoto() }
    Shortcut { sequence: "S"; autoRepeat: false; context: Qt.WindowShortcut; enabled: root.ready && !root.editing; onActivated: root.takePhoto() }
    Shortcut { sequence: "Ctrl+S"; autoRepeat: false; context: Qt.WindowShortcut; enabled: root.ready; onActivated: root.takePhoto() }
    Connections {
        target: root.controller
        function onFrameUpdated() { cameraImage.source = "image://camera/current#" + Date.now() }
        function onStatusMessageChanged() { root.recordStatus(root.controller.statusMessage) }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 12

        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: 48
            spacing: 13
            Rectangle {
                Layout.preferredWidth: 44; Layout.preferredHeight: 44
                radius: 12; color: "#222d3c"
                CollectIcon { anchors.centerIn: parent; width: 26; height: 26; name: "shutter"; ink: "#809dbc" }
            }
            ColumnLayout {
                spacing: 0
                Text { text: "COLLECT"; color: root.ink; font.pixelSize: 24; font.weight: Font.DemiBold; font.letterSpacing: 1.8 }
                Text { text: "图像采集工作台"; color: root.muted; font.pixelSize: 12 }
            }
            Item { Layout.fillWidth: true }
            Text { text: "多曝光融合  /  PNG"; color: root.muted; font.pixelSize: 11; Layout.rightMargin: 10 }
            Rectangle {
                implicitWidth: headerState.implicitWidth + 30; implicitHeight: 30; radius: 15
                color: root.controller.previewActive ? "#22332e" : "#232a34"
                Row {
                    id: headerState
                    anchors.centerIn: parent; spacing: 7
                    Rectangle { width: 6; height: 6; radius: 3; anchors.verticalCenter: parent.verticalCenter; color: root.controller.previewActive ? "#85ad99" : "#7d8999" }
                    Text {
                        text: root.busy ? "正在处理" : root.controller.previewActive ? "实时预览" : root.controller.cameraOpened ? "预览已暂停" : "相机未连接"
                        color: root.controller.previewActive ? "#96b5a6" : "#96a1b1"; font.pixelSize: 11
                    }
                }
            }
        }

        // Keep every editable setting and action in one top control deck.
        CollectCard {
            objectName: "controlDeck"
            Layout.fillWidth: true
            implicitHeight: controls.implicitHeight + 28
            ColumnLayout {
                id: controls
                anchors.fill: parent
                spacing: 7
                RowLayout {
                    Layout.fillWidth: true; spacing: 7
                    RowLayout {
                        Layout.preferredWidth: 70; spacing: 7
                        CollectIcon { name: "camera"; ink: "#8d9eb4"; Layout.preferredWidth: 19; Layout.preferredHeight: 19 }
                        Text { text: "相机"; color: root.ink; font.pixelSize: 12; font.weight: Font.Medium }
                    }
                    Text { text: "IP"; color: root.muted; font.pixelSize: 11 }
                    CollectField {
                        text: root.controller.cameraIp; mono: true; font.pixelSize: 11; Layout.preferredWidth: 126
                        enabled: !root.controller.cameraOpened && !root.busy
                        onEditingFinished: root.controller.cameraIp = text
                    }
                    Text { text: "序列号"; color: root.muted; font.pixelSize: 11; Layout.leftMargin: 5 }
                    CollectField {
                        text: root.controller.cameraSn; placeholderText: "可选"; mono: true; font.pixelSize: 11; Layout.preferredWidth: 100
                        enabled: !root.controller.cameraOpened && !root.busy
                        onEditingFinished: root.controller.cameraSn = text
                    }
                    Text { text: "分辨率"; color: root.muted; font.pixelSize: 11; Layout.leftMargin: 5 }
                    CollectField {
                        text: root.controller.previewWidth; mono: true; font.pixelSize: 11; Layout.preferredWidth: 56
                        validator: IntValidator { bottom: 1; top: 32768 }
                        enabled: !root.controller.cameraOpened && !root.busy
                        onEditingFinished: if (acceptableInput) root.controller.previewWidth = parseInt(text)
                    }
                    Text { text: "×"; color: root.muted; font.pixelSize: 11 }
                    CollectField {
                        text: root.controller.previewHeight; mono: true; font.pixelSize: 11; Layout.preferredWidth: 56
                        validator: IntValidator { bottom: 1; top: 32768 }
                        enabled: !root.controller.cameraOpened && !root.busy
                        onEditingFinished: if (acceptableInput) root.controller.previewHeight = parseInt(text)
                    }
                    Item { Layout.fillWidth: true }
                    RowLayout {
                        Layout.preferredWidth: 326
                        Layout.minimumWidth: 326
                        Layout.maximumWidth: 326
                        spacing: 7
                        CollectButton {
                            objectName: "openCameraButton"
                            Layout.preferredWidth: 112
                            Layout.minimumWidth: 112
                            Layout.maximumWidth: 112
                            text: root.controller.cameraOpened ? "断开相机" : "连接并预览"; iconName: "connect"; font.pixelSize: 12
                            enabled: !root.busy
                            onClicked: {
                                if (root.controller.cameraOpened) root.controller.closeCamera()
                                else {
                                    root.controller.openCamera()
                                    if (root.controller.cameraOpened) root.controller.startPreview()
                                }
                            }
                        }
                        CollectButton {
                            Layout.preferredWidth: 100
                            Layout.minimumWidth: 100
                            Layout.maximumWidth: 100
                            text: root.controller.previewActive ? "暂停" : "预览"
                            iconName: root.controller.previewActive ? "pause" : "play"; quiet: true; font.pixelSize: 12
                            enabled: root.controller.cameraOpened && !root.busy
                            onClicked: root.controller.previewActive ? root.controller.stopPreview() : root.controller.startPreview()
                        }
                        Item { Layout.preferredWidth: 100; Layout.minimumWidth: 100; Layout.maximumWidth: 100 }
                    }
                }
                Rectangle { Layout.fillWidth: true; height: 1; color: "#2c333d" }
                RowLayout {
                    Layout.fillWidth: true; spacing: 7
                    RowLayout {
                        Layout.preferredWidth: 70; spacing: 7
                        CollectIcon { name: "level"; ink: "#8d9eb4"; Layout.preferredWidth: 19; Layout.preferredHeight: 19 }
                        Text { text: "机器人"; color: root.ink; font.pixelSize: 12; font.weight: Font.Medium }
                    }
                    Text { text: "机器人IP"; color: root.muted; font.pixelSize: 11 }
                    CollectField {
                        text: root.controller.robotIp; mono: true; font.pixelSize: 11; Layout.preferredWidth: 122
                        enabled: !root.controller.robotConnected && !root.busy
                        onEditingFinished: root.controller.robotIp = text
                    }
                    Text { text: "端口"; color: root.muted; font.pixelSize: 11; Layout.leftMargin: 5 }
                    CollectField {
                        text: root.controller.robotPort; mono: true; font.pixelSize: 11; Layout.preferredWidth: 64
                        validator: IntValidator { bottom: 1; top: 65535 }
                        enabled: !root.controller.robotConnected && !root.busy
                        onEditingFinished: if (acceptableInput) root.controller.robotPort = parseInt(text)
                    }
                    Text { text: "密码"; color: root.muted; font.pixelSize: 11; Layout.leftMargin: 5 }
                    CollectField {
                        text: root.controller.robotPasswd; echoMode: TextInput.Password; font.pixelSize: 11; Layout.preferredWidth: 76
                        enabled: !root.controller.robotConnected && !root.busy
                        onEditingFinished: root.controller.robotPasswd = text
                    }
                    Item { Layout.fillWidth: true }
                    RowLayout {
                        Layout.preferredWidth: 326
                        Layout.minimumWidth: 326
                        Layout.maximumWidth: 326
                        spacing: 7
                        CollectButton {
                            objectName: "connectRobotButton"
                            Layout.preferredWidth: 112
                            Layout.minimumWidth: 112
                            Layout.maximumWidth: 112
                            text: root.controller.robotConnected ? "断开机器人" : "连接机器人"; iconName: "connect"; font.pixelSize: 12
                            enabled: !root.busy
                            onClicked: root.controller.robotConnected ? root.controller.disconnectRobot() : root.controller.connectRobot()
                        }
                        CollectButton {
                            objectName: "levelButton"
                            Layout.preferredWidth: 100
                            Layout.minimumWidth: 100
                            Layout.maximumWidth: 100
                            text: "一键调平"; iconName: "level"; font.pixelSize: 12
                            enabled: root.ready && root.controller.robotConnected
                            onClicked: root.controller.levelCamera()
                            ToolTip.visible: hovered
                            ToolTip.text: "镜头平面水平，相机向下；无手眼矩阵时按法兰方向调平。"
                        }
                        CollectButton {
                            objectName: "stopButton"
                            Layout.preferredWidth: 100
                            Layout.minimumWidth: 100
                            Layout.maximumWidth: 100
                            text: "停止操作"; iconName: "stop"; danger: true; quiet: true; font.pixelSize: 12
                            enabled: root.controller.collectionBusy
                            onClicked: root.controller.cancelCollectionOperation()
                        }
                    }
                }
                Rectangle { Layout.fillWidth: true; height: 1; color: "#2c333d" }
                GridLayout {
                    Layout.fillWidth: true
                    columns: root.wideExposureRow ? 2 : 1
                    columnSpacing: 12
                    rowSpacing: 7
                    RowLayout {
                        Layout.alignment: Qt.AlignLeft
                        Layout.maximumWidth: implicitWidth
                        spacing: 7
                        RowLayout {
                            Layout.preferredWidth: 70; spacing: 7
                            CollectIcon { name: "shutter"; ink: "#8d9eb4"; Layout.preferredWidth: 19; Layout.preferredHeight: 19 }
                            Text { text: "采集"; color: root.ink; font.pixelSize: 12; font.weight: Font.Medium }
                        }
                        Text { text: "预览曝光"; color: root.muted; font.pixelSize: 11 }
                        CollectField {
                            id: previewExposure
                            text: root.controller.cameraExposureUs; mono: true; font.pixelSize: 11; Layout.preferredWidth: 80
                            validator: IntValidator { bottom: 1; top: 1000000 }
                            enabled: !root.busy
                        }
                        Text { text: "μs"; color: root.muted; font.pixelSize: 10 }
                        CollectButton {
                            text: "应用"; font.pixelSize: 12
                            enabled: !root.busy && previewExposure.acceptableInput
                            onClicked: root.controller.adjustCameraExposure(parseInt(previewExposure.text))
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 7
                        Text {
                            text: "融合序列"; color: root.muted; font.pixelSize: 11
                            Layout.preferredWidth: root.wideExposureRow ? implicitWidth : 70
                        }
                        RowLayout {
                            spacing: 5
                            Repeater {
                                id: exposureInputs
                                model: exposureSlots
                                CollectField {
                                    objectName: index === 0 ? "exposureInput" : "exposureInput" + (index + 1)
                                    text: value; placeholderText: "可选"
                                    mono: true; font.pixelSize: 11
                                    leftPadding: 7; rightPadding: 7
                                    horizontalAlignment: TextInput.AlignHCenter
                                    Layout.preferredWidth: 62
                                    Layout.minimumWidth: 62
                                    Layout.maximumWidth: 62
                                    validator: IntValidator { bottom: 1; top: 1000000 }
                                    enabled: !root.busy
                                    hoverEnabled: true
                                    onTextEdited: exposureSlots.setProperty(index, "value", text)
                                    onActiveFocusChanged: if (!activeFocus && exposureSlots.count === 8 && !root.busy) root.saveExposures()
                                    ToolTip.visible: hovered && enabled
                                    ToolTip.text: "第 " + (index + 1) + " 档曝光（μs），留空不参与融合"
                                }
                            }
                        }
                        Text { text: "μs"; color: root.muted; font.pixelSize: 10 }
                        Item { Layout.fillWidth: true }
                        CollectButton {
                            text: "图片目录"; iconName: "folder"; quiet: true; font.pixelSize: 12
                            onClicked: root.controller.openCollectionFolder()
                        }
                        CollectButton {
                            objectName: "captureButton"
                            text: "拍照保存"; iconName: "shutter"; primary: true; font.pixelSize: 12
                            enabled: root.ready
                            onClicked: root.takePhoto()
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
                color: "#1b2027"; radius: 10; border.color: "#2c333d"; clip: true
                ColumnLayout {
                    anchors.fill: parent; anchors.margins: 16; spacing: 12
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        CollectIcon { name: "camera"; ink: "#98a5b6"; Layout.preferredWidth: 18; Layout.preferredHeight: 18 }
                        Text { text: "实时取景"; color: root.ink; font.pixelSize: 13; font.weight: Font.Medium }
                        Item { Layout.fillWidth: true }
                        Text {
                            text: root.controller.cameraOpened ? root.controller.cameraActualWidth + " × " + root.controller.cameraActualHeight : "LIVE VIEW"
                            color: root.muted; font.family: "DejaVu Sans Mono"; font.pixelSize: 10; font.letterSpacing: 0.7
                        }
                    }
                    Rectangle {
                        id: imageArea
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        color: "#10141a"; radius: 7; clip: true
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
                        anchors.fill: parent; spacing: 8
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "采集状态"; color: root.ink; font.pixelSize: 13; font.weight: Font.DemiBold }
                            Item { Layout.fillWidth: true }
                            Text { text: "PNG"; color: root.muted; font.family: "DejaVu Sans Mono"; font.pixelSize: 10 }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "相机"; color: root.muted; font.pixelSize: 11 }
                            Text { text: root.controller.previewActive ? "● 预览中" : root.controller.cameraOpened ? "● 已暂停" : "○ 未连接"; color: root.controller.previewActive ? "#90b3a2" : root.muted; font.pixelSize: 11 }
                            Item { Layout.fillWidth: true }
                            Text { text: "机器人"; color: root.muted; font.pixelSize: 11 }
                            Text { text: root.controller.robotConnected ? "● 已连接" : "○ 未连接"; color: root.controller.robotConnected ? "#90b3a2" : root.muted; font.pixelSize: 11 }
                        }
                        Rectangle { Layout.fillWidth: true; height: 1; color: "#2c333d" }
                        RowLayout {
                            Layout.fillWidth: true
                            ColumnLayout {
                                spacing: 2
                                Text { text: "本次已保存"; color: root.muted; font.pixelSize: 10 }
                                Row {
                                    spacing: 5
                                    Text {
                                        text: root.controller.savedImageCount < 1000 ? ("000" + root.controller.savedImageCount).slice(-3) : root.controller.savedImageCount
                                        color: root.ink; font.family: "DejaVu Sans Mono"; font.pixelSize: 27
                                    }
                                    Text { text: "张"; color: root.muted; font.pixelSize: 10; anchors.bottom: parent.bottom; anchors.bottomMargin: 4 }
                                }
                            }
                            Item { Layout.fillWidth: true }
                            ColumnLayout {
                                spacing: 5; Layout.alignment: Qt.AlignRight
                                Text { text: "按日期自动归档"; color: root.muted; font.pixelSize: 10; Layout.alignment: Qt.AlignRight }
                                Text { text: root.archiveDate; color: "#b8c6d7"; font.family: "DejaVu Sans Mono"; font.pixelSize: 11 }
                            }
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
                            Text { text: "最近图片"; color: root.ink; font.pixelSize: 13; font.weight: Font.DemiBold }
                            Item { Layout.fillWidth: true }
                            Text { text: "多曝光融合"; color: root.muted; font.pixelSize: 10 }
                        }
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: root.controller.lastCapturePath ? (root.height < 800 ? 52 : 106) : 66
                            radius: 6; color: "#151a21"; border.color: "#303845"
                            Image {
                                anchors.fill: parent; anchors.margins: 3
                                source: root.controller.lastCapturePath ? "file://" + root.controller.lastCapturePath : ""
                                fillMode: Image.PreserveAspectFit
                            }
                            Text { anchors.centerIn: parent; text: "保存后的图片将在此显示"; visible: !root.controller.lastCapturePath; color: "#65758b"; font.pixelSize: 10 }
                        }
                        Text {
                            visible: root.controller.lastCapturePath !== ""
                            text: root.controller.lastCapturePath.split("/").pop()
                            color: root.muted; font.pixelSize: 10; elide: Text.ElideMiddle; Layout.fillWidth: true
                        }
                    }
                }
                CollectCard {
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
                            delegate: Column {
                                width: operationLog.width - 10; spacing: 4
                                Text { text: stamp; color: "#65758b"; font.family: "DejaVu Sans Mono"; font.pixelSize: 10 }
                                Text { width: parent.width; text: message; color: tone; font.pixelSize: 11; wrapMode: Text.Wrap; lineHeight: 1.25 }
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
