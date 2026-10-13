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
    function takePhoto() {
        controller.outputDirectory = outputField.text
        controller.exposureList = exposuresField.text
        controller.capturePhoto()
    }
    Shortcut { sequence: "Space"; autoRepeat: false; context: Qt.WindowShortcut; enabled: root.ready && !root.editing; onActivated: root.takePhoto() }
    Shortcut { sequence: "S"; autoRepeat: false; context: Qt.WindowShortcut; enabled: root.ready && !root.editing; onActivated: root.takePhoto() }
    Shortcut { sequence: "Ctrl+S"; autoRepeat: false; context: Qt.WindowShortcut; enabled: root.ready; onActivated: root.takePhoto() }
    Connections {
        target: root.controller
        function onFrameUpdated() { cameraImage.source = "image://camera/current#" + Date.now() }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 12

        // A quiet instrument header; connection state and session count are live.
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
            Rectangle {
                implicitWidth: headerState.implicitWidth + 30
                implicitHeight: 30
                radius: 15
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
            Rectangle { Layout.preferredWidth: 1; Layout.preferredHeight: 30; color: "#303845"; Layout.leftMargin: 5; Layout.rightMargin: 5 }
            ColumnLayout {
                spacing: 0
                Text { text: "本次采集"; font.pixelSize: 10; color: root.muted; Layout.alignment: Qt.AlignRight }
                Row {
                    spacing: 5
                    Text {
                        text: root.controller.savedImageCount < 1000 ? ("000" + root.controller.savedImageCount).slice(-3) : root.controller.savedImageCount
                        color: root.ink; font.family: "DejaVu Sans Mono"; font.pixelSize: 23
                    }
                    Text { text: "张"; color: root.muted; font.pixelSize: 11; anchors.bottom: parent.bottom; anchors.bottomMargin: 3 }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 14
            Rectangle {
                id: viewfinder
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#1b2027"
                radius: 10
                border.color: "#2c333d"
                clip: true
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 16
                    spacing: 12
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        CollectIcon { name: "camera"; ink: "#98a5b6"; Layout.preferredWidth: 18; Layout.preferredHeight: 18 }
                        Text { text: "取景器"; color: "#e4eaf2"; font.pixelSize: 13; font.weight: Font.Medium }
                        Item { Layout.fillWidth: true }
                        CollectButton {
                            text: "停止操作"; iconName: "stop"; danger: true; quiet: true
                            visible: root.controller.collectionBusy
                            Layout.preferredHeight: 28
                            onClicked: root.controller.cancelCollectionOperation()
                        }
                        Text {
                            text: root.controller.cameraOpened ? root.controller.cameraActualWidth + " × " + root.controller.cameraActualHeight : "LIVE VIEW"
                            color: "#788595"; font.family: "DejaVu Sans Mono"; font.pixelSize: 10; font.letterSpacing: 0.7
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
                        spacing: 10
                        ColumnLayout {
                            spacing: 2
                            Text { text: "多曝光融合"; color: "#cbd6e6"; font.pixelSize: 12 }
                            Text { text: "PNG  /  原始分辨率"; color: "#6f7e92"; font.pixelSize: 10 }
                        }
                        Item { Layout.fillWidth: true }
                        Rectangle {
                            width: shortcutLabel.implicitWidth + 16; height: 25; radius: 5
                            color: "#232d3b"; border.color: "#354255"
                            Text { id: shortcutLabel; text: "SPACE"; anchors.centerIn: parent; color: "#8a9cb4"; font.pixelSize: 9; font.letterSpacing: 1 }
                        }
                        CollectButton {
                            objectName: "captureButton"
                            text: "拍照保存"
                            iconName: "shutter"
                            primary: true
                            Layout.preferredHeight: 42
                            Layout.preferredWidth: 146
                            enabled: root.ready
                            onClicked: root.takePhoto()
                        }
                    }
                }
            }

            ScrollView {
                id: deviceSettings
                Layout.preferredWidth: 340
                Layout.fillHeight: true
                clip: true
                contentWidth: availableWidth
                ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
                ScrollBar.vertical: ScrollBar {
                    policy: ScrollBar.AsNeeded
                    width: 5
                    contentItem: Rectangle { radius: 2; color: parent.pressed ? "#627187" : "#394451"; opacity: parent.active ? 1 : 0.5 }
                }
                ColumnLayout {
                    width: deviceSettings.availableWidth
                    spacing: 12
                    CollectCard {
                        Layout.fillWidth: true
                        implicitHeight: cameraSettings.implicitHeight + 28
                        ColumnLayout {
                            id: cameraSettings
                            anchors.fill: parent
                            spacing: 7
                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: "相机设置"; color: root.ink; font.pixelSize: 14; font.weight: Font.DemiBold }
                                Item { Layout.fillWidth: true }
                                Text { text: root.controller.cameraBackend.toUpperCase(); color: root.muted; font.pixelSize: 10; font.letterSpacing: 1 }
                            }
                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 8
                                ColumnLayout {
                                    Layout.fillWidth: true; spacing: 4
                                    Text { text: "设备 IP"; color: root.muted; font.pixelSize: 10 }
                                    CollectField {
                                        text: root.controller.cameraIp; mono: true; Layout.fillWidth: true
                                        enabled: !root.controller.cameraOpened && !root.busy
                                        onEditingFinished: root.controller.cameraIp = text
                                    }
                                }
                                ColumnLayout {
                                    Layout.fillWidth: true; spacing: 4
                                    Text { text: "序列号"; color: root.muted; font.pixelSize: 10 }
                                    CollectField {
                                        text: root.controller.cameraSn; placeholderText: "可选"; mono: true; Layout.fillWidth: true
                                        enabled: !root.controller.cameraOpened && !root.busy
                                        onEditingFinished: root.controller.cameraSn = text
                                    }
                                }
                            }
                            RowLayout {
                                Layout.fillWidth: true; spacing: 6
                                Text { text: "分辨率"; color: root.muted; font.pixelSize: 11; Layout.preferredWidth: 60 }
                                CollectField {
                                    text: root.controller.previewWidth; mono: true; Layout.fillWidth: true
                                    validator: IntValidator { bottom: 1; top: 32768 }
                                    enabled: !root.controller.cameraOpened && !root.busy
                                    onEditingFinished: if (acceptableInput) root.controller.previewWidth = parseInt(text)
                                }
                                Text { text: "×"; color: root.muted }
                                CollectField {
                                    text: root.controller.previewHeight; mono: true; Layout.fillWidth: true
                                    validator: IntValidator { bottom: 1; top: 32768 }
                                    enabled: !root.controller.cameraOpened && !root.busy
                                    onEditingFinished: if (acceptableInput) root.controller.previewHeight = parseInt(text)
                                }
                            }
                            RowLayout {
                                Layout.fillWidth: true; spacing: 6
                                CollectButton {
                                    objectName: "openCameraButton"
                                    text: root.controller.cameraOpened ? "断开相机" : "连接并预览"; iconName: "connect"
                                    Layout.fillWidth: true; enabled: !root.busy
                                    onClicked: {
                                        if (root.controller.cameraOpened) root.controller.closeCamera()
                                        else {
                                            root.controller.openCamera()
                                            if (root.controller.cameraOpened) root.controller.startPreview()
                                        }
                                    }
                                }
                                CollectButton {
                                    text: root.controller.previewActive ? "暂停" : "预览"
                                    iconName: root.controller.previewActive ? "pause" : "play"; quiet: true
                                    enabled: root.controller.cameraOpened && !root.busy
                                    onClicked: root.controller.previewActive ? root.controller.stopPreview() : root.controller.startPreview()
                                }
                            }
                            Rectangle { Layout.fillWidth: true; height: 1; color: "#2c333d"; Layout.topMargin: 3; Layout.bottomMargin: 3 }
                            RowLayout {
                                Layout.fillWidth: true; spacing: 6
                                Text { text: "预览曝光"; color: root.muted; font.pixelSize: 11; Layout.preferredWidth: 60 }
                                CollectField {
                                    id: previewExposure
                                    text: root.controller.cameraExposureUs; mono: true; Layout.fillWidth: true
                                    validator: IntValidator { bottom: 1; top: 1000000 }
                                    enabled: !root.busy
                                }
                                Text { text: "μs"; color: root.muted; font.pixelSize: 10 }
                                CollectButton {
                                    text: "应用"; enabled: !root.busy && previewExposure.acceptableInput
                                    onClicked: root.controller.adjustCameraExposure(parseInt(previewExposure.text))
                                }
                            }
                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: "拍照曝光序列"; color: root.muted; font.pixelSize: 11 }
                                Item { Layout.fillWidth: true }
                                Text { text: "μs"; color: root.muted; font.pixelSize: 10 }
                            }
                            CollectField {
                                id: exposuresField; objectName: "exposureInput"
                                Layout.fillWidth: true; text: root.controller.exposureList
                                mono: true; font.pixelSize: 11; enabled: !root.busy
                                onEditingFinished: root.controller.exposureList = text
                            }
                            Text { text: "2–9 档曝光，逗号分隔；拍照时保持静止。"; color: root.muted; font.pixelSize: 10; wrapMode: Text.Wrap; Layout.fillWidth: true }
                        }
                    }
                    CollectCard {
                        objectName: "robotSettingsCard"
                        Layout.fillWidth: true
                        implicitHeight: robotSettings.implicitHeight + 28
                        ColumnLayout {
                            id: robotSettings
                            anchors.fill: parent
                            spacing: 7
                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: "机器人设置"; color: root.ink; font.pixelSize: 14; font.weight: Font.DemiBold }
                                Item { Layout.fillWidth: true }
                                Text { text: root.controller.robotConnected ? "● 已连接" : "○ 未连接"; color: root.controller.robotConnected ? "#83a997" : root.muted; font.pixelSize: 10 }
                            }
                            RowLayout {
                                Layout.fillWidth: true; spacing: 6
                                Text { text: "机器人IP"; color: root.muted; font.pixelSize: 11; Layout.preferredWidth: 60 }
                                CollectField {
                                    text: root.controller.robotIp; mono: true; Layout.fillWidth: true
                                    enabled: !root.controller.robotConnected && !root.busy
                                    onEditingFinished: root.controller.robotIp = text
                                }
                            }
                            RowLayout {
                                Layout.fillWidth: true; spacing: 6
                                Text { text: "端口"; color: root.muted; font.pixelSize: 11 }
                                CollectField {
                                    text: root.controller.robotPort; mono: true; Layout.preferredWidth: 76
                                    validator: IntValidator { bottom: 1; top: 65535 }
                                    enabled: !root.controller.robotConnected && !root.busy
                                    onEditingFinished: if (acceptableInput) root.controller.robotPort = parseInt(text)
                                }
                                Text { text: "密码"; color: root.muted; font.pixelSize: 11; Layout.leftMargin: 3 }
                                CollectField {
                                    text: root.controller.robotPasswd; echoMode: TextInput.Password; Layout.fillWidth: true
                                    enabled: !root.controller.robotConnected && !root.busy
                                    onEditingFinished: root.controller.robotPasswd = text
                                }
                            }
                            RowLayout {
                                Layout.fillWidth: true; spacing: 6
                                CollectButton {
                                    text: root.controller.robotConnected ? "断开连接" : "连接机器人"; iconName: "connect"
                                    Layout.fillWidth: true; enabled: !root.busy
                                    onClicked: root.controller.robotConnected ? root.controller.disconnectRobot() : root.controller.connectRobot()
                                }
                                CollectButton {
                                    objectName: "levelButton"
                                    text: "一键调平"; iconName: "level"; Layout.fillWidth: true
                                    enabled: root.ready && root.controller.robotConnected
                                    onClicked: root.controller.levelCamera()
                                }
                            }
                            Text {
                                text: "镜头平面水平 · 相机朝下\n无手眼矩阵时，按法兰方向调平。"
                                color: root.muted; font.pixelSize: 10; lineHeight: 1.3
                                Layout.fillWidth: true; wrapMode: Text.Wrap
                            }
                        }
                    }
                    CollectCard {
                        Layout.fillWidth: true
                        implicitHeight: librarySettings.implicitHeight + 28
                        ColumnLayout {
                            id: librarySettings
                            anchors.fill: parent
                            spacing: 7
                            Text { text: "采集文件"; color: root.ink; font.pixelSize: 14; font.weight: Font.DemiBold }
                            CollectField {
                                id: outputField
                                Layout.fillWidth: true; text: root.controller.outputDirectory; enabled: !root.busy
                                onEditingFinished: root.controller.outputDirectory = text
                            }
                            CollectButton {
                                text: "打开文件夹"; iconName: "folder"; quiet: true; Layout.fillWidth: true
                                onClicked: Qt.openUrlExternally("file://" + root.controller.outputDirectory)
                            }
                            Rectangle {
                                Layout.fillWidth: true; Layout.preferredHeight: 90
                                radius: 6; color: "#151a21"; border.color: "#303845"
                                visible: root.controller.lastCapturePath !== ""
                                Image {
                                    anchors.fill: parent; anchors.margins: 3
                                    source: root.controller.lastCapturePath ? "file://" + root.controller.lastCapturePath : ""
                                    fillMode: Image.PreserveAspectFit
                                }
                            }
                            Text {
                                visible: root.controller.lastCapturePath !== ""
                                text: root.controller.lastCapturePath.split("/").pop()
                                color: root.muted; font.pixelSize: 10; elide: Text.ElideMiddle; Layout.fillWidth: true
                            }
                        }
                    }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 7
            Rectangle { width: 5; height: 5; radius: 2.5; color: root.busy ? "#809dbc" : "#789789" }
            Text {
                Layout.fillWidth: true
                text: root.controller.statusMessage
                color: "#8a97a8"; font.pixelSize: 11
                elide: Text.ElideMiddle
                ToolTip.visible: statusHover.containsMouse
                ToolTip.text: text
                MouseArea { id: statusHover; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
            }
            Text { text: "空格 / S / Ctrl+S 拍照"; color: "#748194"; font.pixelSize: 10 }
        }
    }
}
