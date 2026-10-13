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
    color: "#f0f1eb"
    font.family: "Noto Sans CJK SC"
    font.pixelSize: 13
    property var controller: calibrationController
    readonly property bool busy: controller.collectionBusy || controller.running
    readonly property bool ready: controller.cameraOpened && controller.previewActive && !busy
    readonly property bool editing: activeFocusItem !== null && activeFocusItem.cursorPosition !== undefined
    readonly property color ink: "#273242"
    readonly property color muted: "#858d94"
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
                radius: 12; color: "#e1e8fa"
                CollectIcon { anchors.centerIn: parent; width: 26; height: 26; name: "shutter"; ink: "#3b70f1" }
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
                color: root.controller.previewActive ? "#e2eee7" : "#e6e9e2"
                Row {
                    id: headerState
                    anchors.centerIn: parent; spacing: 7
                    Rectangle { width: 6; height: 6; radius: 3; anchors.verticalCenter: parent.verticalCenter; color: root.controller.previewActive ? "#4a9271" : "#9aa29b" }
                    Text {
                        text: root.busy ? "正在处理" : root.controller.previewActive ? "实时预览" : root.controller.cameraOpened ? "预览已暂停" : "相机未连接"
                        color: root.controller.previewActive ? "#4c775f" : "#7a837d"; font.pixelSize: 11
                    }
                }
            }
            Rectangle { Layout.preferredWidth: 1; Layout.preferredHeight: 30; color: "#d8ddd6"; Layout.leftMargin: 5; Layout.rightMargin: 5 }
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

        CollectCard {
            Layout.fillWidth: true
            implicitHeight: cameraConfig.implicitHeight + 32
            RowLayout {
                id: cameraConfig
                anchors.fill: parent
                spacing: 12
                ColumnLayout {
                    spacing: 3
                    Text { text: "相机连接"; color: root.ink; font.weight: Font.Medium; font.pixelSize: 14 }
                    Text { text: root.controller.cameraBackend.toUpperCase(); color: root.muted; font.pixelSize: 10; font.letterSpacing: 1 }
                }
                Rectangle { Layout.preferredWidth: 1; Layout.preferredHeight: 38; color: "#e8eae5"; Layout.leftMargin: 2; Layout.rightMargin: 2 }
                ColumnLayout {
                    spacing: 5
                    Text { text: "设备 IP"; color: root.muted; font.pixelSize: 10 }
                    CollectField {
                        text: root.controller.cameraIp; mono: true; Layout.preferredWidth: 132
                        enabled: !root.controller.cameraOpened && !root.busy
                        onEditingFinished: root.controller.cameraIp = text
                    }
                }
                ColumnLayout {
                    spacing: 5
                    Text { text: "序列号"; color: root.muted; font.pixelSize: 10 }
                    CollectField {
                        text: root.controller.cameraSn; placeholderText: "可选"; mono: true; Layout.preferredWidth: 130
                        enabled: !root.controller.cameraOpened && !root.busy
                        onEditingFinished: root.controller.cameraSn = text
                    }
                }
                ColumnLayout {
                    spacing: 5
                    Text { text: "图像分辨率"; color: root.muted; font.pixelSize: 10 }
                    RowLayout {
                        spacing: 5
                        CollectField {
                            text: root.controller.previewWidth; mono: true; Layout.preferredWidth: 66
                            validator: IntValidator { bottom: 1; top: 32768 }
                            enabled: !root.controller.cameraOpened && !root.busy
                            onEditingFinished: if (acceptableInput) root.controller.previewWidth = parseInt(text)
                        }
                        Text { text: "×"; color: "#9aa2a9" }
                        CollectField {
                            text: root.controller.previewHeight; mono: true; Layout.preferredWidth: 66
                            validator: IntValidator { bottom: 1; top: 32768 }
                            enabled: !root.controller.cameraOpened && !root.busy
                            onEditingFinished: if (acceptableInput) root.controller.previewHeight = parseInt(text)
                        }
                    }
                }
                Item { Layout.fillWidth: true }
                CollectButton {
                    objectName: "openCameraButton"
                    text: root.controller.cameraOpened ? "断开相机" : "连接并预览"
                    iconName: "connect"
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
                    text: root.controller.previewActive ? "暂停" : "预览"
                    iconName: root.controller.previewActive ? "pause" : "play"
                    quiet: true
                    enabled: root.controller.cameraOpened && !root.busy
                    onClicked: root.controller.previewActive ? root.controller.stopPreview() : root.controller.startPreview()
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
                color: "#1a2029"
                radius: 14
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
                        Text {
                            text: root.controller.cameraOpened ? root.controller.cameraActualWidth + " × " + root.controller.cameraActualHeight : "LIVE VIEW"
                            color: "#788595"; font.family: "DejaVu Sans Mono"; font.pixelSize: 10; font.letterSpacing: 0.7
                        }
                    }
                    Rectangle {
                        id: imageArea
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        color: "#111720"; radius: 7; clip: true
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
                            Rectangle { width: 1; height: imageArea.height; x: (index + 1) * imageArea.width / 9; color: "#191f29" }
                        }
                        Repeater {
                            model: root.controller.cameraOpened ? 0 : 5
                            Rectangle { height: 1; width: imageArea.width; y: (index + 1) * imageArea.height / 6; color: "#191f29" }
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
                id: photoSettings
                Layout.preferredWidth: 294
                Layout.fillHeight: true
                clip: true
                contentWidth: availableWidth
                ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
                ColumnLayout {
                    width: photoSettings.availableWidth
                    spacing: 12
                    CollectCard {
                        Layout.fillWidth: true
                        implicitHeight: exposureSettings.implicitHeight + 32
                        ColumnLayout {
                            id: exposureSettings
                            anchors.fill: parent
                            spacing: 8
                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: "拍照参数"; font.pixelSize: 15; font.weight: Font.DemiBold; color: root.ink }
                                Item { Layout.fillWidth: true }
                                Text { text: "01"; font.family: "DejaVu Sans Mono"; font.pixelSize: 11; color: "#b1b8be" }
                            }
                            Text { text: "预览曝光"; font.pixelSize: 11; color: "#77828e" }
                            RowLayout {
                                spacing: 6
                                CollectField {
                                    id: previewExposure
                                    text: root.controller.cameraExposureUs; mono: true; Layout.fillWidth: true
                                    validator: IntValidator { bottom: 1; top: 1000000 }
                                    enabled: !root.busy
                                }
                                Text { text: "μs"; color: "#919ba6"; font.pixelSize: 11; Layout.rightMargin: 3 }
                                CollectButton {
                                    text: "应用"; enabled: !root.busy && previewExposure.acceptableInput
                                    onClicked: root.controller.adjustCameraExposure(parseInt(previewExposure.text))
                                }
                            }
                            Rectangle { Layout.fillWidth: true; height: 1; color: "#edf0e9"; Layout.topMargin: 1; Layout.bottomMargin: 1 }
                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: "拍照曝光序列"; font.pixelSize: 11; color: "#77828e" }
                                Item { Layout.fillWidth: true }
                                Text { text: "μs"; color: "#a2aab1"; font.pixelSize: 10 }
                            }
                            CollectField {
                                id: exposuresField
                                objectName: "exposureInput"
                                Layout.fillWidth: true
                                text: root.controller.exposureList
                                mono: true; font.pixelSize: 10
                                enabled: !root.busy
                                onEditingFinished: root.controller.exposureList = text
                            }
                            Text {
                                text: "逗号分隔，支持 2–9 档曝光。\n拍照时请保持相机与被拍物体静止。"
                                color: "#959fa8"; font.pixelSize: 10; lineHeight: 1.3
                                wrapMode: Text.Wrap; Layout.fillWidth: true
                            }
                        }
                    }
                    CollectCard {
                        Layout.fillWidth: true
                        implicitHeight: librarySettings.implicitHeight + 32
                        ColumnLayout {
                            id: librarySettings
                            anchors.fill: parent
                            spacing: 10
                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: "采集文件"; font.pixelSize: 15; font.weight: Font.DemiBold; color: root.ink }
                                Item { Layout.fillWidth: true }
                                Text { text: "02"; font.family: "DejaVu Sans Mono"; font.pixelSize: 11; color: "#b1b8be" }
                            }
                            Text { text: "保存目录"; font.pixelSize: 11; color: "#77828e" }
                            CollectField {
                                id: outputField
                                Layout.fillWidth: true
                                text: root.controller.outputDirectory
                                enabled: !root.busy
                                onEditingFinished: root.controller.outputDirectory = text
                            }
                            CollectButton {
                                text: "打开文件夹"; iconName: "folder"; quiet: true; Layout.fillWidth: true
                                onClicked: Qt.openUrlExternally("file://" + root.controller.outputDirectory)
                            }
                            Rectangle {
                                Layout.fillWidth: true; Layout.preferredHeight: 78
                                radius: 6; color: "#f0f2ed"; border.color: "#e8ebe4"
                                visible: root.controller.lastCapturePath !== ""
                                Image {
                                    anchors.fill: parent; anchors.margins: 3
                                    source: root.controller.lastCapturePath ? "file://" + root.controller.lastCapturePath : ""
                                    fillMode: Image.PreserveAspectFit
                                }
                                Column {
                                    anchors.centerIn: parent; spacing: 4; visible: root.controller.lastCapturePath === ""
                                    CollectIcon { name: "camera"; ink: "#b3bdb2"; anchors.horizontalCenter: parent.horizontalCenter; width: 21; height: 21 }
                                    Text { text: "尚未保存图片"; font.pixelSize: 10; color: "#a1aaa1" }
                                }
                            }
                            Text {
                                visible: root.controller.lastCapturePath !== ""
                                text: root.controller.lastCapturePath ? root.controller.lastCapturePath.split("/").pop() : "最近一张融合图片将在这里显示"
                                color: "#949ea6"; font.pixelSize: 10; elide: Text.ElideMiddle; Layout.fillWidth: true
                            }
                        }
                    }
                }
            }
        }

        CollectCard {
            Layout.fillWidth: true
            implicitHeight: robotSettings.implicitHeight + 32
            ColumnLayout {
                id: robotSettings
                anchors.fill: parent
                spacing: 8
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10
                    Text { text: "机器人调平"; color: root.ink; font.pixelSize: 15; font.weight: Font.DemiBold }
                    Rectangle {
                        width: robotState.implicitWidth + 16; height: 22; radius: 11
                        color: root.controller.robotConnected ? "#e3eee7" : "#edf0e9"
                        Text { id: robotState; anchors.centerIn: parent; text: root.controller.robotConnected ? "已连接" : "未连接"; color: root.controller.robotConnected ? "#5c876d" : "#a0a8a0"; font.pixelSize: 10 }
                    }
                    Text { text: "镜头平面水平 · 相机朝下"; color: "#9ba4ac"; font.pixelSize: 11 }
                    Item { Layout.fillWidth: true }
                    CollectButton {
                        text: "停止操作"; iconName: "stop"; quiet: true; danger: true
                        enabled: root.controller.collectionBusy
                        onClicked: root.controller.cancelCollectionOperation()
                    }
                    CollectButton {
                        objectName: "levelButton"
                        text: "一键调平"; iconName: "level"
                        enabled: root.ready && root.controller.robotConnected
                        onClicked: {
                            root.controller.handEyePath = handEyeField.text
                            root.controller.handEyeUnits = units.currentText
                            root.controller.levelCamera()
                        }
                    }
                }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    Text { text: "机器人IP"; color: root.muted; font.pixelSize: 11; Layout.preferredWidth: 64 }
                    CollectField {
                        text: root.controller.robotIp; mono: true; Layout.preferredWidth: 132
                        enabled: !root.controller.robotConnected && !root.busy
                        onEditingFinished: root.controller.robotIp = text
                    }
                    Text { text: "端口"; color: root.muted; font.pixelSize: 11; Layout.leftMargin: 6 }
                    CollectField {
                        text: root.controller.robotPort; mono: true; Layout.preferredWidth: 68
                        validator: IntValidator { bottom: 1; top: 65535 }
                        enabled: !root.controller.robotConnected && !root.busy
                        onEditingFinished: if (acceptableInput) root.controller.robotPort = parseInt(text)
                    }
                    Text { text: "密码"; color: root.muted; font.pixelSize: 11; Layout.leftMargin: 6 }
                    CollectField {
                        text: root.controller.robotPasswd; echoMode: TextInput.Password; Layout.preferredWidth: 85
                        enabled: !root.controller.robotConnected && !root.busy
                        onEditingFinished: root.controller.robotPasswd = text
                    }
                    CollectButton {
                        text: root.controller.robotConnected ? "断开连接" : "连接机器人"
                        enabled: !root.busy
                        onClicked: root.controller.robotConnected ? root.controller.disconnectRobot() : root.controller.connectRobot()
                    }
                    Item { Layout.fillWidth: true }
                    Text {
                        text: "调平会移动机器人，并在目标姿态保留镜头中心位置。"
                        color: "#9ba4ac"; font.pixelSize: 10
                        visible: root.width >= 1160
                    }
                }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 8
                    Text { text: "手眼"; color: root.muted; font.pixelSize: 11; Layout.preferredWidth: 34 }
                    CollectField {
                        id: handEyeField; text: root.controller.handEyePath; Layout.fillWidth: true
                        enabled: !root.busy
                        onEditingFinished: root.controller.handEyePath = text
                    }
                    Text { text: "平移单位"; color: root.muted; font.pixelSize: 11; Layout.leftMargin: 6 }
                    ComboBox {
                        id: units
                        model: ["m", "mm"]
                        currentIndex: root.controller.handEyeUnits === "mm" ? 1 : 0
                        implicitHeight: 36; implicitWidth: 75
                        font.family: "DejaVu Sans Mono"; font.pixelSize: 11
                        leftPadding: 12; rightPadding: 26
                        enabled: !root.busy
                        onActivated: root.controller.handEyeUnits = currentText
                        contentItem: Text {
                            text: units.displayText; font: units.font; color: "#556173"
                            verticalAlignment: Text.AlignVCenter
                        }
                        background: Rectangle { radius: 7; color: "#f6f7f5"; border.color: units.activeFocus ? "#3b70f1" : "#e3e6e2" }
                        indicator: Text { x: units.width - 20; anchors.verticalCenter: parent.verticalCenter; text: "⌄"; color: "#7e8c9c"; font.pixelSize: 16 }
                    }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 7
            Rectangle { width: 5; height: 5; radius: 2.5; color: root.busy ? "#3b70f1" : "#92a18f" }
            Text {
                Layout.fillWidth: true
                text: root.controller.statusMessage
                color: "#818c8b"; font.pixelSize: 11
                elide: Text.ElideMiddle
                ToolTip.visible: statusHover.containsMouse
                ToolTip.text: text
                MouseArea { id: statusHover; anchors.fill: parent; hoverEnabled: true; acceptedButtons: Qt.NoButton }
            }
            Text { text: "空格 / S / Ctrl+S 拍照"; color: "#a1a99f"; font.pixelSize: 10 }
        }
    }
}
