import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "."
Item {
    id: page
    property var controller
    property bool locked: false
    signal leave()
    property int tab: 0
    property int camSubTab: 0
    property string cameraIp: ""
    property string robotIp: ""
    property string robotSpeed: ""
    property string previewWidth: ""
    property string previewHeight: ""
    property string previewExposure: ""
    property string camera2Ip: ""
    property string camera2Sn: ""
    property string preview2Width: ""
    property string preview2Height: ""
    property string preview2Exposure: ""
    property string camera2Gain: ""
    property var savedDraft: ({})
    property int revision: 0
    property bool loaded: false
    property string feedback: ""
    readonly property var draft: {
        var dependency = revision
        var exposures = []
        for (var i = 0; i < exposureSlots.count; ++i) {
            var value = exposureSlots.get(i).value.trim()
            if (value !== "") exposures.push(value)
        }
        var exposures2 = []
        for (var j = 0; j < exposure2Slots.count; ++j) {
            var v2 = exposure2Slots.get(j).value.trim()
            if (v2 !== "") exposures2.push(v2)
        }
        return {cameraIp: cameraIp, camera2Ip: camera2Ip, camera2Sn: camera2Sn, robotIp: robotIp, robotSpeedMmS: robotSpeed,
            previewWidth: previewWidth, previewHeight: previewHeight, preview2Width: preview2Width, preview2Height: preview2Height,
            cameraExposureUs: previewExposure, camera2ExposureUs: preview2Exposure, camera2Gain: camera2Gain,
            exposureList: exposures.join(","), exposure2List: exposures2.join(",")}
    }
    readonly property int dirtyCount: {
        var count = 0
        for (var key in draft) if (String(draft[key]) !== String(savedDraft[key])) ++count
        return loaded ? count : 0
    }
    readonly property bool dirty: dirtyCount > 0
    function exposureListError(list) {
        var values = list.split(",").filter(function(v) { return v !== "" })
        if (values.length < 2 || values.length > 5) return "融合曝光应包含 2～5 个数值，未写入配置"
        var seen = {}
        for (var i = 0; i < values.length; ++i) {
            var n = Number(values[i])
            if (!/^\d+$/.test(values[i]) || !isFinite(n) || n < 1 || n > 1000000 || seen[n])
                return "融合曝光应为不同的正整数（1～1000000 μs），未写入配置"
            seen[n] = true
        }
        return ""
    }
    function restoreDefaultExposures(model) {
        if (locked) return
        var defaults = ["5000", "8500", "14500", "25000", ""]
        for (var i = 0; i < model.count; ++i) model.setProperty(i, "value", defaults[i])
        ++revision
    }
    readonly property string exposureError: exposureListError(draft.exposureList)
    readonly property string exposure2Error: exposureListError(draft.exposure2List)
    function reload() {
        loaded = false
        cameraIp = controller.cameraIp; robotIp = controller.robotIp
        robotSpeed = String(controller.robotSpeedMmS)
        previewWidth = String(controller.previewWidth); previewHeight = String(controller.previewHeight)
        previewExposure = String(controller.cameraExposureUs)
        camera2Ip = controller.camera2Ip
        camera2Sn = controller.camera2Sn
        preview2Width = String(controller.preview2Width)
        preview2Height = String(controller.preview2Height)
        preview2Exposure = String(controller.camera2ExposureUs)
        camera2Gain = String(controller.camera2Gain)
        var values = controller.exposureList.split(/[,，;\s]+/).filter(function(v) { return v !== "" })
        for (var i = 0; i < exposureSlots.count; ++i) exposureSlots.setProperty(i, "value", i < values.length ? values[i] : "")
        var values2 = controller.exposure2List.split(/[,，;\s]+/).filter(function(v) { return v !== "" })
        for (var k = 0; k < exposure2Slots.count; ++k) exposure2Slots.setProperty(k, "value", k < values2.length ? values2[k] : "")
        ++revision
        savedDraft = JSON.parse(JSON.stringify(draft))
        loaded = true; feedback = "配置已载入，编辑后点击「保存配置」"
    }
    function requestLeave() { if (locked) return; if (dirty) leaveConfirm.open(); else leave() }
    function save() {
        if (locked) return
        if (exposureError) { feedback = exposureError; return }
        if (exposure2Error) { feedback = "相机 2 " + exposure2Error; return }
        feedback = "正在保存配置…"
        controller.applySettings(draft)
    }
    Connections {
        target: page.controller
        function onSettingsFinished(error) {
            if (error) { page.feedback = error; return }
            page.reload(); page.feedback = "配置已保存"
        }
    }
    Component.onCompleted: reload()
    ListModel {
        id: exposureSlots
        ListElement { value: "" }
        ListElement { value: "" }
        ListElement { value: "" }
        ListElement { value: "" }
        ListElement { value: "" }
    }
    ListModel {
        id: exposure2Slots
        ListElement { value: "" }
        ListElement { value: "" }
        ListElement { value: "" }
        ListElement { value: "" }
        ListElement { value: "" }
    }
    ColumnLayout {
        anchors.fill: parent; spacing: 0
        // TOP HEADER WITH CENTERED SEGMENTED TABS
        Rectangle {
            color: Theme.surface; Layout.fillWidth: true; Layout.preferredHeight: 58
            Rectangle { anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom; height: 1; color: Theme.hairline }
            RowLayout {
                anchors.fill: parent; anchors.leftMargin: 16; anchors.rightMargin: 16; spacing: 12
                CollectButton {
                    objectName: "settingsBackButton"; text: "← 返回采集"; small: true
                    implicitHeight: 32; enabled: !page.locked; onClicked: page.requestLeave()
                }
                HmiText {
                    visible: page.width >= 900; text: "系统设置"; color: Theme.ink
                    font.pixelSize: Theme.title; font.weight: Font.DemiBold
                }
                Item { Layout.fillWidth: true }
                // Segmented Tab Switcher
                Rectangle {
                    Layout.preferredHeight: 34; implicitWidth: tabRow.implicitWidth + 6
                    radius: 8; color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                    RowLayout {
                        id: tabRow; anchors.centerIn: parent; spacing: 2
                        Repeater {
                            model: ["设备连接", "成像与曝光", "存储与诊断"]
                            CollectButton {
                                objectName: "settingsTab" + index; text: modelData
                                small: true; segmented: true; implicitHeight: 28
                                leftPadding: page.width < 820 ? 8 : 14; rightPadding: page.width < 820 ? 8 : 14
                                selected: page.tab === index
                                onClicked: page.tab = index
                            }
                        }
                    }
                }
                Item { Layout.fillWidth: true }
                Rectangle {
                    visible: page.dirty; Layout.preferredHeight: 24; implicitWidth: dirtyLbl.implicitWidth + 14; radius: 12
                    color: Theme.warningSoft; border.width: 1; border.color: Theme.warningLine
                    HmiText { id: dirtyLbl; anchors.centerIn: parent; text: page.dirtyCount + " 项未保存"; font.pixelSize: Theme.micro; color: Theme.warningText }
                }
                HmiText {
                    visible: !page.dirty && page.width >= 900
                    text: appVersion + " · " + appGitCommit; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink3
                }
            }
        }

        // SCROLLABLE TWO-COLUMN CARD DECK
        ScrollView {
            id: scroll
            // The viewport size comes from the page, independently of the scrollable cards.
            implicitHeight: 0
            objectName: "settingsScroll"
            Layout.fillWidth: true; Layout.fillHeight: true
            clip: true; contentWidth: availableWidth
            ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
            Item {
                width: scroll.availableWidth
                implicitHeight: Math.max(scroll.availableHeight, pair.height + 32)
                Item {
                    id: pair
                    width: Math.min(parent.width - 32, Theme.pageContentWidth)
                    x: Math.round((parent.width - width) / 2)
                    y: Math.round((parent.height - height) / 2)
                    // Reserve the tallest tab so switching only changes the visible form without flicker.
                    height: Math.max(leftContent.requiredHeight + Theme.padding * 2,
                        rightContent.requiredHeight + Theme.padding * 2,
                        Math.min(Theme.pageCardMinHeight, Math.max(0, scroll.availableHeight - 32)))
                    // LEFT CARD
                    HmiPanel {
                        objectName: "settingsLeftCard"; width: (parent.width - 14) / 2; height: parent.height; x: 0
                        Column {
                            id: leftContent; width: parent.width; spacing: 14
                            readonly property real deviceHeight: Math.max(camera1DeviceFields.implicitHeight, camera2DeviceFields.implicitHeight)
                            readonly property real requiredHeight: 32 + spacing
                                + Math.max(deviceHeight, imagingSettings.implicitHeight, storageSettings.implicitHeight)
                            RowLayout {
                                id: leftHeaderRow; width: parent.width; height: 32; spacing: 8
                                HmiText {
                                    text: ["相机配置", "预览与校正", "数据存储"][page.tab]
                                    color: Theme.ink; font.pixelSize: Theme.appTitle; font.weight: Font.DemiBold
                                    Layout.fillWidth: true
                                }
                                // Camera 1 | Camera 2 sub-tab switcher on Devices tab
                                Rectangle {
                                    visible: page.tab === 0
                                    Layout.preferredHeight: 30; implicitWidth: camSubRow.implicitWidth + 6
                                    radius: 7; color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                                    RowLayout {
                                        id: camSubRow; anchors.centerIn: parent; spacing: 2
                                        CollectButton {
                                            objectName: "settingsCam1SubTab"; text: "相机 1"; small: true; segmented: true
                                            implicitHeight: 24; leftPadding: 10; rightPadding: 10
                                            selected: page.camSubTab === 0; onClicked: page.camSubTab = 0
                                        }
                                        CollectButton {
                                            objectName: "settingsCam2SubTab"; text: "相机 2"; small: true; segmented: true
                                            implicitHeight: 24; leftPadding: 10; rightPadding: 10
                                            selected: page.camSubTab === 1; onClicked: page.camSubTab = 1
                                        }
                                    }
                                }
                            }

                            // TAB 0 - Camera 1 Device Fields
                            Column {
                                id: camera1DeviceFields; visible: page.tab === 0 && page.camSubTab === 0; width: parent.width; spacing: 8
                                HmiText { text: "相机 1 IP"; font.pixelSize: Theme.meta }
                                CollectField { objectName: "cameraIpInput"; width: parent.width; text: page.cameraIp; enabled: !page.locked && !page.controller.cameraOpened; onTextChanged: page.cameraIp = text }
                                HmiText { visible: page.controller.cameraOpened; width: parent.width; text: "相机 1 已连接，断开后可编辑 IP 与分辨率"; color: Theme.warningText; font.pixelSize: Theme.micro }
                                HmiText { text: "相机 1 分辨率 · px"; font.pixelSize: Theme.meta; topPadding: 6 }
                                RowLayout {
                                    width: parent.width; spacing: 8
                                    CollectField { objectName: "resolutionWidthInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.previewWidth; enabled: !page.locked && !page.controller.cameraOpened; onTextChanged: page.previewWidth = text }
                                    HmiText { text: "×"; color: Theme.ink3 }
                                    CollectField { objectName: "resolutionHeightInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.previewHeight; enabled: !page.locked && !page.controller.cameraOpened; onTextChanged: page.previewHeight = text }
                                }
                                Rectangle {
                                    width: parent.width; height: 34; radius: 6
                                    color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                                    HmiText { anchors.fill: parent; anchors.leftMargin: 10; anchors.rightMargin: 10; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight; wrapMode: Text.NoWrap; text: "后端：" + page.controller.cameraBackend + " · " + page.controller.cameraModel; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink2 }
                                }
                            }

                            // TAB 0 - Camera 2 Device Fields
                            Column {
                                id: camera2DeviceFields; visible: page.tab === 0 && page.camSubTab === 1; width: parent.width; spacing: 8
                                HmiText { width: parent.width; text: "相机 2 IP（可选，与序列号二选一）"; font.pixelSize: Theme.meta }
                                CollectField { objectName: "camera2IpInput"; width: parent.width; text: page.camera2Ip; placeholderText: "192.168.66.254"; enabled: !page.locked && !page.controller.camera2Opened; onTextChanged: page.camera2Ip = text }
                                HmiText { width: parent.width; text: "相机 2 序列号（可选）"; font.pixelSize: Theme.meta }
                                CollectField { objectName: "camera2SnInput"; width: parent.width; text: page.camera2Sn; placeholderText: "如 DA1234567"; enabled: !page.locked && !page.controller.camera2Opened; onTextChanged: page.camera2Sn = text }
                                HmiText { visible: page.controller.camera2Opened; width: parent.width; text: "相机 2 已连接，断开后可编辑 IP、序列号、分辨率与增益"; color: Theme.warningText; font.pixelSize: Theme.micro }
                                RowLayout {
                                    width: parent.width; spacing: 8
                                    ColumnLayout {
                                        Layout.fillWidth: true; Layout.preferredWidth: 1; Layout.minimumWidth: 0; spacing: 4
                                        HmiText { text: "宽度 · px"; font.pixelSize: Theme.micro }
                                        CollectField { objectName: "resolution2WidthInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.preview2Width; enabled: !page.locked && !page.controller.camera2Opened; onTextChanged: page.preview2Width = text }
                                    }
                                    ColumnLayout {
                                        Layout.fillWidth: true; Layout.preferredWidth: 1; Layout.minimumWidth: 0; spacing: 4
                                        HmiText { text: "高度 · px"; font.pixelSize: Theme.micro }
                                        CollectField { objectName: "resolution2HeightInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.preview2Height; enabled: !page.locked && !page.controller.camera2Opened; onTextChanged: page.preview2Height = text }
                                    }
                                    ColumnLayout {
                                        Layout.preferredWidth: 84; Layout.minimumWidth: 64; spacing: 4
                                        HmiText { text: "增益"; font.pixelSize: Theme.micro }
                                        CollectField { objectName: "camera2GainInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.camera2Gain; enabled: !page.locked && !page.controller.camera2Opened; onTextChanged: page.camera2Gain = text }
                                    }
                                }
                            }

                            // TAB 1 - Preview Exposure & Undistortion
                            Column {
                                id: imagingSettings; visible: page.tab === 1; width: parent.width; spacing: 10
                                RowLayout {
                                    width: parent.width; spacing: 10
                                    ColumnLayout {
                                        Layout.fillWidth: true; Layout.preferredWidth: 1; Layout.minimumWidth: 0; spacing: 4
                                        HmiText { text: "相机 1 预览曝光 · μs"; font.pixelSize: Theme.meta }
                                        CollectField { objectName: "previewExposureInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.previewExposure; enabled: !page.locked; onTextChanged: page.previewExposure = text }
                                    }
                                    ColumnLayout {
                                        Layout.fillWidth: true; Layout.preferredWidth: 1; Layout.minimumWidth: 0; spacing: 4
                                        HmiText { text: "相机 2 预览曝光 · μs"; font.pixelSize: Theme.meta }
                                        CollectField { objectName: "preview2ExposureInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.preview2Exposure; enabled: !page.locked; onTextChanged: page.preview2Exposure = text }
                                    }
                                }
                                HmiText { width: parent.width; text: "预览曝光独立于融合序列，保存后生效（1～10000000 μs）。"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                Rectangle { width: parent.width; height: 1; color: Theme.hairline }
                                CollectButton {
                                    objectName: "undistortionButton"; width: parent.width; small: true; implicitHeight: 36
                                    positive: page.controller.undistortionEnabled
                                    text: page.controller.undistortionEnabled ? "相机 1 去畸变已启用" : "启用相机 1 去畸变"
                                    enabled: !page.locked && page.controller.cameraOpened && (page.controller.undistortionReady || page.controller.undistortionEnabled)
                                    onClicked: page.controller.toggleUndistortion()
                                }
                                HmiText { objectName: "undistortionStatus"; width: parent.width; text: page.controller.undistortionStatus; font.pixelSize: Theme.micro; color: page.controller.undistortionReady ? Theme.okText : Theme.ink3 }
                                CollectButton {
                                    objectName: "undistortion2Button"; width: parent.width; small: true; implicitHeight: 36
                                    positive: page.controller.undistortion2Enabled
                                    text: page.controller.undistortion2Enabled ? "相机 2 去畸变已启用" : "启用相机 2 去畸变"
                                    enabled: !page.locked && page.controller.camera2Opened && (page.controller.undistortion2Ready || page.controller.undistortion2Enabled)
                                    onClicked: page.controller.toggleUndistortion2()
                                }
                                HmiText { objectName: "undistortion2Status"; width: parent.width; text: page.controller.undistortion2Status; font.pixelSize: Theme.micro; color: page.controller.undistortion2Ready ? Theme.okText : Theme.ink3 }
                                HmiText { width: parent.width; text: "校正开关即时生效（也可在主页预览栏一键切换）；曝光需保存。"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                            }

                            // TAB 2 - Storage
                            Column {
                                id: storageSettings; visible: page.tab === 2; width: parent.width; spacing: 12
                                Rectangle {
                                    width: parent.width; height: 38; radius: 6
                                    color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                                    HmiText {
                                        anchors.fill: parent; anchors.leftMargin: 12; anchors.rightMargin: 12
                                        verticalAlignment: Text.AlignVCenter; color: Theme.ink; font.family: Theme.mono; font.pixelSize: Theme.meta
                                        text: "图片 " + page.controller.savedImageCount +
                                            (page.controller.savedGroupCount > 0 ? "（" + page.controller.savedGroupCount + " 组）" : "") +
                                            "   ·   TCP 点位 " + page.controller.savedPoseCount
                                    }
                                }
                                Column {
                                    width: parent.width; spacing: 4
                                    HmiText { text: "保存根目录"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                    Rectangle {
                                        width: parent.width; height: outDirText.implicitHeight + 16; radius: 6
                                        color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                                        HmiText { id: outDirText; anchors.fill: parent; anchors.margins: 8; text: page.controller.outputDirectory; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink; wrapMode: Text.WrapAnywhere }
                                    }
                                }
                                CollectButton { objectName: "collectionFolderButton"; small: true; implicitHeight: 34; text: "打开图片目录"; onClicked: page.controller.openCollectionFolder() }
                                HmiText { width: parent.width; text: page.controller.lastCapturePath || "尚未保存图片"; font.pixelSize: Theme.micro; font.family: Theme.mono; color: Theme.ink3; wrapMode: Text.WrapAnywhere }
                                HmiText { visible: page.controller.lastCapturePath2 !== ""; width: parent.width; text: "相机 2：" + page.controller.lastCapturePath2; font.pixelSize: Theme.micro; font.family: Theme.mono; color: Theme.ink3; wrapMode: Text.WrapAnywhere }
                                HmiText { width: parent.width; text: "保存保留原图、融合图、校正图及位姿 JSON；预览十字线不写入图片。"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                            }
                        }
                    }

                    // RIGHT CARD
                    HmiPanel {
                        objectName: "settingsRightCard"; width: (parent.width - 14) / 2; height: parent.height; x: width + 14
                        Column {
                            id: rightContent; width: parent.width; spacing: 14
                            readonly property real requiredHeight: 32 + spacing
                                + Math.max(robotSettings.implicitHeight, fusionSettings.implicitHeight, diagnosticSettings.implicitHeight)
                            HmiText { id: rightHeading; height: 32; verticalAlignment: Text.AlignVCenter; text: ["机器人配置", "融合曝光序列", "诊断与标定"][page.tab]; color: Theme.ink; font.pixelSize: Theme.appTitle; font.weight: Font.DemiBold }

                            // TAB 0 - Robot Settings
                            Column {
                                id: robotSettings; visible: page.tab === 0; width: parent.width; spacing: 8
                                HmiText { text: "机器人 IP"; font.pixelSize: Theme.meta }
                                CollectField { objectName: "robotIpInput"; width: parent.width; text: page.robotIp; enabled: !page.locked && !page.controller.robotConnected; onTextChanged: page.robotIp = text }
                                HmiText { visible: page.controller.robotConnected; width: parent.width; text: "机器人已连接，断开后可编辑 IP"; color: Theme.warningText; font.pixelSize: Theme.micro }
                                HmiText { text: "自动采集移动速度 · mm/s"; font.pixelSize: Theme.meta; topPadding: 6 }
                                CollectField {
                                    objectName: "robotSpeedInput"; width: parent.width
                                    text: page.robotSpeed; enabled: !page.locked
                                    validator: IntValidator { bottom: 1; top: 1000 }
                                    onTextChanged: page.robotSpeed = text
                                }
                                HmiText { width: parent.width; text: "范围 1～1000 mm/s，用于九点及按配置自动采图；手动示教速度在主页面实时设置。"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                HmiText { width: parent.width; text: "TCP 工具切换、实时位姿与微调均位于主页面底部示教区。"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                            }

                            // TAB 1 - Horizontal 5-Column Exposure Sequences
                            Column {
                                id: fusionSettings; visible: page.tab === 1; width: parent.width; spacing: 10
                                HmiText { width: parent.width; text: "按输入顺序采集 2～5 档，空白档位不参与融合（单位 μs）。"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                // Cam 1 Sequence
                                RowLayout {
                                    width: parent.width
                                    HmiText { text: "相机 1 融合序列"; font.pixelSize: Theme.meta; font.weight: Font.DemiBold; color: Theme.ink }
                                    Item { Layout.fillWidth: true }
                                    HmiText {
                                        text: "恢复默认四档"; font.pixelSize: Theme.micro; color: Theme.accent
                                        opacity: page.locked ? 0.4 : 1
                                        MouseArea { anchors.fill: parent; enabled: !page.locked; cursorShape: Qt.PointingHandCursor; onClicked: page.restoreDefaultExposures(exposureSlots) }
                                    }
                                }
                                RowLayout {
                                    width: parent.width; spacing: 6
                                    Repeater {
                                        model: exposureSlots
                                        ColumnLayout {
                                            Layout.fillWidth: true; Layout.preferredWidth: 1; Layout.minimumWidth: 0; spacing: 3
                                            HmiText { Layout.fillWidth: true; horizontalAlignment: Text.AlignHCenter; text: "#" + (index + 1); font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                            CollectField {
                                                objectName: index === 0 ? "exposureInput" : "exposureInput" + (index + 1)
                                                Layout.fillWidth: true; Layout.minimumWidth: 0; implicitHeight: 38; leftPadding: 4; rightPadding: 4
                                                horizontalAlignment: TextInput.AlignHCenter; font.pixelSize: Theme.meta
                                                text: value; placeholderText: "可选"; enabled: !page.locked; invalid: page.exposureError !== ""
                                                onTextChanged: { if (page.loaded) { exposureSlots.setProperty(index, "value", text); ++page.revision } }
                                            }
                                        }
                                    }
                                }
                                HmiText { objectName: "exposureError"; visible: page.exposureError !== ""; width: parent.width; text: page.exposureError; color: Theme.danger; font.pixelSize: Theme.micro }

                                Rectangle { width: parent.width; height: 1; color: Theme.hairline }

                                // Cam 2 Sequence
                                RowLayout {
                                    width: parent.width
                                    HmiText { text: "相机 2 融合序列"; font.pixelSize: Theme.meta; font.weight: Font.DemiBold; color: Theme.ink }
                                    Item { Layout.fillWidth: true }
                                    HmiText {
                                        text: "恢复默认四档"; font.pixelSize: Theme.micro; color: Theme.accent
                                        opacity: page.locked ? 0.4 : 1
                                        MouseArea { anchors.fill: parent; enabled: !page.locked; cursorShape: Qt.PointingHandCursor; onClicked: page.restoreDefaultExposures(exposure2Slots) }
                                    }
                                }
                                RowLayout {
                                    width: parent.width; spacing: 6
                                    Repeater {
                                        model: exposure2Slots
                                        ColumnLayout {
                                            Layout.fillWidth: true; Layout.preferredWidth: 1; Layout.minimumWidth: 0; spacing: 3
                                            HmiText { Layout.fillWidth: true; horizontalAlignment: Text.AlignHCenter; text: "#" + (index + 1); font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                            CollectField {
                                                objectName: index === 0 ? "exposure2Input" : "exposure2Input" + (index + 1)
                                                Layout.fillWidth: true; Layout.minimumWidth: 0; implicitHeight: 38; leftPadding: 4; rightPadding: 4
                                                horizontalAlignment: TextInput.AlignHCenter; font.pixelSize: Theme.meta
                                                text: value; placeholderText: "可选"; enabled: !page.locked; invalid: page.exposure2Error !== ""
                                                onTextChanged: { if (page.loaded) { exposure2Slots.setProperty(index, "value", text); ++page.revision } }
                                            }
                                        }
                                    }
                                }
                                HmiText { objectName: "exposure2Error"; visible: page.exposure2Error !== ""; width: parent.width; text: page.exposure2Error; color: Theme.danger; font.pixelSize: Theme.micro }
                            }

                            // TAB 2 - Diagnostics
                            Column {
                                id: diagnosticSettings; visible: page.tab === 2; width: parent.width; spacing: 12
                                Rectangle {
                                    width: parent.width; height: 38; radius: 6
                                    color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                                    HmiText { anchors.fill: parent; anchors.leftMargin: 12; anchors.rightMargin: 12; verticalAlignment: Text.AlignVCenter; text: "版本 " + appVersion + " · " + appGitCommit; font.family: Theme.mono; font.pixelSize: Theme.meta; color: Theme.ink }
                                }
                                Column {
                                    width: parent.width; spacing: 4
                                    HmiText { text: "相机 1 校正参数"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                    Rectangle {
                                        width: parent.width; height: he1Text.implicitHeight + 14; radius: 6
                                        color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                                        HmiText { id: he1Text; anchors.fill: parent; anchors.margins: 7; text: page.controller.handEyePath; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink2; wrapMode: Text.WrapAnywhere }
                                    }
                                }
                                Column {
                                    width: parent.width; spacing: 4
                                    HmiText { text: "相机 2 校正参数"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                                    Rectangle {
                                        width: parent.width; height: he2Text.implicitHeight + 14; radius: 6
                                        color: Theme.surfaceAlt; border.width: 1; border.color: Theme.hairline
                                        HmiText { id: he2Text; anchors.fill: parent; anchors.margins: 7; text: page.controller.handEyePath2; font.family: Theme.mono; font.pixelSize: Theme.micro; color: Theme.ink2; wrapMode: Text.WrapAnywhere }
                                    }
                                }
                                CollectButton { objectName: "diagnosticFolderButton"; small: true; implicitHeight: 34; text: "打开日志目录"; onClicked: page.controller.openDiagnosticFolder() }
                                HmiText { width: parent.width; text: "完整运行日志保留在诊断目录，主页面显示实时状态与最近结果。"; font.pixelSize: Theme.micro; color: Theme.ink3 }
                            }
                        }
                    }
                }
            }
        }

        // STICKY BOTTOM FOOTER
        Rectangle {
            objectName: "settingsFooter"; Layout.fillWidth: true; Layout.preferredHeight: 62; color: Theme.surface
            Rectangle { anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top; height: 1; color: Theme.hairline }
            RowLayout {
                anchors.fill: parent; anchors.leftMargin: 16; anchors.rightMargin: 16; spacing: 10
                HmiText {
                    objectName: "settingsFeedback"; Layout.fillWidth: true; font.pixelSize: Theme.meta
                    elide: Text.ElideRight; wrapMode: Text.NoWrap
                    text: page.controller.settingsSaving ? "正在保存配置，请稍候…"
                        : page.locked ? "操作进行中，设置已锁定"
                        : page.feedback + (page.dirty ? " · " + page.dirtyCount + " 项未保存" : "")
                    color: page.dirty ? Theme.warningText : Theme.ink2
                }
                CollectButton { objectName: "resetSettingsButton"; text: "还原编辑"; small: true; quiet: true; implicitHeight: 36; enabled: page.dirty && !page.locked; onClicked: page.reload() }
                CollectButton { objectName: "saveSettingsButton"; text: page.controller.settingsSaving ? "保存中…" : "保存配置"; small: true; primary: true; implicitHeight: 36; Layout.preferredWidth: 132; enabled: !page.locked && page.dirty; onClicked: page.save() }
            }
        }
    }

    Dialog {
        id: leaveConfirm; objectName: "dirtySettingsDialog"; anchors.centerIn: parent; width: Math.min(460, page.width - 32); modal: true
        header: HmiText { objectName: "dirtyDialogTitle"; text: leaveConfirm.title; color: Theme.ink; font.pixelSize: Theme.appTitle; font.weight: Font.DemiBold; padding: 18 }
        title: "配置尚未保存"; closePolicy: Popup.CloseOnEscape
        background: Rectangle { color: Theme.surface; radius: Theme.radiusCard; border.color: Theme.hairlineStrong }
        contentItem: ColumnLayout {
            spacing: 14
            HmiText { text: "离开将放弃本次编辑。"; Layout.fillWidth: true; font.pixelSize: Theme.body }
            RowLayout {
                Item { Layout.fillWidth: true }
                CollectButton { objectName: "keepSettingsButton"; small: true; text: "继续编辑"; onClicked: leaveConfirm.close() }
                CollectButton { objectName: "discardSettingsButton"; small: true; text: "放弃并返回"; danger: true; solidDanger: true; onClicked: { leaveConfirm.close(); page.reload(); page.leave() } }
            }
        }
    }
}
