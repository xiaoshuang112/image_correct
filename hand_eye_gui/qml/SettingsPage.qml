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
        Rectangle {
            color: Theme.surface; Layout.fillWidth: true; Layout.preferredHeight: 64
            RowLayout {
                anchors.fill: parent; anchors.margins: 12; spacing: 16
                CollectButton { objectName: "settingsBackButton"; text: "← 返回"; quiet: true; enabled: !page.locked; onClicked: page.requestLeave() }
                HmiText { text: "采集 › 设置"; color: Theme.ink; font.pixelSize: Theme.title; Layout.fillWidth: true }
                HmiText { text: page.dirty ? page.dirtyCount + " 项未保存" : "配置"; color: page.dirty ? Theme.warning : Theme.ink3 }
            }
        }
        Item {
            Layout.fillWidth: true; Layout.preferredHeight: 76
            RowLayout {
                width: Math.min(parent.width - 32, Theme.pageContentWidth); height: Theme.controlHeight
                anchors.centerIn: parent; spacing: 8
                Repeater {
                    model: ["设备", "成像", "存储与诊断"]
                    CollectButton { objectName: "settingsTab" + index; text: modelData; selected: page.tab === index; onClicked: page.tab = index }
                }
                Item { Layout.fillWidth: true }
            }
        }
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
                    // Reserve the tallest tab so switching only changes the visible form.
                    height: Math.max(leftContent.requiredHeight + Theme.padding * 2,
                        rightContent.requiredHeight + Theme.padding * 2,
                        Math.min(Theme.pageCardMinHeight, Math.max(0, scroll.availableHeight - 32)))
                    HmiPanel {
                        objectName: "settingsLeftCard"; width: (parent.width - 16) / 2; height: parent.height; x: 0
                        Column {
                            id: leftContent; width: parent.width; spacing: 16
                            readonly property real requiredHeight: leftHeading.implicitHeight + spacing
                                + Math.max(cameraSettings.implicitHeight, imagingSettings.implicitHeight, storageSettings.implicitHeight)
                            HmiText { id: leftHeading; text: ["相机", "预览与校正", "数据存储"][page.tab]; color: Theme.ink; font.pixelSize: Theme.appTitle }
                            Column {
                                id: cameraSettings; visible: page.tab === 0; width: parent.width; spacing: 8
                                HmiText { text: "相机 IP" }
                                CollectField { objectName: "cameraIpInput"; width: parent.width; text: page.cameraIp; enabled: !page.locked && !page.controller.cameraOpened; onTextChanged: page.cameraIp = text }
                                HmiText { visible: page.controller.cameraOpened; width: parent.width; text: "相机已连接，断开后可编辑 IP 与分辨率"; font.pixelSize: Theme.meta }
                                HmiText { text: "预览分辨率 · px"; topPadding: 8 }
                                RowLayout {
                                    width: parent.width; spacing: 8
                                    CollectField { objectName: "resolutionWidthInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.previewWidth; enabled: !page.locked && !page.controller.cameraOpened; onTextChanged: page.previewWidth = text }
                                    HmiText { text: "×" }
                                    CollectField { objectName: "resolutionHeightInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.previewHeight; enabled: !page.locked && !page.controller.cameraOpened; onTextChanged: page.previewHeight = text }
                                }
                                HmiText { width: parent.width; text: "后端：" + page.controller.cameraBackend + " · " + page.controller.cameraModel; font.pixelSize: Theme.meta }
                                HmiText { text: "相机 2"; topPadding: 16; font.pixelSize: Theme.body; font.weight: Font.DemiBold }
                                HmiText { width: parent.width; text: "相机 2 IP（可选，与序列号二选一）" }
                                CollectField { objectName: "camera2IpInput"; width: parent.width; text: page.camera2Ip; enabled: !page.locked && !page.controller.camera2Opened; onTextChanged: page.camera2Ip = text }
                                HmiText { width: parent.width; text: "相机 2 序列号（可选）" }
                                CollectField { objectName: "camera2SnInput"; width: parent.width; text: page.camera2Sn; enabled: !page.locked && !page.controller.camera2Opened; onTextChanged: page.camera2Sn = text }
                                HmiText { visible: page.controller.camera2Opened; width: parent.width; text: "相机 2 已连接，断开后可编辑 IP、序列号、分辨率与增益"; font.pixelSize: Theme.meta }
                                HmiText { width: parent.width; text: "相机 2 分辨率 · px"; topPadding: 8 }
                                RowLayout {
                                    width: parent.width; spacing: 8
                                    CollectField { objectName: "resolution2WidthInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.preview2Width; enabled: !page.locked && !page.controller.camera2Opened; onTextChanged: page.preview2Width = text }
                                    HmiText { text: "×" }
                                    CollectField { objectName: "resolution2HeightInput"; Layout.fillWidth: true; Layout.minimumWidth: 0; text: page.preview2Height; enabled: !page.locked && !page.controller.camera2Opened; onTextChanged: page.preview2Height = text }
                                }
                                HmiText { width: parent.width; text: "相机 2 增益"; font.pixelSize: Theme.meta }
                                CollectField { objectName: "camera2GainInput"; width: parent.width; text: page.camera2Gain; enabled: !page.locked && !page.controller.camera2Opened; onTextChanged: page.camera2Gain = text }
                            }
                            Column {
                                id: imagingSettings; visible: page.tab === 1; width: parent.width; spacing: 8
                                HmiText { text: "预览曝光 · μs" }
                                CollectField { objectName: "previewExposureInput"; width: parent.width; text: page.previewExposure; enabled: !page.locked; onTextChanged: page.previewExposure = text }
                                HmiText { width: parent.width; text: "预览曝光独立于融合序列；保存后生效。范围 1～10000000 μs。"; font.pixelSize: Theme.meta }
                                HmiText { text: "相机 2 预览曝光 · μs"; topPadding: 12 }
                                CollectField { objectName: "preview2ExposureInput"; width: parent.width; text: page.preview2Exposure; enabled: !page.locked; onTextChanged: page.preview2Exposure = text }
                                Item { width: 1; height: 8 }
                                CollectButton {
                                    objectName: "undistortionButton"; width: parent.width
                                    text: page.controller.undistortionEnabled ? "相机 1 去畸变已启用" : "启用相机 1 去畸变"
                                    enabled: !page.locked && page.controller.cameraOpened && (page.controller.undistortionReady || page.controller.undistortionEnabled)
                                    onClicked: page.controller.toggleUndistortion()
                                }
                                HmiText { objectName: "undistortionStatus"; width: parent.width; text: page.controller.undistortionStatus; font.pixelSize: Theme.meta }
                                CollectButton {
                                    objectName: "undistortion2Button"; width: parent.width
                                    text: page.controller.undistortion2Enabled ? "相机 2 去畸变已启用" : "启用相机 2 去畸变"
                                    enabled: !page.locked && page.controller.camera2Opened && (page.controller.undistortion2Ready || page.controller.undistortion2Enabled)
                                    onClicked: page.controller.toggleUndistortion2()
                                }
                                HmiText { objectName: "undistortion2Status"; width: parent.width; text: page.controller.undistortion2Status; font.pixelSize: Theme.meta }
                                HmiText { width: parent.width; text: "校正开关即时生效于预览 / 拍照；曝光配置需要保存。"; font.pixelSize: Theme.meta }
                            }
                            Column {
                                id: storageSettings; visible: page.tab === 2; width: parent.width; spacing: 16
                                HmiText {
                                    width: parent.width; color: Theme.ink; font.family: Theme.mono
                                    text: "图片 " + page.controller.savedImageCount +
                                        (page.controller.savedGroupCount > 0 ? "（" + page.controller.savedGroupCount + " 组）" : "") +
                                        "   TCP 点位 " + page.controller.savedPoseCount
                                }
                                HmiText { width: parent.width; text: page.controller.outputDirectory; font.family: Theme.mono; font.pixelSize: Theme.meta; wrapMode: Text.WrapAnywhere }
                                CollectButton { objectName: "collectionFolderButton"; text: "打开图片目录"; onClicked: page.controller.openCollectionFolder() }
                                HmiText { width: parent.width; text: page.controller.lastCapturePath || "尚未保存图片"; font.pixelSize: Theme.meta; wrapMode: Text.WrapAnywhere }
                                HmiText { visible: page.controller.lastCapturePath2 !== ""; width: parent.width; text: "相机 2：" + page.controller.lastCapturePath2; font.pixelSize: Theme.meta; wrapMode: Text.WrapAnywhere }
                                HmiText { width: parent.width; text: "保存仍保留现有原图、融合图、校正图及位姿数据。预览十字线不会写入图片。"; font.pixelSize: Theme.meta }
                            }
                        }
                    }
                    HmiPanel {
                        objectName: "settingsRightCard"; width: (parent.width - 16) / 2; height: parent.height; x: width + 16
                        Column {
                            id: rightContent; width: parent.width; spacing: 16
                            readonly property real requiredHeight: rightHeading.implicitHeight + spacing
                                + Math.max(robotSettings.implicitHeight, fusionSettings.implicitHeight, diagnosticSettings.implicitHeight)
                            HmiText { id: rightHeading; text: ["机器人", "融合曝光序列", "诊断"][page.tab]; color: Theme.ink; font.pixelSize: Theme.appTitle }
                            Column {
                                id: robotSettings; visible: page.tab === 0; width: parent.width; spacing: 8
                                HmiText { text: "机器人 IP" }
                                CollectField { objectName: "robotIpInput"; width: parent.width; text: page.robotIp; enabled: !page.locked && !page.controller.robotConnected; onTextChanged: page.robotIp = text }
                                HmiText { visible: page.controller.robotConnected; width: parent.width; text: "机器人已连接，断开后可编辑 IP"; font.pixelSize: Theme.meta }
                                HmiText { text: "九点采集速度 · mm/s"; topPadding: 8 }
                                CollectField {
                                    objectName: "robotSpeedInput"; width: parent.width
                                    text: page.robotSpeed; enabled: !page.locked
                                    validator: IntValidator { bottom: 1; top: 1000 }
                                    onTextChanged: page.robotSpeed = text
                                }
                                HmiText { width: parent.width; text: "范围 1～1000 mm/s，仅用于九点自动采集。手动平移和角度速度在主页面分别设置。"; font.pixelSize: Theme.meta }
                                HmiText { width: parent.width; text: "TCP 工具列表与实时位姿在主页面查看。移动前请确认所选工具和目标坐标。"; font.pixelSize: Theme.meta }
                            }
                            Column {
                                id: fusionSettings; visible: page.tab === 1; width: parent.width; spacing: 8
                                HmiText { width: parent.width; text: "按输入顺序采集 2～5 档；留空的档位不参与融合。单位 μs。"; font.pixelSize: Theme.meta }
                                Repeater {
                                    model: exposureSlots
                                    RowLayout {
                                        width: parent.width; spacing: 8
                                        HmiText { text: String(index + 1); font.family: Theme.mono; Layout.preferredWidth: 20 }
                                        CollectField {
                                            objectName: index === 0 ? "exposureInput" : "exposureInput" + (index + 1)
                                            Layout.fillWidth: true; Layout.minimumWidth: 0; text: value; placeholderText: "可选"; enabled: !page.locked; invalid: page.exposureError !== ""
                                            onTextChanged: { if (page.loaded) { exposureSlots.setProperty(index, "value", text); ++page.revision } }
                                        }
                                    }
                                }
                                HmiText { objectName: "exposureError"; width: parent.width; text: page.exposureError; color: Theme.danger; font.pixelSize: Theme.meta }
                                HmiText { text: "相机 2 融合曝光序列"; topPadding: 12; font.weight: Font.DemiBold }
                                Repeater {
                                    model: exposure2Slots
                                    RowLayout {
                                        width: parent.width; spacing: 8
                                        HmiText { text: String(index + 1); font.family: Theme.mono; Layout.preferredWidth: 20 }
                                        CollectField {
                                            objectName: index === 0 ? "exposure2Input" : "exposure2Input" + (index + 1)
                                            Layout.fillWidth: true; Layout.minimumWidth: 0; text: value; placeholderText: "可选"; enabled: !page.locked
                                            invalid: page.exposure2Error !== ""
                                            onTextChanged: { if (page.loaded) { exposure2Slots.setProperty(index, "value", text); ++page.revision } }
                                        }
                                    }
                                }
                                HmiText { objectName: "exposure2Error"; width: parent.width; text: page.exposure2Error; color: Theme.danger; font.pixelSize: Theme.meta }
                            }
                            Column {
                                id: diagnosticSettings; visible: page.tab === 2; width: parent.width; spacing: 16
                                HmiText { text: "版本 " + appVersion + " · " + appGitCommit; font.family: Theme.mono; width: parent.width }
                                HmiText { width: parent.width; text: "相机 1 校正参数\n" + page.controller.handEyePath; font.family: Theme.mono; font.pixelSize: Theme.meta; wrapMode: Text.WrapAnywhere }
                                HmiText { width: parent.width; text: "相机 2 校正参数\n" + page.controller.handEyePath2; font.family: Theme.mono; font.pixelSize: Theme.meta; wrapMode: Text.WrapAnywhere }
                                CollectButton { objectName: "diagnosticFolderButton"; text: "打开日志目录"; onClicked: page.controller.openDiagnosticFolder() }
                                HmiText { width: parent.width; text: "完整运行日志保留在诊断目录，主页面显示任务、进度与最近结果。"; font.pixelSize: Theme.meta }
                            }
                        }
                    }
                }
            }
        }
        Rectangle {
            objectName: "settingsFooter"; Layout.fillWidth: true; Layout.preferredHeight: 72; color: Theme.surface
            RowLayout {
                anchors.fill: parent; anchors.margins: 16; spacing: 12
                HmiText { objectName: "settingsFeedback"; Layout.fillWidth: true; text: page.controller.settingsSaving ? "正在保存配置，请稍候…" : page.locked ? "操作进行中，设置已锁定" : page.feedback + (page.dirty ? " · " + page.dirtyCount + " 项未保存" : ""); color: page.dirty ? Theme.warning : Theme.ink2 }
                CollectButton { objectName: "resetSettingsButton"; text: "还原编辑"; quiet: true; enabled: page.dirty && !page.locked; onClicked: page.reload() }
                CollectButton { objectName: "saveSettingsButton"; text: page.controller.settingsSaving ? "保存中…" : "保存配置"; primary: true; Layout.preferredWidth: 144; enabled: !page.locked && page.dirty; onClicked: page.save() }
            }
        }
    }
    Dialog {
        id: leaveConfirm; objectName: "dirtySettingsDialog"; anchors.centerIn: parent; width: Math.min(480, page.width - 32); modal: true
        header: HmiText { objectName: "dirtyDialogTitle"; text: leaveConfirm.title; color: Theme.ink; font.pixelSize: Theme.appTitle; padding: 20 }
        title: "配置尚未保存"; closePolicy: Popup.CloseOnEscape
        background: Rectangle { color: Theme.surface; radius: Theme.radiusCard; border.color: Theme.hairline }
        contentItem: ColumnLayout {
            spacing: 16
            HmiText { text: "离开将放弃本次编辑。"; Layout.fillWidth: true }
            RowLayout {
                Item { Layout.fillWidth: true }
                CollectButton { objectName: "keepSettingsButton"; text: "继续编辑"; onClicked: leaveConfirm.close() }
                CollectButton { objectName: "discardSettingsButton"; text: "放弃并返回"; danger: true; onClicked: { leaveConfirm.close(); page.reload(); page.leave() } }
            }
        }
    }
}
