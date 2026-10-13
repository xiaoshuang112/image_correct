import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Button {
    id: control
    property bool primary: false
    property bool quiet: false
    property bool danger: false
    property string iconName: ""
    readonly property color textInk: !enabled ? (primary ? "#7387ad" : "#a3a8ac")
        : primary ? "#ffffff" : danger ? "#b75050" : "#334052"
    implicitHeight: 38
    implicitWidth: contentItem.implicitWidth + 28
    leftPadding: 14
    rightPadding: 14
    hoverEnabled: true
    font.family: "Noto Sans CJK SC"
    font.pixelSize: 13
    font.weight: Font.Medium
    contentItem: RowLayout {
        spacing: 7
        CollectIcon {
            visible: control.iconName !== ""
            name: control.iconName
            ink: control.textInk
            Layout.preferredWidth: 17
            Layout.preferredHeight: 17
        }
        Text {
            text: control.text
            color: control.textInk
            font: control.font
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            Layout.fillWidth: true
        }
    }
    background: Rectangle {
        radius: 8
        color: !control.enabled ? (control.primary ? "#dce5f7" : "#f3f4f1")
            : control.primary ? (control.down ? "#234fc9" : control.hovered ? "#3268ee" : "#3b70f1")
            : control.down ? "#e5e9ef" : control.hovered ? "#edf1f8" : control.quiet ? "transparent" : "#ffffff"
        border.width: control.primary ? 0 : 1
        border.color: control.activeFocus ? "#3b70f1" : control.quiet ? "transparent" : "#dfe3e5"
        Behavior on color { ColorAnimation { duration: 100 } }
    }
}
