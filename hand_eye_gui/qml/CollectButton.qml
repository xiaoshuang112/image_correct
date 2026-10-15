import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Button {
    id: control
    property bool primary: false
    property bool quiet: false
    property bool danger: false
    property string iconName: ""
    readonly property color textInk: !enabled ? (primary ? "#8190a6" : "#6b7583")
        : primary ? "#0c2919" : danger ? "#f87171" : "#9ccaff"
    implicitHeight: 36
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
        radius: 7
        color: !control.enabled ? (control.primary ? "#2b3749" : control.quiet ? "transparent" : "#1d232b")
            : control.primary ? (control.down ? "#61c68d" : control.hovered ? "#91ebba" : "#73dfa1")
            : control.down ? "#303b49" : control.hovered ? "#293341" : control.quiet ? "transparent" : (control.danger ? "#2c1618" : "#162435")
        border.width: control.primary ? 0 : 1
        border.color: control.activeFocus ? "#53769e" : control.quiet ? "transparent" : (control.danger ? "#633033" : "#355776")
        Behavior on color { ColorAnimation { duration: 100 } }
    }
}
