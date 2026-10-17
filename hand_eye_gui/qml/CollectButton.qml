import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Button {
    id: control
    property bool primary: false
    property bool positive: false
    property bool quiet: false
    property bool danger: false
    property string iconName: ""
    readonly property color textInk: !enabled ? "#8295aa"
        : positive ? "#a0e1bf" : primary ? "#10253e" : danger ? "#d39aa2" : "#a3cfff"
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
        radius: 6
        color: !control.enabled ? (control.primary ? "#25384f" : control.quiet ? "transparent" : "#1c2531")
            : control.positive ? (control.down ? "#224e3c" : control.hovered ? "#214735" : "#1a382e")
            : control.primary ? (control.down ? "#5a9fdf" : control.hovered ? "#97cbff" : "#77baff")
            : control.down ? "#303f51" : control.hovered ? "#29394d" : control.quiet ? "transparent" : (control.danger ? "#291f29" : "#1d2a3a")
        border.width: 1
        border.color: control.activeFocus ? "#a6d5ff" : control.quiet ? "transparent" : control.positive ? "#3f8165" : control.danger ? "#5b3d47" : control.primary && control.enabled ? "#77baff" : "#365675"
        Behavior on color { ColorAnimation { duration: 100 } }
    }
}
