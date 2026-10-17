import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "."

Button {
    id: control
    property bool primary: false
    property bool positive: false
    property bool quiet: false
    property bool danger: false
    property string iconName: ""
    readonly property color textInk: !enabled ? (primary ? Theme.primaryDisabledInk : Theme.disabledInk)
        : primary ? Theme.accentInk : positive ? Theme.ok : danger ? Theme.danger
        : quiet ? (hovered ? Theme.ink : Theme.ink2) : Theme.ink
    implicitHeight: Theme.controlHeight
    implicitWidth: contentItem.implicitWidth + leftPadding + rightPadding
    leftPadding: 14
    rightPadding: 14
    topPadding: 0
    bottomPadding: 0
    hoverEnabled: true
    font.family: Theme.sans
    font.pixelSize: Theme.body
    font.weight: Font.Medium
    contentItem: RowLayout {
        spacing: 6
        CollectIcon {
            visible: control.iconName !== ""
            name: control.iconName
            ink: control.textInk
            Layout.preferredWidth: 16
            Layout.preferredHeight: 16
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
        radius: Theme.radiusControl
        color: !control.enabled ? (control.primary ? Theme.primaryDisabledBg : control.quiet ? "transparent" : Theme.disabledBg)
            : control.primary ? (control.down ? Theme.accentDown : control.hovered ? Theme.accentHover : Theme.accent)
            : control.positive ? Theme.okSoft
            : control.danger ? (control.hovered ? Theme.dangerSoft : "transparent")
            : control.quiet ? "transparent"
            : control.down ? Theme.raisedDown : control.hovered ? Theme.raisedHover : Theme.raised
        border.width: control.primary && control.enabled || control.quiet ? 0 : 1
        border.color: control.activeFocus ? Theme.accent
            : !control.enabled ? Theme.hairline
            : control.positive ? Theme.okLine : control.danger ? Theme.dangerLine
            : control.hovered ? Theme.lineHover : Theme.line
        Behavior on color { ColorAnimation { duration: 120 } }
        Behavior on border.color { ColorAnimation { duration: 120 } }
    }
}
