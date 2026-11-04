import QtQuick 2.15
import QtQuick.Controls 2.15
import "."
Button {
    id: control
    property bool primary: false
    property bool quiet: false
    property bool danger: false
    property bool selected: false
    property bool small: false
    implicitHeight: small ? 32 : Theme.controlHeight
    implicitWidth: contentItem.implicitWidth + leftPadding + rightPadding
    leftPadding: small ? 12 : 16
    rightPadding: leftPadding
    topPadding: 0; bottomPadding: 0
    hoverEnabled: true
    opacity: enabled ? 1 : 0.4
    font.family: Theme.sans
    font.pixelSize: small ? Theme.body : Theme.title
    font.weight: primary || selected ? Font.DemiBold : Font.Normal
    contentItem: Text {
        text: control.text; font: control.font
        color: control.primary ? Theme.white : control.danger ? Theme.danger : Theme.ink
        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
    }
    background: Rectangle {
        radius: Theme.radiusControl
        color: control.primary ? Theme.accent : control.down || control.hovered ? Theme.raisedHover : control.quiet ? "transparent" : Theme.raised
        border.width: control.danger ? 1.5 : 1
        border.color: control.activeFocus ? Theme.accent : control.danger ? Theme.danger : control.selected ? Theme.accentLine : control.primary ? Theme.accent : Theme.hairline
    }
}
