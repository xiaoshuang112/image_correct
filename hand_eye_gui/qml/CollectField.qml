import QtQuick 2.15
import QtQuick.Controls 2.15
import "."

TextField {
    id: control
    property bool mono: true
    implicitHeight: Theme.controlHeight
    implicitWidth: 140
    leftPadding: 10
    rightPadding: 10
    font.family: mono ? Theme.mono : Theme.sans
    font.pixelSize: Theme.body
    color: enabled ? Theme.ink : Theme.ink2
    placeholderTextColor: Theme.ink3
    selectionColor: Theme.accentSelection
    selectedTextColor: Theme.ink
    background: Rectangle {
        radius: Theme.radiusControl
        color: control.enabled ? Theme.sunken : Theme.disabledBg
        border.width: 1
        border.color: control.activeFocus ? Theme.accent : control.enabled ? Theme.line : Theme.hairline
        Behavior on border.color { ColorAnimation { duration: 120 } }
    }
}
