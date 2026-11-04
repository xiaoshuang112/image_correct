import QtQuick 2.15
import QtQuick.Controls 2.15
import "."
TextField {
    id: control
    property bool mono: true
    property bool invalid: false
    implicitHeight: Theme.controlHeight; implicitWidth: 140
    leftPadding: 12; rightPadding: 12
    opacity: enabled ? 1 : 0.4
    font.family: mono ? Theme.mono : Theme.sans; font.pixelSize: Theme.title
    color: Theme.ink; placeholderTextColor: Theme.ink3
    selectionColor: Theme.accentSelection; selectedTextColor: Theme.ink
    background: Rectangle {
        radius: Theme.radiusControl; color: Theme.bg; border.width: 1
        border.color: control.invalid ? Theme.danger : control.activeFocus ? Theme.accent : Theme.hairline
    }
}
