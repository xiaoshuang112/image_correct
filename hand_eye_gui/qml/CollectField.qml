import QtQuick 2.15
import QtQuick.Controls 2.15
import "."
TextField {
    id: control
    property bool mono: true
    property bool invalid: false
    property bool draft: false
    implicitHeight: Theme.controlHeight; implicitWidth: 140
    leftPadding: 10; rightPadding: 10
    opacity: enabled ? 1 : 0.42
    font.family: mono ? Theme.mono : Theme.sans; font.pixelSize: Theme.body
    color: Theme.ink; placeholderTextColor: Theme.ink3
    selectionColor: Theme.accentSelection; selectedTextColor: Theme.ink
    background: Rectangle {
        radius: Theme.radiusControl
        color: control.draft ? "#14F59E0B" : Theme.surfaceAlt
        border.width: control.activeFocus || control.invalid || control.draft ? 1.5 : 1
        border.color: control.invalid ? Theme.danger
                    : control.activeFocus ? Theme.accent
                    : control.draft ? Theme.warning
                    : Theme.hairline
    }
}
