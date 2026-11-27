import QtQuick 2.15
import QtQuick.Controls 2.15
import "."
Button {
    id: control
    property bool primary: false
    property bool quiet: false
    property bool danger: false
    property bool solidDanger: false
    property bool selected: false
    property bool segmented: false
    property bool positive: false
    property bool warningTone: false
    property bool pill: false
    property bool small: false
    property bool statusDot: false
    property color dotColor: Theme.ink3
    property bool dot2Visible: false
    property color dot2Color: Theme.ink3
    implicitHeight: small ? 32 : Theme.controlHeight
    implicitWidth: contentRow.implicitWidth + leftPadding + rightPadding
    leftPadding: small ? 10 : 14
    rightPadding: leftPadding
    topPadding: 0; bottomPadding: 0
    hoverEnabled: true
    opacity: enabled ? 1 : 0.42
    font.family: Theme.sans
    font.pixelSize: small ? Theme.meta : Theme.body
    font.weight: primary || selected || solidDanger || positive ? Font.DemiBold : Font.Normal

    contentItem: Item {
        implicitWidth: contentRow.implicitWidth
        implicitHeight: contentRow.implicitHeight
        Row {
            id: contentRow
            anchors.centerIn: parent
            spacing: 6
            Row {
                visible: control.statusDot
                anchors.verticalCenter: parent.verticalCenter
                spacing: 3
                Rectangle {
                    width: 7; height: 7; radius: 3.5
                    color: control.dotColor
                }
                Rectangle {
                    visible: control.dot2Visible
                    width: 7; height: 7; radius: 3.5
                    color: control.dot2Color
                }
            }
            Text {
                anchors.verticalCenter: parent.verticalCenter
                text: control.text
                font: control.font
                color: (control.primary || control.solidDanger || (control.segmented && control.selected)) ? Theme.white
                     : control.danger ? Theme.danger
                     : control.positive ? Theme.okText
                     : control.warningTone ? Theme.warningText
                     : (control.segmented && !control.selected) ? (control.hovered ? Theme.ink : Theme.ink2)
                     : Theme.ink
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                elide: Text.ElideRight
            }
        }
    }

    background: Rectangle {
        radius: control.pill ? Math.round(height / 2) : Theme.radiusControl
        color: control.solidDanger ? (control.down || control.hovered ? Theme.dangerHover : Theme.danger)
             : control.primary ? (control.down || control.hovered ? Theme.accentHover : Theme.accent)
             : control.segmented ? (control.selected ? Theme.accent : control.down || control.hovered ? Theme.raised : "transparent")
             : control.positive ? (control.down || control.hovered ? "#2E22C55E" : Theme.okSoft)
             : control.warningTone ? (control.down || control.hovered ? "#30F59E0B" : Theme.warningSoft)
             : control.selected ? Theme.accentSoft
             : control.down || control.hovered ? Theme.raisedHover
             : control.quiet ? "transparent"
             : Theme.raised
        border.width: control.segmented && !control.selected ? 0 : (control.danger && !control.solidDanger ? 1.5 : 1)
        border.color: control.activeFocus ? Theme.accent
                    : control.solidDanger ? Theme.danger
                    : control.danger ? Theme.danger
                    : control.positive ? Theme.okLine
                    : control.warningTone ? Theme.warningLine
                    : control.segmented && control.selected ? Theme.accent
                    : control.selected ? Theme.accentLine
                    : control.primary ? Theme.accent
                    : Theme.hairline
    }
}
