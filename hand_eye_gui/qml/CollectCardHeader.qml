import QtQuick 2.15
import QtQuick.Layouts 1.15
import "."

RowLayout {
    id: header
    property string number: ""
    property string title: ""
    property color tint: Theme.accent
    property color tintSoft: Theme.accentSoft
    Layout.fillWidth: true
    Layout.preferredHeight: 20
    spacing: 8
    Rectangle {
        visible: header.number !== ""
        implicitWidth: numberText.implicitWidth + 10; implicitHeight: 18; radius: Theme.radiusSmall
        color: header.tintSoft
        Text { id: numberText; anchors.centerIn: parent; text: header.number; color: header.tint; font.family: Theme.mono; font.pixelSize: Theme.meta; font.weight: Font.Medium }
    }
    Text { text: header.title; color: Theme.ink; font.pixelSize: Theme.title; font.weight: Font.Medium }
    Item { Layout.fillWidth: true; Layout.minimumWidth: 0 }
}
