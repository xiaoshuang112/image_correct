import QtQuick 2.15
import QtQuick.Layouts 1.15
import "."

RowLayout {
    property string number: ""
    property string title: ""
    Layout.fillWidth: true
    Layout.preferredHeight: 20
    spacing: 8
    Text { visible: parent.number !== ""; text: parent.number; color: Theme.accent; font.family: Theme.mono; font.pixelSize: Theme.meta; font.weight: Font.Medium }
    Text { text: parent.title; color: Theme.ink; font.pixelSize: Theme.title; font.weight: Font.Medium }
    Item { Layout.fillWidth: true; Layout.minimumWidth: 0 }
}
