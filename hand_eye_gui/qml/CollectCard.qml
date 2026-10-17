import QtQuick 2.15
import QtQuick.Controls 2.15
import "."

Pane {
    padding: Theme.padding
    background: Rectangle {
        radius: Theme.radiusCard
        color: Theme.surface
        border.width: 1
        border.color: Theme.hairline
        Rectangle { x: Theme.radiusCard; y: 1; width: parent.width - Theme.radiusCard * 2; height: 1; color: Theme.highlight }
    }
}
