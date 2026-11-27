import QtQuick 2.15
import "."
Canvas {
    id: icon
    property color ink: Theme.accent
    implicitWidth: 26; implicitHeight: 26
    onInkChanged: requestPaint()
    onWidthChanged: requestPaint()
    onHeightChanged: requestPaint()
    onPaint: {
        var ctx = getContext("2d")
        ctx.reset()
        ctx.scale(width / 24, height / 24)
        ctx.strokeStyle = ink
        ctx.lineWidth = 2.0
        ctx.lineCap = "round"
        ctx.lineJoin = "round"
        // Four viewfinder corners
        ctx.beginPath()
        ctx.moveTo(3, 8); ctx.lineTo(3, 5); ctx.arcTo(3, 3, 5, 3, 2); ctx.lineTo(8, 3)
        ctx.moveTo(16, 3); ctx.lineTo(19, 3); ctx.arcTo(21, 3, 21, 5, 2); ctx.lineTo(21, 8)
        ctx.moveTo(21, 16); ctx.lineTo(21, 19); ctx.arcTo(21, 21, 19, 21, 2); ctx.lineTo(16, 21)
        ctx.moveTo(8, 21); ctx.lineTo(5, 21); ctx.arcTo(3, 21, 3, 19, 2); ctx.lineTo(3, 16)
        ctx.stroke()
        // Center lens circle
        ctx.beginPath()
        ctx.arc(12, 12, 4.2, 0, Math.PI * 2)
        ctx.stroke()
        ctx.fillStyle = ink
        ctx.beginPath()
        ctx.arc(13.2, 10.8, 1.3, 0, Math.PI * 2)
        ctx.fill()
    }
}
