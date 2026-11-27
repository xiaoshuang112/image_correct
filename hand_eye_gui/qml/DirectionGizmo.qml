import QtQuick 2.15
import "."
Rectangle {
    id: gizmo
    property string directionX: "none"
    property string directionY: "none"
    color: Theme.overlay
    border.width: 1
    border.color: Theme.hairline
    radius: 8
    function vector(direction) { return ({up: [0,-1], down: [0,1], left: [-1,0], right: [1,0], none: [0,0]})[direction] || [0,0] }
    onDirectionXChanged: drawing.requestPaint()
    onDirectionYChanged: drawing.requestPaint()
    Canvas {
        id: drawing; anchors.fill: parent
        onPaint: {
            var ctx = getContext("2d")
            var c = width / 2
            var length = width * 0.27
            ctx.clearRect(0, 0, width, height)
            var x = gizmo.vector(gizmo.directionX)
            var y = gizmo.vector(gizmo.directionY)
            // Right-angle square indicator when both axes are set
            if ((x[0] || x[1]) && (y[0] || y[1])) {
                var s = Math.max(6, width * 0.08)
                ctx.strokeStyle = Theme.ink3
                ctx.lineWidth = 1.2
                ctx.beginPath()
                ctx.moveTo(c + x[0] * s, c + x[1] * s)
                ctx.lineTo(c + (x[0] + y[0]) * s, c + (x[1] + y[1]) * s)
                ctx.lineTo(c + y[0] * s, c + y[1] * s)
                ctx.stroke()
            }
            var vectors = [x, y]
            var colors = [Theme.axisX, Theme.axisY]
            var labels = ["X+", "Y+"]
            ctx.lineCap = "round"
            ctx.lineJoin = "round"
            for (var i = 0; i < 2; ++i) {
                var v = vectors[i]
                if (!v[0] && !v[1]) continue
                var ex = c + v[0] * length
                var ey = c + v[1] * length
                ctx.strokeStyle = colors[i]
                ctx.fillStyle = colors[i]
                ctx.lineWidth = 2.2
                ctx.beginPath()
                ctx.moveTo(c, c)
                ctx.lineTo(ex, ey)
                ctx.moveTo(ex - v[0] * 6 - v[1] * 4, ey - v[1] * 6 + v[0] * 4)
                ctx.lineTo(ex, ey)
                ctx.lineTo(ex - v[0] * 6 + v[1] * 4, ey - v[1] * 6 - v[0] * 4)
                ctx.stroke()
                ctx.font = "bold 11px sans-serif"
                ctx.textAlign = "center"
                ctx.textBaseline = "middle"
                ctx.fillText(labels[i], c + v[0] * (length + 13), c + v[1] * (length + 13))
            }
            // Origin dot
            ctx.fillStyle = Theme.accent
            ctx.beginPath()
            ctx.arc(c, c, 2.5, 0, Math.PI * 2)
            ctx.fill()
        }
    }
}
