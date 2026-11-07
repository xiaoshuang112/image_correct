import QtQuick 2.15
import "."
Rectangle {
    id: gizmo
    property string directionX: "none"
    property string directionY: "none"
    color: Theme.overlay; radius: 6
    function vector(direction) { return ({up: [0,-1], down: [0,1], left: [-1,0], right: [1,0], none: [0,0]})[direction] || [0,0] }
    onDirectionXChanged: drawing.requestPaint()
    onDirectionYChanged: drawing.requestPaint()
    Canvas {
        id: drawing; anchors.fill: parent
        onPaint: {
            var ctx = getContext("2d"), c = width / 2, length = width * 0.28
            ctx.clearRect(0,0,width,height); ctx.strokeStyle = Theme.ink; ctx.fillStyle = Theme.ink; ctx.lineWidth = 2
            var vectors = [gizmo.vector(gizmo.directionX), gizmo.vector(gizmo.directionY)]
            for (var i = 0; i < 2; ++i) {
                var v = vectors[i]
                if (!v[0] && !v[1]) continue
                var ex = c + v[0] * length, ey = c + v[1] * length
                ctx.beginPath(); ctx.moveTo(c,c); ctx.lineTo(ex,ey)
                ctx.moveTo(ex-v[0]*6-v[1]*4,ey-v[1]*6+v[0]*4); ctx.lineTo(ex,ey); ctx.lineTo(ex-v[0]*6+v[1]*4,ey-v[1]*6-v[0]*4); ctx.stroke()
                ctx.font = "12px sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle"
                ctx.fillText(i === 0 ? "X+" : "Y+", c + v[0]*(length+13), c + v[1]*(length+13))
            }
            var x = vectors[0], y = vectors[1], s = 7
            ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(c+x[0]*s,c+x[1]*s); ctx.lineTo(c+(x[0]+y[0])*s,c+(x[1]+y[1])*s); ctx.lineTo(c+y[0]*s,c+y[1]*s); ctx.stroke()
        }
    }
}
