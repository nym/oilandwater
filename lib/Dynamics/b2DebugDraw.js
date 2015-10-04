/**
*  Class b2DebugDraw
*
* @param 
*
*/
b2DebugDraw = Box2D.Dynamics.b2DebugDraw = function b2DebugDraw() {
    this.m_drawScale = 1.0;
    this.m_lineThickness = 1.0;
    this.m_alpha = 1.0;
    this.m_fillAlpha = 1.0;
    this.m_xformScale = 1.0;
    var __this = this;
    //#WORKAROUND
    this.m_sprite = {
     graphics: {
        clear: function () {
           __this.m_ctx.clearRect(0, 0, __this.m_ctx.canvas.width, __this.m_ctx.canvas.height)
        }
     }
    };
    this.m_drawFlags = 0;
};
b2DebugDraw.e_shapeBit           = 0x0001;
b2DebugDraw.e_jointBit           = 0x0002;
b2DebugDraw.e_aabbBit            = 0x0004;
b2DebugDraw.e_pairBit            = 0x0008;
b2DebugDraw.e_centerOfMassBit    = 0x0010;
b2DebugDraw.e_controllerBit      = 0x0020;
b2DebugDraw.constructor = b2DebugDraw;
b2DebugDraw.prototype = {
    m_drawScale         : 1.0,
    m_lineThickness     : 1.0,
    m_alpha             : 1.0,
    m_fillAlpha         : 1.0,
    m_xformScale        : 1.0,
    m_drawFlags         : 0x000000,
    m_ctx               : null,


    /**
     * SetFlags
     *
     * @param flags
     *
     */
    SetFlags: function (flags) {
        flags = flags || 0;
        this.m_drawFlags = flags;
    },

    /**
     * GetFlags
     *
     * @param
     *
     */
    GetFlags: function () {
        return this.m_drawFlags;
    },

    /**
     * AppendFlags
     *
     * @param flags
     *
     */
    AppendFlags: function (flags) {
        flags = flags || 0;
        this.m_drawFlags |= flags;
    },

    /**
     * ClearFlags
     *
     * @param flags
     *
     */
    ClearFlags: function (flags) {
        flags = flags || 0;
        this.m_drawFlags &= ~flags;
    },

    /**
     * SetSprite
     *
     * @param sprite
     *
     */
    SetSprite: function (sprite) {
        this.m_ctx = sprite;
    },

    /**
     * GetSprite
     *
     * @param
     *
     */
    GetSprite: function () {
        return this.m_ctx;
    },

    /**
     * SetDrawScale
     *
     * @param drawScale
     *
     */
    SetDrawScale: function (drawScale) {
        drawScale = drawScale || 0;
        this.m_drawScale = drawScale;
    },

    /**
     * GetDrawScale
     *
     * @param
     *
     */
    GetDrawScale: function () {
        return this.m_drawScale;
    },

    /**
     * SetLineThickness
     *
     * @param lineThickness
     *
     */
    SetLineThickness: function (lineThickness) {
        lineThickness = lineThickness || 0;
        this.m_lineThickness = lineThickness;
        this.m_ctx.strokeWidth = lineThickness;
    },

    /**
     * GetLineThickness
     *
     * @param
     *
     */
    GetLineThickness: function () {
        return this.m_lineThickness;
    },

    /**
     * SetAlpha
     *
     * @param alpha
     *
     */
    SetAlpha: function (alpha) {
        alpha = alpha || 0;
        this.m_alpha = alpha;
    },

    /**
     * GetAlpha
     *
     * @param
     *
     */
    GetAlpha: function () {
        return this.m_alpha;
    },

    /**
     * SetFillAlpha
     *
     * @param alpha
     *
     */
    SetFillAlpha: function (alpha) {
        alpha = alpha || 0;
        this.m_fillAlpha = alpha;
    },

    /**
     * GetFillAlpha
     *
     * @param
     *
     */
    GetFillAlpha: function () {
        return this.m_fillAlpha;
    },

    /**
     * SetXFormScale
     *
     * @param xformScale
     *
     */
    SetXFormScale: function (xformScale) {
        xformScale = xformScale || 0;
        this.m_xformScale = xformScale;
    },

    /**
     * GetXFormScale
     *
     * @param
     *
     */
    GetXFormScale: function () {
        return this.m_xformScale;
    },

    /**
     * DrawPolygon
     *
     * @param vertices
     * @param vertexCount
     * @param color
     *
     */
    DrawPolygon: function (vertices, vertexCount, color) {
        if (!vertexCount) return;
        var s = this.m_ctx,
            drawScale = this.m_drawScale;
        s.beginPath();
        s.strokeStyle = this._color(color.color, this.m_alpha);
        s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
        for (var i = 1; i < vertexCount; i++) {
            s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
        }
        s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
        s.closePath();
        s.stroke();
    },

    /**
     * DrawSolidPolygon
     *
     * @param vertices
     * @param vertexCount
     * @param color
     *
     */
    DrawSolidPolygon: function (vertices, vertexCount, color) {
        if (!vertexCount) return;
        var s = this.m_ctx,
            drawScale = this.m_drawScale;
        s.beginPath();
        s.strokeStyle = this._color(color.color, this.m_alpha);
        s.fillStyle = this._color(color.color, this.m_fillAlpha);
        s.moveTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
        for (var i = 1; i < vertexCount; i++) {
            s.lineTo(vertices[i].x * drawScale, vertices[i].y * drawScale);
        }
        s.lineTo(vertices[0].x * drawScale, vertices[0].y * drawScale);
        s.closePath();
        s.fill();
        s.stroke();
    },

    /**
     * DrawCircle
     *
     * @param center
     * @param radius
     * @param color
     *
     */
    DrawCircle: function (center, radius, color) {
        if (!radius) return;
        var s = this.m_ctx,
            drawScale = this.m_drawScale;
        s.beginPath();
        s.strokeStyle = this._color(color.color, this.m_alpha);
        s.arc(center.x * drawScale, center.y * drawScale, radius * drawScale, 0, Math.PI * 2, true);
        s.closePath();
        s.stroke();
    },

    /**
     * DrawSolidCircle
     *
     * @param center
     * @param radius
     * @param axis
     * @param color
     *
     */
    DrawSolidCircle: function (center, radius, axis, color) {
        if (!radius) return;
        var s = this.m_ctx,
            drawScale = this.m_drawScale,
            cx = center.x * drawScale,
            cy = center.y * drawScale;
        s.moveTo(0, 0);
        s.beginPath();
        s.strokeStyle = this._color(color.color, this.m_alpha);
        s.fillStyle = this._color(color.color, this.m_fillAlpha);
        s.arc(cx, cy, radius * drawScale, 0, Math.PI * 2, true);
        s.moveTo(cx, cy);
        s.lineTo((center.x + axis.x * radius) * drawScale, (center.y + axis.y * radius) * drawScale);
        s.closePath();
        s.fill();
        s.stroke();
    },

    /**
     * DrawSegment
     *
     * @param p1
     * @param p2
     * @param color
     *
     */
    DrawSegment: function (p1, p2, color) {
        var s = this.m_ctx,
            drawScale = this.m_drawScale;
        s.strokeStyle = this._color(color.color, this.m_alpha);
        s.beginPath();
        s.moveTo(p1.x * drawScale, p1.y * drawScale);
        s.lineTo(p2.x * drawScale, p2.y * drawScale);
        s.closePath();
        s.stroke();
    },

    /**
     * DrawTransform
     *
     * @param xf
     *
     */
    DrawTransform: function (xf) {
        var s = this.m_ctx,
            drawScale = this.m_drawScale;
        s.beginPath();
        s.strokeStyle = this._color(0xff0000, this.m_alpha);
        s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
        s.lineTo((xf.position.x + this.m_xformScale * xf.R.col1.x) * drawScale, (xf.position.y + this.m_xformScale * xf.R.col1.y) * drawScale);

        s.strokeStyle = this._color(0xff00, this.m_alpha);
        s.moveTo(xf.position.x * drawScale, xf.position.y * drawScale);
        s.lineTo((xf.position.x + this.m_xformScale * xf.R.col2.x) * drawScale, (xf.position.y + this.m_xformScale * xf.R.col2.y) * drawScale);
        s.closePath();
        s.stroke();
    },

    /**
    * _color
    *
    * @param color
    * @param alpha
    *
    */
    _color: function (color, alpha) {
       return "rgba(" + ((color & 0xFF0000) >> 16) + "," + ((color & 0xFF00) >> 8) + "," + (color & 0xFF) + "," + alpha + ")";
    }
};

