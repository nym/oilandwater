/**
*  Class b2Mat22
*
* @param 
*
*/
b2Mat22 = Box2D.Common.Math.b2Mat22 = function b2Mat22() {
  this.col1 = new b2Vec2(0, 0);
  this.col2 = new b2Vec2(0, 0);
  this.SetIdentity();
};
/**
 * Static FromAngle
 *
 * @param angle
 *
 */
b2Mat22.FromAngle = function (angle) {
    var mat = new b2Mat22();
    mat.Set(angle);
    return mat;
};

/**
 * Static FromVV
 *
 * @param c1
 * @param c2
 *
 */
b2Mat22.FromVV = function (c1, c2) {
    var mat = new b2Mat22();
    mat.SetVV(c1, c2);
    return mat;
};

b2Mat22.constructor = b2Mat22;
b2Mat22.prototype = {

    col1: null,
    col2: null,


    /**
     * Set
     *
     * @param angle
     *
     */
    Set: function (angle) {
        var c = Math.cos(angle),
            s = Math.sin(angle);
        this.col1.x = c;
        this.col2.x = (-s);
        this.col1.y = s;
        this.col2.y = c;
        return this;
    },

    /**
     * SetVV
     *
     * @param c1
     * @param c2
     *
     */
    SetVV: function (c1, c2) {
        this.col1.SetV(c1);
        this.col2.SetV(c2);
        return this;
    },

    /**
     * Copy
     *
     * @param
     *
     */
    Copy: function () {
        var mat = new b2Mat22();
        mat.SetM(this);
        return mat;
    },

    /**
     * SetM
     *
     * @param m
     *
     */
    SetM: function (m) {
        this.col1.SetV(m.col1);
        this.col2.SetV(m.col2);
        return this;
    },

    /**
     * AddM
     *
     * @param m
     *
     */
    AddM: function (m) {
        this.col1.x += m.col1.x;
        this.col1.y += m.col1.y;
        this.col2.x += m.col2.x;
        this.col2.y += m.col2.y;
        return this;
    },

    /**
     * SetIdentity
     *
     * @param
     *
     */
    SetIdentity: function () {
        this.col1.x = 1.0;
        this.col2.x = 0.0;
        this.col1.y = 0.0;
        this.col2.y = 1.0;
        return this;
    },

    /**
     * SetZero
     *
     * @param
     *
     */
    SetZero: function () {
        this.col1.x = 0.0;
        this.col2.x = 0.0;
        this.col1.y = 0.0;
        this.col2.y = 0.0;
        return this;
    },

    /**
     * GetAngle
     *
     * @param
     *
     */
    GetAngle: function () {
        return Math.atan2(this.col1.y, this.col1.x);
    },

    /**
     * GetInverse
     *
     * @param out
     *
     */
    GetInverse: function (out) {
        var a = this.col1.x,
            b = this.col2.x,
            c = this.col1.y,
            d = this.col2.y,
            det = a * d - b * c;
        if (det !== 0.0) {
            det = 1.0 / det;
        }
        out.col1.x = det * d;
        out.col2.x = (-det * b);
        out.col1.y = (-det * c);
        out.col2.y = det * a;
        return out;
    },

    /**
     * Solve
     *
     * @param out
     * @param bX
     * @param bY
     *
     */
    Solve: function (out, bX, bY) {
        var a11 = this.col1.x,
            a12 = this.col2.x,
            a21 = this.col1.y,
            a22 = this.col2.y,
            det = a11 * a22 - a12 * a21;
        if (det !== 0.0) {
            det = 1.0 / det;
        }
        out.x = det * (a22 * bX - a12 * bY);
        out.y = det * (a11 * bY - a21 * bX);
        return out;
    },

    /**
     * Abs
     *
     * @param
     *
     */
    Abs: function () {
        this.col1.Abs();
        this.col2.Abs();
        return this;
    }
};