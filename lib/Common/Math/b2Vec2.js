/**
*  Class b2Vec2
*
* @param x
* @param y
*
*/
b2Vec2 = Box2D.Common.Math.b2Vec2 = function b2Vec2(x, y) {
  this.x = x;
  this.y = y;
};
/**
 * Static Make
 *
 * @param x
 * @param y
 *
 */
b2Vec2.Make = function (x, y) {
    return new b2Vec2(x, y);
};
b2Vec2.constructor = b2Vec2;
b2Vec2.prototype = {
    x: 0,
    y: 0,
    /**
     * SetZero
     *
     * @param
     *
     */
    SetZero: function () {
        this.x = 0;
        this.y = 0;
        return this;
    },

    /**
     * Set
     *
     * @param x
     * @param y
     *
     */
    Set: function (x, y) {
        this.x = x;
        this.y = y;
        return this;
    },

    /**
     * SetV
     *
     * @param v
     *
     */
    SetV: function (v) {
        this.x = v.x;
        this.y = v.y;
        return this;
    },

    /**
     * GetNegative
     *
     * @param
     *
     */
    GetNegative: function () {
        return new b2Vec2((-this.x), (-this.y));
    },

    /**
     * NegativeSelf
     *
     * @param
     *
     */
    NegativeSelf: function () {
        this.x = (-this.x);
        this.y = (-this.y);
        return this;
    },

    /**
     * Copy
     *
     * @param
     *
     */
    Copy: function () {
        return new b2Vec2(this.x, this.y);
    },

    /**
     * Add
     *
     * @param v
     *
     */
    Add: function (v) {
        this.x += v.x;
        this.y += v.y;
        return this;
    },

    /**
     * Subtract
     *
     * @param v
     *
     */
    Subtract: function (v) {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    },

    /**
     * Multiply
     *
     * @param a
     *
     */
    Multiply: function (a) {
        this.x *= a;
        this.y *= a;
        return this;
    },

    /**
     * MulM
     *
     * @param A
     *
     */
    MulM: function (A) {
        var tX = this.x;
        this.x = A.col1.x * tX + A.col2.x * this.y;
        this.y = A.col1.y * tX + A.col2.y * this.y;
        return this;
    },

    /**
     * MulTM
     *
     * @param A
     *
     */
    MulTM: function (A) {
        var tX = b2Math.Dot(this, A.col1);
        this.y = b2Math.Dot(this, A.col2);
        this.x = tX;
        return this;
    },

    /**
     * CrossVF
     *
     * @param s
     *
     */
    CrossVF: function (s) {
        var tX = this.x;
        this.x = s * this.y;
        this.y = (-s * tX);
        return this;
    },

    /**
     * CrossFV
     *
     * @param s
     *
     */
    CrossFV: function (s) {
        var tX = this.x;
        this.x = (-s * this.y);
        this.y = s * tX;
        return this;
    },

    /**
     * MinV
     *
     * @param b
     *
     */
    MinV: function (b) {
        this.x = this.x < b.x ? this.x : b.x;
        this.y = this.y < b.y ? this.y : b.y;
        return this;
    },

    /**
     * MaxV
     *
     * @param b
     *
     */
    MaxV: function (b) {
        this.x = this.x > b.x ? this.x : b.x;
        this.y = this.y > b.y ? this.y : b.y;
        return this;
    },

    /**
     * Abs
     *
     * @param
     *
     */
    Abs: function () {
        if (this.x < 0) this.x = (-this.x);
        if (this.y < 0) this.y = (-this.y);
        return this;
    },

    /**
     * Length
     *
     * @param
     *
     */
    Length: function () {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    },

    /**
     * LengthSquared
     *
     * @param
     *
     */
    LengthSquared: function () {
        return (this.x * this.x + this.y * this.y);
    },

    /**
     * Normalize
     *
     * @param
     *
     */
    Normalize: function () {
        var length = Math.sqrt(this.x * this.x + this.y * this.y);
        if (length < b2Settings.b2_epsilon) {
            return 0.0;
        }
        var invLength = 1.0 / length;
        this.x *= invLength;
        this.y *= invLength;
        return length;
    },

    /**
     * IsValid
     *
     * @param
     *
     */
    IsValid: function () {
        return b2Math.IsValid(this.x) && b2Math.IsValid(this.y);
    }
}