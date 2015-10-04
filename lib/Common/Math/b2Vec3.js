/**
*  Class b2Vec3
*
* @param x
* @param y
* @param z
*
*/
b2Vec3 = Box2D.Common.Math.b2Vec3 = function b2Vec3(x, y, z) {

  this.x = x;
  this.y = y;
  this.z = z;
};
b2Vec3.constructor = b2Vec3;
b2Vec3.prototype = {

    x: 0,
    y: 0,
    z: 0,
    /**
     * SetZero
     *
     * @param
     *
     */
    SetZero: function () {
        this.x = this.y = this.z = 0.0;
        return this;
    },

    /**
     * Set
     *
     * @param x
     * @param y
     * @param z
     *
     */
    Set: function (x, y, z) {
        this.x = x;
        this.y = y;
        this.z = z;
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
        this.z = v.z;
        return this;
    },

    /**
     * GetNegative
     *
     * @param
     *
     */
    GetNegative: function () {
        return new b2Vec3((-this.x), (-this.y), (-this.z));
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
        this.z = (-this.z);
        return this;
    },

    /**
     * Copy
     *
     * @param
     *
     */
    Copy: function () {
        return new b2Vec3(this.x, this.y, this.z);
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
        this.z += v.z;
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
        this.z -= v.z;
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
        this.z *= a;
        return this;
    }
}