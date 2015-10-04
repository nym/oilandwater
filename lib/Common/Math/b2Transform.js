/**
*  Class b2Transform
*
* @param pos
* @param r
*
*/
b2Transform = Box2D.Common.Math.b2Transform = function b2Transform(pos, r) {
    this.position = new b2Vec2(0, 0);
    this.R = new b2Mat22();
    if (pos) {
        this.position.SetV(pos);
        this.R.SetM(r);
    }
};
b2Transform.constructor = b2Transform;
b2Transform.prototype = {


    position: null,
    R: null,


    /**
     * Initialize
     *
     * @param pos
     * @param r
     *
     */
    Initialize: function (pos, r) {
        this.position.SetV(pos);
        this.R.SetM(r);
        return this;
    },

    /**
     * SetIdentity
     *
     * @param
     *
     */
    SetIdentity: function () {
        this.position.SetZero();
        this.R.SetIdentity();
        return this;
    },

    /**
     * Set
     *
     * @param x
     *
     */
    Set: function (x) {
        this.position.SetV(x.position);
        this.R.SetM(x.R);
        return this;
    },

    /**
     * GetAngle
     *
     * @param
     *
     */
    GetAngle: function () {
        return Math.atan2(this.R.col1.y, this.R.col1.x);
    }
};