/**
*  Class b2Sweep
*
* @param 
*
*/
b2Sweep = Box2D.Common.Math.b2Sweep = function b2Sweep() {
  this.localCenter = new b2Vec2(0, 0);
  this.c0 = new b2Vec2(0, 0);
  this.c = new b2Vec2(0, 0);

};
b2Sweep.constructor = b2Sweep;
b2Sweep.prototype = {


    localCenter: null,
    c: null,
    c0: null,
    a: 0,
    a0: 0,
    t0: 0,

    /**
     * Set
     *
     * @param other
     *
     */
    Set: function (other) {
        this.localCenter.SetV(other.localCenter);
        this.c0.SetV(other.c0);
        this.c.SetV(other.c);
        this.a0 = other.a0;
        this.a = other.a;
        this.t0 = other.t0;
        return this;
    },

    /**
     * Copy
     *
     * @param
     *
     */
    Copy: function () {
        var copy = new b2Sweep();
        copy.localCenter.SetV(this.localCenter);
        copy.c0.SetV(this.c0);
        copy.c.SetV(this.c);
        copy.a0 = this.a0;
        copy.a = this.a;
        copy.t0 = this.t0;
        return copy;
    },

    /**
     * GetTransform
     *
     * @param xf
     * @param alpha
     *
     */
    GetTransform: function (xf, alpha) {
        alpha = alpha || 0;
        xf.position.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
        xf.position.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
        var angle = (1.0 - alpha) * this.a0 + alpha * this.a;
        xf.R.Set(angle);
        var tMat = xf.R;
        xf.position.x -= (tMat.col1.x * this.localCenter.x + tMat.col2.x * this.localCenter.y);
        xf.position.y -= (tMat.col1.y * this.localCenter.x + tMat.col2.y * this.localCenter.y);
    },

    /**
     * Advance
     *
     * @param t
     *
     */
    Advance: function (t) {
        t = t || 0;
        if (this.t0 < t && 1.0 - this.t0 > b2Settings.b2_epsilon) {
            var alpha = (t - this.t0) / (1.0 - this.t0);
            this.c0.x = (1.0 - alpha) * this.c0.x + alpha * this.c.x;
            this.c0.y = (1.0 - alpha) * this.c0.y + alpha * this.c.y;
            this.a0 = (1.0 - alpha) * this.a0 + alpha * this.a;
            this.t0 = t;
        }
    }
};