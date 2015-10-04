
   /**
    *  Class b2Color
    *
    * @param rr
    * @param gg
    * @param bb
    *
    */
   b2Color = Box2D.Common.b2Color = function b2Color(rr, gg, bb) {
      this._r = Box2D.parseUInt(255 * b2Math.Clamp(rr || 0, 0.0, 1.0));
      this._g = Box2D.parseUInt(255 * b2Math.Clamp(gg || 0, 0.0, 1.0));
      this._b = Box2D.parseUInt(255 * b2Math.Clamp(bb || 0, 0.0, 1.0));
   };
   b2Color.constructor = b2Color;
   b2Color.prototype._r = 0;
   b2Color.prototype._g = 0;
   b2Color.prototype._b = 0;

   /**
    * Set
    *
    * @param rr
    * @param gg
    * @param bb
    *
    */
   b2Color.prototype.Set = function (rr, gg, bb) {
      this._r = Box2D.parseUInt(255 * b2Math.Clamp(rr, 0.0, 1.0));
      this._g = Box2D.parseUInt(255 * b2Math.Clamp(gg, 0.0, 1.0));
      this._b = Box2D.parseUInt(255 * b2Math.Clamp(bb, 0.0, 1.0));
   };