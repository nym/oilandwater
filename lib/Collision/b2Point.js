
   /**
    *  Class b2Point
    *
    * @param 
    *
    */
   b2Point = Box2D.Collision.b2Point = function b2Point() {
      this.p = new b2Vec2(0, 0);

   };
   b2Point.constructor = b2Point;
   b2Point.prototype.p = null;

   /**
    * Support
    *
    * @param xf
    * @param vX
    * @param vY
    *
    */
   b2Point.prototype.Support = function (xf, vX, vY) {
      return this.p;
   };

   /**
    * GetFirstVertex
    *
    * @param xf
    *
    */
   b2Point.prototype.GetFirstVertex = function (xf) {
      return this.p;
   };