
   /**
    *  Class b2ContactPoint
    *
    * @param 
    *
    */
   b2ContactPoint = Box2D.Collision.b2ContactPoint = function b2ContactPoint() {
      this.position = new b2Vec2(0, 0);
      this.velocity = new b2Vec2(0, 0);
      this.normal = new b2Vec2(0, 0);
      this.id = new b2ContactID();

   };
   b2ContactPoint.constructor = b2ContactPoint;

   b2ContactPoint.prototype.position = null;
   b2ContactPoint.prototype.velocity = null;
   b2ContactPoint.prototype.normal = null;
   b2ContactPoint.prototype.id = null;