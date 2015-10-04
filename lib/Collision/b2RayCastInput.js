
   /**
    *  Class b2RayCastInput
    *
    * @param p1
    * @param p2
    * @param maxFraction
    *
    */
   b2RayCastInput = Box2D.Collision.b2RayCastInput = function b2RayCastInput(p1, p2, maxFraction) {
      this.p1 = new b2Vec2(0, 0);
      this.p2 = new b2Vec2(0, 0);
      p1 = p1 || null;
      p2 = p2 || null;
      maxFraction = maxFraction || 1;
      if (p1) this.p1.SetV(p1);
      if (p2) this.p2.SetV(p2);
      this.maxFraction = maxFraction;
   };
   b2RayCastInput.constructor = b2RayCastInput;

   b2RayCastInput.prototype.p1 = null;
   b2RayCastInput.prototype.p2 = null;
   b2RayCastInput.prototype.maxFraction = 1;

