
   /**
    *  Class b2ContactConstraintPoint
    *
    * @param 
    *
    */
   b2ContactConstraintPoint = Box2D.Dynamics.Contacts.b2ContactConstraintPoint = function b2ContactConstraintPoint() {
      this.localPoint = new b2Vec2(0, 0);
      this.rA = new b2Vec2(0, 0);
      this.rB = new b2Vec2(0, 0);

   };
   b2ContactConstraintPoint.constructor = b2ContactConstraintPoint;
   b2ContactConstraintPoint.prototype.localPoint    = null;
   b2ContactConstraintPoint.prototype.rA            = null;
   b2ContactConstraintPoint.prototype.rB            = null;