
   /**
    *  Class b2ContactConstraint
    *
    * @param 
    *
    */
   b2ContactConstraint = Box2D.Dynamics.Contacts.b2ContactConstraint = function b2ContactConstraint() {
      this.localPlaneNormal = new b2Vec2(0, 0);
      this.localPoint = new b2Vec2(0, 0);
      this.normal = new b2Vec2(0, 0);
      this.normalMass = new b2Mat22();
      this.K = new b2Mat22();
      this.points = [];
      for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++) {
         this.points.push(new b2ContactConstraintPoint());
      }
   };
   b2ContactConstraint.constructor = b2ContactConstraint;
   b2ContactConstraint.prototype.localPlaneNormal  = null;
   b2ContactConstraint.prototype.localPoint        = null;
   b2ContactConstraint.prototype.normal            = null;
   b2ContactConstraint.prototype.normalMass        = null;
   b2ContactConstraint.prototype.K                 = null;
   b2ContactConstraint.prototype.points            = null;
