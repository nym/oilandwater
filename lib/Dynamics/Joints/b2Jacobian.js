
   /**
    *  Class b2Jacobian
    *
    * @param 
    *
    */
   b2Jacobian = Box2D.Dynamics.Joints.b2Jacobian = function b2Jacobian() {
      this.linearA = new b2Vec2(0, 0);
      this.linearB = new b2Vec2(0, 0);

   };
   b2Jacobian.constructor = b2Jacobian;
   b2Jacobian.prototype.linearA           = null;
   b2Jacobian.prototype.linearB           = null;
   b2Jacobian.prototype.angularA          = null;
   b2Jacobian.prototype.angularB          = null;

   /**
    * SetZero
    *
    * @param 
    *
    */
   b2Jacobian.prototype.SetZero = function () {
      this.linearA.SetZero();
      this.angularA = 0.0;
      this.linearB.SetZero();
      this.angularB = 0.0;
   };

   /**
    * Set
    *
    * @param x1
    * @param a1
    * @param x2
    * @param a2
    *
    */
   b2Jacobian.prototype.Set = function (x1, a1, x2, a2) {
      a1 = a1 || 0;
      a2 = a2 || 0;
      this.linearA.SetV(x1);
      this.angularA = a1;
      this.linearB.SetV(x2);
      this.angularB = a2;
   };

   /**
    * Compute
    *
    * @param x1
    * @param a1
    * @param x2
    * @param a2
    *
    */
   b2Jacobian.prototype.Compute = function (x1, a1, x2, a2) {
      a1 = a1 || 0;
      a2 = a2 || 0;
      return (this.linearA.x * x1.x + this.linearA.y * x1.y) + this.angularA * a1 + (this.linearB.x * x2.x + this.linearB.y * x2.y) + this.angularB * a2;
   };