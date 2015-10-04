
   /**
    *  Class b2FrictionJoint
    *
    * @param def
    *
    */
   b2FrictionJoint = Box2D.Dynamics.Joints.b2FrictionJoint = function b2FrictionJoint(def) {

      this.m_localAnchorA = new b2Vec2(0, 0);
      this.m_localAnchorB = new b2Vec2(0, 0);
      this.m_linearMass = new b2Mat22();
      this.m_linearImpulse = new b2Vec2(0, 0);
      b2Joint.call(this, def);
      this.m_localAnchorA.SetV(def.localAnchorA);
      this.m_localAnchorB.SetV(def.localAnchorB);
      this.m_linearMass.SetZero();
      this.m_linearImpulse.SetZero();
      this.m_maxForce = def.maxForce;
      this.m_maxTorque = def.maxTorque;
   };
   b2FrictionJoint.constructor = b2FrictionJoint;
   b2FrictionJoint.prototype = Object.create(b2Joint.prototype);
   b2FrictionJoint.prototype.m_linearMass        = null;
   b2FrictionJoint.prototype.m_linearImpulse     = null;
   b2FrictionJoint.prototype.m_angularMass       = 0.0;
   b2FrictionJoint.prototype.m_angularImpulse    = 0.0;
   b2FrictionJoint.prototype.m_maxForce          = 0;
   b2FrictionJoint.prototype.m_maxTorque         = 0;


   /**
    * GetAnchorA
    *
    * @param 
    *
    */
   b2FrictionJoint.prototype.GetAnchorA = function () {
      return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
   };

   /**
    * GetAnchorB
    *
    * @param 
    *
    */
   b2FrictionJoint.prototype.GetAnchorB = function () {
      return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
   };

   /**
    * GetReactionForce
    *
    * @param inv_dt
    *
    */
   b2FrictionJoint.prototype.GetReactionForce = function (inv_dt) {
      inv_dt = inv_dt || 0;
      return new b2Vec2(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
   };

   /**
    * GetReactionTorque
    *
    * @param inv_dt
    *
    */
   b2FrictionJoint.prototype.GetReactionTorque = function (inv_dt) {
      inv_dt = inv_dt || 0;
      return inv_dt * this.m_angularImpulse;
   };

   /**
    * SetMaxForce
    *
    * @param force
    *
    */
   b2FrictionJoint.prototype.SetMaxForce = function (force) {
      force = force || 0;
      this.m_maxForce = force;
   };

   /**
    * GetMaxForce
    *
    * @param 
    *
    */
   b2FrictionJoint.prototype.GetMaxForce = function () {
      return this.m_maxForce;
   };

   /**
    * SetMaxTorque
    *
    * @param torque
    *
    */
   b2FrictionJoint.prototype.SetMaxTorque = function (torque) {
      torque = torque || 0;
      this.m_maxTorque = torque;
   };

   /**
    * GetMaxTorque
    *
    * @param 
    *
    */
   b2FrictionJoint.prototype.GetMaxTorque = function () {
      return this.m_maxTorque;
   };

   /**
    * InitVelocityConstraints
    *
    * @param step
    *
    */
   b2FrictionJoint.prototype.InitVelocityConstraints = function (step) {
      var tMat,
          tX = 0,
          bA = this.m_bodyA,
          bB = this.m_bodyB;
      tMat = bA.m_xf.R;
      var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x,
          rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
      tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
      rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
      rAX = tX;
      tMat = bB.m_xf.R;
      var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x,
          rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
      tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
      rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
      rBX = tX;
      var mA = bA.m_invMass,
          mB = bB.m_invMass,
          iA = bA.m_invI,
          iB = bB.m_invI,
          K = new b2Mat22();
      K.col1.x = mA + mB;
      K.col2.x = 0.0;
      K.col1.y = 0.0;
      K.col2.y = mA + mB;
      K.col1.x += iA * rAY * rAY;
      K.col2.x += (-iA * rAX * rAY);
      K.col1.y += (-iA * rAX * rAY);
      K.col2.y += iA * rAX * rAX;
      K.col1.x += iB * rBY * rBY;
      K.col2.x += (-iB * rBX * rBY);
      K.col1.y += (-iB * rBX * rBY);
      K.col2.y += iB * rBX * rBX;
      K.GetInverse(this.m_linearMass);
      this.m_angularMass = iA + iB;
      if (this.m_angularMass > 0.0) {
         this.m_angularMass = 1.0 / this.m_angularMass;
      }
      if (step.warmStarting) {
         this.m_linearImpulse.x *= step.dtRatio;
         this.m_linearImpulse.y *= step.dtRatio;
         this.m_angularImpulse *= step.dtRatio;
         var P = this.m_linearImpulse;
         bA.m_linearVelocity.x -= mA * P.x;
         bA.m_linearVelocity.y -= mA * P.y;
         bA.m_angularVelocity -= iA * (rAX * P.y - rAY * P.x + this.m_angularImpulse);
         bB.m_linearVelocity.x += mB * P.x;
         bB.m_linearVelocity.y += mB * P.y;
         bB.m_angularVelocity += iB * (rBX * P.y - rBY * P.x + this.m_angularImpulse);
      }
      else {
         this.m_linearImpulse.SetZero();
         this.m_angularImpulse = 0.0;
      }
   };

   /**
    * SolveVelocityConstraints
    *
    * @param step
    *
    */
   b2FrictionJoint.prototype.SolveVelocityConstraints = function (step) {
      var tMat,
          tX = 0,
          bA = this.m_bodyA,
          bB = this.m_bodyB,
          vA = bA.m_linearVelocity,
          wA = bA.m_angularVelocity,
          vB = bB.m_linearVelocity,
          wB = bB.m_angularVelocity,
          mA = bA.m_invMass,
          mB = bB.m_invMass,
          iA = bA.m_invI,
          iB = bB.m_invI;
      tMat = bA.m_xf.R;
      var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x,
          rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
      tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
      rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
      rAX = tX;
      tMat = bB.m_xf.R;
      var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x,
          rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
      tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
      rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
      rBX = tX;
      var maxImpulse = 0; {
         var Cdot = wB - wA,
          impulse = (-this.m_angularMass * Cdot),
          oldImpulse = this.m_angularImpulse;
         maxImpulse = step.dt * this.m_maxTorque;
         this.m_angularImpulse = b2Math.Clamp(this.m_angularImpulse + impulse, (-maxImpulse), maxImpulse);
         impulse = this.m_angularImpulse - oldImpulse;
         wA -= iA * impulse;
         wB += iB * impulse;
      } {
         var CdotX = vB.x - wB * rBY - vA.x + wA * rAY,
          CdotY = vB.y + wB * rBX - vA.y - wA * rAX,
          impulseV = b2Math.MulMV(this.m_linearMass, new b2Vec2((-CdotX), (-CdotY))),
          oldImpulseV = this.m_linearImpulse.Copy();
         this.m_linearImpulse.Add(impulseV);
         maxImpulse = step.dt * this.m_maxForce;
         if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
            this.m_linearImpulse.Normalize();
            this.m_linearImpulse.Multiply(maxImpulse);
         }
         impulseV = b2Math.SubtractVV(this.m_linearImpulse, oldImpulseV);
         vA.x -= mA * impulseV.x;
         vA.y -= mA * impulseV.y;
         wA -= iA * (rAX * impulseV.y - rAY * impulseV.x);
         vB.x += mB * impulseV.x;
         vB.y += mB * impulseV.y;
         wB += iB * (rBX * impulseV.y - rBY * impulseV.x);
      }
      bA.m_angularVelocity = wA;
      bB.m_angularVelocity = wB;
   };

   /**
    * SolvePositionConstraints
    *
    * @param baumgarte
    *
    */
   b2FrictionJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      baumgarte = baumgarte || 0;
      return true;
   };

   /**
    * GetType
    *
    * @param 
    *
    */
   b2FrictionJoint.prototype.GetType = function () {
      return this.m_type;
   };

   /**
    * GetBodyA
    *
    * @param 
    *
    */
   b2FrictionJoint.prototype.GetBodyA = function () {
      return this.m_bodyA;
   };

   /**
    * GetBodyB
    *
    * @param 
    *
    */
   b2FrictionJoint.prototype.GetBodyB = function () {
      return this.m_bodyB;
   };

   /**
    * GetNext
    *
    * @param 
    *
    */
   b2FrictionJoint.prototype.GetNext = function () {
      return this.m_next;
   };

   /**
    * GetUserData
    *
    * @param 
    *
    */
   b2FrictionJoint.prototype.GetUserData = function () {
      return this.m_userData;
   };

   /**
    * SetUserData
    *
    * @param data
    *
    */
   b2FrictionJoint.prototype.SetUserData = function (data) {
      this.m_userData = data;
   };

   /**
    * IsActive
    *
    * @param 
    *
    */
   b2FrictionJoint.prototype.IsActive = function () {
      return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
   };

   /**
    * b2Joint
    *
    * @param def
    *
    */
   b2FrictionJoint.prototype.b2Joint = function (def) {
      b2Assert(def.bodyA !== def.bodyB);
      this.m_type = def.type;
      this.m_prev = null;
      this.m_next = null;
      this.m_bodyA = def.bodyA;
      this.m_bodyB = def.bodyB;
      this.m_collideConnected = def.collideConnected;
      this.m_islandFlag = false;
      this.m_userData = def.userData;
   };

   /**
    * FinalizeVelocityConstraints
    *
    * @param 
    *
    */
   b2FrictionJoint.prototype.FinalizeVelocityConstraints = function () {};