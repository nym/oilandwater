
   /**
    *  Class b2DistanceJoint
    *
    * @param def
    *
    */
   b2DistanceJoint = Box2D.Dynamics.Joints.b2DistanceJoint = function b2DistanceJoint(def) {

      this.m_localAnchor1 = new b2Vec2(0, 0);
      this.m_localAnchor2 = new b2Vec2(0, 0);
      this.m_u = new b2Vec2(0, 0);
      b2Joint.call(this, def);
      this.m_localAnchor1.SetV(def.localAnchorA);
      this.m_localAnchor2.SetV(def.localAnchorB);
      this.m_length = def.length;
      this.m_frequencyHz = def.frequencyHz;
      this.m_dampingRatio = def.dampingRatio;
   };
   b2DistanceJoint.constructor = b2DistanceJoint;
   b2DistanceJoint.prototype = Object.create(b2Joint.prototype );
   b2DistanceJoint.prototype.m_u                     = null;
   b2DistanceJoint.prototype.m_length                = 0;
   b2DistanceJoint.prototype.m_frequencyHz           = 0;
   b2DistanceJoint.prototype.m_dampingRatio          = 0;
   b2DistanceJoint.prototype.m_impulse               = 0.0;
   b2DistanceJoint.prototype.m_gamma                 = 0.0;
   b2DistanceJoint.prototype.m_bias                  = 0.0;

   /**
    * GetAnchorA
    *
    * @param 
    *
    */
   b2DistanceJoint.prototype.GetAnchorA = function () {
      return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
   };

   /**
    * GetAnchorB
    *
    * @param 
    *
    */
   b2DistanceJoint.prototype.GetAnchorB = function () {
      return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
   };

   /**
    * GetReactionForce
    *
    * @param inv_dt
    *
    */
   b2DistanceJoint.prototype.GetReactionForce = function (inv_dt) {
      inv_dt = inv_dt || 0;
      return new b2Vec2(inv_dt * this.m_impulse * this.m_u.x, inv_dt * this.m_impulse * this.m_u.y);
   };

   /**
    * GetReactionTorque
    *
    * @param inv_dt
    *
    */
   b2DistanceJoint.prototype.GetReactionTorque = function (inv_dt) {
      inv_dt = inv_dt || 0;
      return 0.0;
   };

   /**
    * GetLength
    *
    * @param 
    *
    */
   b2DistanceJoint.prototype.GetLength = function () {
      return this.m_length;
   };

   /**
    * SetLength
    *
    * @param length
    *
    */
   b2DistanceJoint.prototype.SetLength = function (length) {
      length = length || 0;
      this.m_length = length;
   };

   /**
    * GetFrequency
    *
    * @param 
    *
    */
   b2DistanceJoint.prototype.GetFrequency = function () {
      return this.m_frequencyHz;
   };

   /**
    * SetFrequency
    *
    * @param hz
    *
    */
   b2DistanceJoint.prototype.SetFrequency = function (hz) {
      hz = hz || 0;
      this.m_frequencyHz = hz;
   };

   /**
    * GetDampingRatio
    *
    * @param 
    *
    */
   b2DistanceJoint.prototype.GetDampingRatio = function () {
      return this.m_dampingRatio;
   };

   /**
    * SetDampingRatio
    *
    * @param ratio
    *
    */
   b2DistanceJoint.prototype.SetDampingRatio = function (ratio) {
      ratio = ratio || 0;
      this.m_dampingRatio = ratio;
   };

   /**
    * InitVelocityConstraints
    *
    * @param step
    *
    */
   b2DistanceJoint.prototype.InitVelocityConstraints = function (step) {
      var tMat,
          tX = 0,
          bA = this.m_bodyA,
          bB = this.m_bodyB;
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x,
          r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x,
          r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      this.m_u.x = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X;
      this.m_u.y = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
      var length = Math.sqrt(this.m_u.x * this.m_u.x + this.m_u.y * this.m_u.y);
      if (length > b2Settings.b2_linearSlop) {
         this.m_u.Multiply(1.0 / length);
      }
      else {
         this.m_u.SetZero();
      }
      var cr1u = (r1X * this.m_u.y - r1Y * this.m_u.x),
          cr2u = (r2X * this.m_u.y - r2Y * this.m_u.x),
          invMass = bA.m_invMass + bA.m_invI * cr1u * cr1u + bB.m_invMass + bB.m_invI * cr2u * cr2u;
      this.m_mass = invMass !== 0.0 ? 1.0 / invMass : 0.0;
      if (this.m_frequencyHz > 0.0) {
         var C = length - this.m_length,
          omega = 2.0 * Math.PI * this.m_frequencyHz,
          d = 2.0 * this.m_mass * this.m_dampingRatio * omega,
          k = this.m_mass * omega * omega;
         this.m_gamma = step.dt * (d + step.dt * k);
         this.m_gamma = this.m_gamma !== 0.0 ? 1 / this.m_gamma : 0.0;
         this.m_bias = C * step.dt * k * this.m_gamma;
         this.m_mass = invMass + this.m_gamma;
         this.m_mass = this.m_mass !== 0.0 ? 1.0 / this.m_mass : 0.0;
      }
      if (step.warmStarting) {
         this.m_impulse *= step.dtRatio;
         var PX = this.m_impulse * this.m_u.x,
          PY = this.m_impulse * this.m_u.y;
         bA.m_linearVelocity.x -= bA.m_invMass * PX;
         bA.m_linearVelocity.y -= bA.m_invMass * PY;
         bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
         bB.m_linearVelocity.x += bB.m_invMass * PX;
         bB.m_linearVelocity.y += bB.m_invMass * PY;
         bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
      }
      else {
         this.m_impulse = 0.0;
      }
   };

   /**
    * SolveVelocityConstraints
    *
    * @param step
    *
    */
   b2DistanceJoint.prototype.SolveVelocityConstraints = function (step) {
      var tMat,
          bA = this.m_bodyA,
          bB = this.m_bodyB;
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x,
          r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y,
          tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x,
          r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var v1X = bA.m_linearVelocity.x + ((-bA.m_angularVelocity * r1Y)),
          v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X),
          v2X = bB.m_linearVelocity.x + ((-bB.m_angularVelocity * r2Y)),
          v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X),
          Cdot = (this.m_u.x * (v2X - v1X) + this.m_u.y * (v2Y - v1Y)),
          impulse = (-this.m_mass * (Cdot + this.m_bias + this.m_gamma * this.m_impulse));
      this.m_impulse += impulse;
      var PX = impulse * this.m_u.x,
          PY = impulse * this.m_u.y;
      bA.m_linearVelocity.x -= bA.m_invMass * PX;
      bA.m_linearVelocity.y -= bA.m_invMass * PY;
      bA.m_angularVelocity -= bA.m_invI * (r1X * PY - r1Y * PX);
      bB.m_linearVelocity.x += bB.m_invMass * PX;
      bB.m_linearVelocity.y += bB.m_invMass * PY;
      bB.m_angularVelocity += bB.m_invI * (r2X * PY - r2Y * PX);
   };

   /**
    * SolvePositionConstraints
    *
    * @param baumgarte
    *
    */
   b2DistanceJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      baumgarte = baumgarte || 0;
      var tMat;
      if (this.m_frequencyHz > 0.0) {
         return true;
      }
      var bA = this.m_bodyA,
          bB = this.m_bodyB;
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x,
          r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y,
          tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x,
          r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X,
          dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y,
          length = Math.sqrt(dX * dX + dY * dY);
      dX /= length;
      dY /= length;
      var C = length - this.m_length;
      C = b2Math.Clamp(C, (-b2Settings.b2_maxLinearCorrection), b2Settings.b2_maxLinearCorrection);
      var impulse = (-this.m_mass * C);
      this.m_u.Set(dX, dY);
      var PX = impulse * this.m_u.x,
          PY = impulse * this.m_u.y;
      bA.m_sweep.c.x -= bA.m_invMass * PX;
      bA.m_sweep.c.y -= bA.m_invMass * PY;
      bA.m_sweep.a -= bA.m_invI * (r1X * PY - r1Y * PX);
      bB.m_sweep.c.x += bB.m_invMass * PX;
      bB.m_sweep.c.y += bB.m_invMass * PY;
      bB.m_sweep.a += bB.m_invI * (r2X * PY - r2Y * PX);
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
      return b2Math.Abs(C) < b2Settings.b2_linearSlop;
   };

   /**
    * GetType
    *
    * @param 
    *
    */
   b2DistanceJoint.prototype.GetType = function () {
      return this.m_type;
   };

   /**
    * GetBodyA
    *
    * @param 
    *
    */
   b2DistanceJoint.prototype.GetBodyA = function () {
      return this.m_bodyA;
   };

   /**
    * GetBodyB
    *
    * @param 
    *
    */
   b2DistanceJoint.prototype.GetBodyB = function () {
      return this.m_bodyB;
   };

   /**
    * GetNext
    *
    * @param 
    *
    */
   b2DistanceJoint.prototype.GetNext = function () {
      return this.m_next;
   };

   /**
    * GetUserData
    *
    * @param 
    *
    */
   b2DistanceJoint.prototype.GetUserData = function () {
      return this.m_userData;
   };

   /**
    * SetUserData
    *
    * @param data
    *
    */
   b2DistanceJoint.prototype.SetUserData = function (data) {
      this.m_userData = data;
   };

   /**
    * IsActive
    *
    * @param 
    *
    */
   b2DistanceJoint.prototype.IsActive = function () {
      return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
   };

   /**
    * b2Joint
    *
    * @param def
    *
    */
   b2DistanceJoint.prototype.b2Joint = function (def) {
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
   b2DistanceJoint.prototype.FinalizeVelocityConstraints = function () {};