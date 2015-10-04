
   /**
    *  Class b2MouseJoint
    *
    * @param def
    *
    */
   b2MouseJoint = Box2D.Dynamics.Joints.b2MouseJoint = function b2MouseJoint(def) {

      this.K = new b2Mat22();
      this.K1 = new b2Mat22();
      this.K2 = new b2Mat22();
      this.m_localAnchor = new b2Vec2(0, 0);
      this.m_target = new b2Vec2(0, 0);
      this.m_impulse = new b2Vec2(0, 0);
      this.m_mass = new b2Mat22();
      this.m_C = new b2Vec2(0, 0);
      b2Joint.call(this, def);
      this.m_target.SetV(def.target);
      var tX = this.m_target.x - this.m_bodyB.m_xf.position.x,
          tY = this.m_target.y - this.m_bodyB.m_xf.position.y,
          tMat = this.m_bodyB.m_xf.R;
      this.m_localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y);
      this.m_localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y);
      this.m_maxForce = def.maxForce;
      this.m_impulse.SetZero();
      this.m_frequencyHz = def.frequencyHz;
      this.m_dampingRatio = def.dampingRatio;
   };
   b2MouseJoint.constructor = b2MouseJoint;
   b2MouseJoint.prototype = Object.create(b2Joint.prototype );
   b2MouseJoint.prototype.K               = null;
   b2MouseJoint.prototype.K1              = null;
   b2MouseJoint.prototype.K2              = null;
   b2MouseJoint.prototype.m_localAnchor   = null;
   b2MouseJoint.prototype.m_target        = null;
   b2MouseJoint.prototype.m_impulse       = null;
   b2MouseJoint.prototype.m_mass          = null;
   b2MouseJoint.prototype.m_C             = null;
   b2MouseJoint.prototype.m_frequencyHz   = 0.0;
   b2MouseJoint.prototype.m_dampingRatio  = 0.0;
   b2MouseJoint.prototype.m_beta          = 0.0;
   b2MouseJoint.prototype.m_gamma         = 0.0;

   /**
    * GetAnchorA
    *
    * @param 
    *
    */
   b2MouseJoint.prototype.GetAnchorA = function () {
      return this.m_target;
   };

   /**
    * GetAnchorB
    *
    * @param 
    *
    */
   b2MouseJoint.prototype.GetAnchorB = function () {
      return this.m_bodyB.GetWorldPoint(this.m_localAnchor);
   };

   /**
    * GetReactionForce
    *
    * @param inv_dt
    *
    */
   b2MouseJoint.prototype.GetReactionForce = function (inv_dt) {
      inv_dt = inv_dt || 0;
      return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
   };

   /**
    * GetReactionTorque
    *
    * @param inv_dt
    *
    */
   b2MouseJoint.prototype.GetReactionTorque = function (inv_dt) {
      inv_dt = inv_dt || 0;
      return 0.0;
   };

   /**
    * GetTarget
    *
    * @param 
    *
    */
   b2MouseJoint.prototype.GetTarget = function () {
      return this.m_target;
   };

   /**
    * SetTarget
    *
    * @param target
    *
    */
   b2MouseJoint.prototype.SetTarget = function (target) {
      if (this.m_bodyB.IsAwake() === false) {
         this.m_bodyB.SetAwake(true);
      }
      this.m_target = target;
   };

   /**
    * GetMaxForce
    *
    * @param 
    *
    */
   b2MouseJoint.prototype.GetMaxForce = function () {
      return this.m_maxForce;
   };

   /**
    * SetMaxForce
    *
    * @param maxForce
    *
    */
   b2MouseJoint.prototype.SetMaxForce = function (maxForce) {
      maxForce = maxForce || 0;
      this.m_maxForce = maxForce;
   };

   /**
    * GetFrequency
    *
    * @param 
    *
    */
   b2MouseJoint.prototype.GetFrequency = function () {
      return this.m_frequencyHz;
   };

   /**
    * SetFrequency
    *
    * @param hz
    *
    */
   b2MouseJoint.prototype.SetFrequency = function (hz) {
      hz = hz || 0;
      this.m_frequencyHz = hz;
   };

   /**
    * GetDampingRatio
    *
    * @param 
    *
    */
   b2MouseJoint.prototype.GetDampingRatio = function () {
      return this.m_dampingRatio;
   };

   /**
    * SetDampingRatio
    *
    * @param ratio
    *
    */
   b2MouseJoint.prototype.SetDampingRatio = function (ratio) {
      ratio = ratio || 0;
      this.m_dampingRatio = ratio;
   };

   /**
    * InitVelocityConstraints
    *
    * @param step
    *
    */
   b2MouseJoint.prototype.InitVelocityConstraints = function (step) {
      var b = this.m_bodyB,
          mass = b.GetMass(),
          omega = 2.0 * Math.PI * this.m_frequencyHz,
          d = 2.0 * mass * this.m_dampingRatio * omega,
          k = mass * omega * omega,
          rX,
          rY,
          tX;
      this.m_gamma = step.dt * (d + step.dt * k);
      this.m_gamma = this.m_gamma !== 0 ? 1 / this.m_gamma : 0.0;
      this.m_beta = step.dt * k * this.m_gamma;
      var tMat;tMat = b.m_xf.R,
          rX = this.m_localAnchor.x - b.m_sweep.localCenter.x,
          rY = this.m_localAnchor.y - b.m_sweep.localCenter.y,
          tX = (tMat.col1.x * rX + tMat.col2.x * rY);rY = (tMat.col1.y * rX + tMat.col2.y * rY);
      rX = tX;
      var invMass = b.m_invMass,
          invI = b.m_invI;this.K1.col1.x = invMass;
      this.K1.col2.x = 0.0;
      this.K1.col1.y = 0.0;
      this.K1.col2.y = invMass;
      this.K2.col1.x = invI * rY * rY;
      this.K2.col2.x = (-invI * rX * rY);
      this.K2.col1.y = (-invI * rX * rY);
      this.K2.col2.y = invI * rX * rX;
      this.K.SetM(this.K1);
      this.K.AddM(this.K2);
      this.K.col1.x += this.m_gamma;
      this.K.col2.y += this.m_gamma;
      this.K.GetInverse(this.m_mass);
      this.m_C.x = b.m_sweep.c.x + rX - this.m_target.x;
      this.m_C.y = b.m_sweep.c.y + rY - this.m_target.y;
      b.m_angularVelocity *= 0.98;
      this.m_impulse.x *= step.dtRatio;
      this.m_impulse.y *= step.dtRatio;
      b.m_linearVelocity.x += invMass * this.m_impulse.x;
      b.m_linearVelocity.y += invMass * this.m_impulse.y;
      b.m_angularVelocity += invI * (rX * this.m_impulse.y - rY * this.m_impulse.x);
   };

   /**
    * SolveVelocityConstraints
    *
    * @param step
    *
    */
   b2MouseJoint.prototype.SolveVelocityConstraints = function (step) {
      var b = this.m_bodyB,
          tMat,
          tX = 0,
          tY = 0;
      tMat = b.m_xf.R;
      var rX = this.m_localAnchor.x - b.m_sweep.localCenter.x,
          rY = this.m_localAnchor.y - b.m_sweep.localCenter.y;
      tX = (tMat.col1.x * rX + tMat.col2.x * rY);
      rY = (tMat.col1.y * rX + tMat.col2.y * rY);
      rX = tX;
      var CdotX = b.m_linearVelocity.x + ((-b.m_angularVelocity * rY)),
          CdotY = b.m_linearVelocity.y + (b.m_angularVelocity * rX);
      tMat = this.m_mass;
      tX = CdotX + this.m_beta * this.m_C.x + this.m_gamma * this.m_impulse.x;
      tY = CdotY + this.m_beta * this.m_C.y + this.m_gamma * this.m_impulse.y;
      var impulseX = (-(tMat.col1.x * tX + tMat.col2.x * tY)),
          impulseY = (-(tMat.col1.y * tX + tMat.col2.y * tY)),
          oldImpulseX = this.m_impulse.x,
          oldImpulseY = this.m_impulse.y;
      this.m_impulse.x += impulseX;
      this.m_impulse.y += impulseY;
      var maxImpulse = step.dt * this.m_maxForce;
      if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
         this.m_impulse.Multiply(maxImpulse / this.m_impulse.Length());
      }
      impulseX = this.m_impulse.x - oldImpulseX;
      impulseY = this.m_impulse.y - oldImpulseY;
      b.m_linearVelocity.x += b.m_invMass * impulseX;
      b.m_linearVelocity.y += b.m_invMass * impulseY;
      b.m_angularVelocity += b.m_invI * (rX * impulseY - rY * impulseX);
   };

   /**
    * SolvePositionConstraints
    *
    * @param baumgarte
    *
    */
   b2MouseJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      baumgarte = baumgarte || 0;
      return true;
   };

   /**
    * GetType
    *
    * @param 
    *
    */
   b2MouseJoint.prototype.GetType = function () {
      return this.m_type;
   };

   /**
    * GetBodyA
    *
    * @param 
    *
    */
   b2MouseJoint.prototype.GetBodyA = function () {
      return this.m_bodyA;
   };

   /**
    * GetBodyB
    *
    * @param 
    *
    */
   b2MouseJoint.prototype.GetBodyB = function () {
      return this.m_bodyB;
   };

   /**
    * GetNext
    *
    * @param 
    *
    */
   b2MouseJoint.prototype.GetNext = function () {
      return this.m_next;
   };

   /**
    * GetUserData
    *
    * @param 
    *
    */
   b2MouseJoint.prototype.GetUserData = function () {
      return this.m_userData;
   };

   /**
    * SetUserData
    *
    * @param data
    *
    */
   b2MouseJoint.prototype.SetUserData = function (data) {
      this.m_userData = data;
   };

   /**
    * IsActive
    *
    * @param 
    *
    */
   b2MouseJoint.prototype.IsActive = function () {
      return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
   };

   /**
    * b2Joint
    *
    * @param def
    *
    */
   b2MouseJoint.prototype.b2Joint = function (def) {
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
   b2MouseJoint.prototype.FinalizeVelocityConstraints = function () {};