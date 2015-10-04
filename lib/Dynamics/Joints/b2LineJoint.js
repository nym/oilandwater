
   /**
    *  Class b2LineJoint
    *
    * @param def
    *
    */
   b2LineJoint = Box2D.Dynamics.Joints.b2LineJoint = function b2LineJoint(def) {

      this.m_localAnchor1 = new b2Vec2(0, 0);
      this.m_localAnchor2 = new b2Vec2(0, 0);
      this.m_localXAxis1 = new b2Vec2(0, 0);
      this.m_localYAxis1 = new b2Vec2(0, 0);
      this.m_axis = new b2Vec2(0, 0);
      this.m_perp = new b2Vec2(0, 0);
      this.m_K = new b2Mat22();
      this.m_impulse = new b2Vec2(0, 0);
      b2Joint.call(this, def);
      this.m_localAnchor1.SetV(def.localAnchorA);
      this.m_localAnchor2.SetV(def.localAnchorB);
      this.m_localXAxis1.SetV(def.localAxisA);
      this.m_localYAxis1.x = (-this.m_localXAxis1.y);
      this.m_localYAxis1.y = this.m_localXAxis1.x;
      this.m_impulse.SetZero();
      this.m_lowerTranslation = def.lowerTranslation;
      this.m_upperTranslation = def.upperTranslation;
      this.m_maxMotorForce = def.maxMotorForce;
      this.m_motorSpeed = def.motorSpeed;
      this.m_enableLimit = def.enableLimit;
      this.m_enableMotor = def.enableMotor;
      this.m_limitState = b2Joint.e_inactiveLimit;
      this.m_axis.SetZero();
      this.m_perp.SetZero();
   };
   b2LineJoint.constructor = b2LineJoint;
   b2LineJoint.prototype = Object.create(b2Joint.prototype );
   b2LineJoint.prototype.m_localXAxis1             = null;
   b2LineJoint.prototype.m_localYAxis1             = null;
   b2LineJoint.prototype.m_axis                    = null;
   b2LineJoint.prototype.m_perp                    = null;
   b2LineJoint.prototype.m_K                       = null;
   b2LineJoint.prototype.m_impulse                 = null;
   b2LineJoint.prototype.m_motorMass               = 0.0;
   b2LineJoint.prototype.m_motorImpulse            = 0.0;
   b2LineJoint.prototype.m_lowerTranslation        = 0.0;
   b2LineJoint.prototype.m_upperTranslation        = 0.0;
   b2LineJoint.prototype.m_maxMotorForce           = 0.0;
   b2LineJoint.prototype.m_motorSpeed              = 0.0;
   b2LineJoint.prototype.m_enableLimit             = false;
   b2LineJoint.prototype.m_enableMotor             = false;
   b2LineJoint.prototype.m_limitState              = b2Joint.e_inactiveLimit;

   /**
    * GetAnchorA
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetAnchorA = function () {
      return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
   };

   /**
    * GetAnchorB
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetAnchorB = function () {
      return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
   };

   /**
    * GetReactionForce
    *
    * @param inv_dt
    *
    */
   b2LineJoint.prototype.GetReactionForce = function (inv_dt) {
      inv_dt = inv_dt || 0;
      return new b2Vec2(inv_dt * (this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.x), inv_dt * (this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.y));
   };

   /**
    * GetReactionTorque
    *
    * @param inv_dt
    *
    */
   b2LineJoint.prototype.GetReactionTorque = function (inv_dt) {
      return inv_dt * this.m_impulse.y;
   };

   /**
    * GetJointTranslation
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetJointTranslation = function () {
      var bA = this.m_bodyA,
          bB = this.m_bodyB,
          p1 = bA.GetWorldPoint(this.m_localAnchor1),
          p2 = bB.GetWorldPoint(this.m_localAnchor2),
          dX = p2.x - p1.x,
          dY = p2.y - p1.y,
          axis = bA.GetWorldVector(this.m_localXAxis1);
      return axis.x * dX + axis.y * dY;
   };

   /**
    * GetJointSpeed
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetJointSpeed = function () {
      var bA = this.m_bodyA,
          bB = this.m_bodyB,
          tMat;
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
      var p1X = bA.m_sweep.c.x + r1X,
          p1Y = bA.m_sweep.c.y + r1Y,
          p2X = bB.m_sweep.c.x + r2X,
          p2Y = bB.m_sweep.c.y + r2Y,
          dX = p2X - p1X,
          dY = p2Y - p1Y,
          axis = bA.GetWorldVector(this.m_localXAxis1),
          v1 = bA.m_linearVelocity,
          v2 = bB.m_linearVelocity,
          w1 = bA.m_angularVelocity,
          w2 = bB.m_angularVelocity;
      return (dX * ((-w1 * axis.y)) + dY * (w1 * axis.x)) + (axis.x * (((v2.x + ((-w2 * r2Y))) - v1.x) - ((-w1 * r1Y))) + axis.y * (((v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)));
   };

   /**
    * IsLimitEnabled
    *
    * @param 
    *
    */
   b2LineJoint.prototype.IsLimitEnabled = function () {
      return this.m_enableLimit;
   };

   /**
    * EnableLimit
    *
    * @param flag
    *
    */
   b2LineJoint.prototype.EnableLimit = function (flag) {
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_enableLimit = flag;
   };

   /**
    * GetLowerLimit
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetLowerLimit = function () {
      return this.m_lowerTranslation;
   };

   /**
    * GetUpperLimit
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetUpperLimit = function () {
      return this.m_upperTranslation;
   };

   /**
    * SetLimits
    *
    * @param lower
    * @param upper
    *
    */
   b2LineJoint.prototype.SetLimits = function (lower, upper) {
      lower = lower || 0;
      upper = upper || 0;
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_lowerTranslation = lower;
      this.m_upperTranslation = upper;
   };

   /**
    * IsMotorEnabled
    *
    * @param 
    *
    */
   b2LineJoint.prototype.IsMotorEnabled = function () {
      return this.m_enableMotor;
   };

   /**
    * EnableMotor
    *
    * @param flag
    *
    */
   b2LineJoint.prototype.EnableMotor = function (flag) {
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_enableMotor = flag;
   };

   /**
    * SetMotorSpeed
    *
    * @param speed
    *
    */
   b2LineJoint.prototype.SetMotorSpeed = function (speed) {
      speed = speed || 0;
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_motorSpeed = speed;
   };

   /**
    * GetMotorSpeed
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetMotorSpeed = function () {
      return this.m_motorSpeed;
   };

   /**
    * SetMaxMotorForce
    *
    * @param force
    *
    */
   b2LineJoint.prototype.SetMaxMotorForce = function (force) {
      force = force || 0;
      this.m_bodyA.SetAwake(true);
      this.m_bodyB.SetAwake(true);
      this.m_maxMotorForce = force;
   };

   /**
    * GetMaxMotorForce
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetMaxMotorForce = function () {
      return this.m_maxMotorForce;
   };

   /**
    * GetMotorForce
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetMotorForce = function () {
      return this.m_motorImpulse;
   };

   /**
    * InitVelocityConstraints
    *
    * @param step
    *
    */
   b2LineJoint.prototype.InitVelocityConstraints = function (step) {
      var bA = this.m_bodyA,
          bB = this.m_bodyB,
          tMat,
          tX = 0;
      this.m_localCenterA.SetV(bA.GetLocalCenter());
      this.m_localCenterB.SetV(bB.GetLocalCenter());
      var xf1 = bA.GetTransform(),
          xf2 = bB.GetTransform();
      tMat = bA.m_xf.R;
      var r1X = this.m_localAnchor1.x - this.m_localCenterA.x,
          r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = bB.m_xf.R;
      var r2X = this.m_localAnchor2.x - this.m_localCenterB.x,
          r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var dX = bB.m_sweep.c.x + r2X - bA.m_sweep.c.x - r1X,
          dY = bB.m_sweep.c.y + r2Y - bA.m_sweep.c.y - r1Y;
      this.m_invMassA = bA.m_invMass;
      this.m_invMassB = bB.m_invMass;
      this.m_invIA = bA.m_invI;
      this.m_invIB = bB.m_invI; {
         this.m_axis.SetV(b2Math.MulMV(xf1.R, this.m_localXAxis1));
         this.m_a1 = (dX + r1X) * this.m_axis.y - (dY + r1Y) * this.m_axis.x;
         this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
         this.m_motorMass = this.m_invMassA + this.m_invMassB + this.m_invIA * this.m_a1 * this.m_a1 + this.m_invIB * this.m_a2 * this.m_a2;
         this.m_motorMass = this.m_motorMass > b2Settings.b2_epsilon ? 1.0 / this.m_motorMass : 0.0;
      } {
         this.m_perp.SetV(b2Math.MulMV(xf1.R, this.m_localYAxis1));
         this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
         this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
         var m1 = this.m_invMassA,
          m2 = this.m_invMassB,
          i1 = this.m_invIA,
          i2 = this.m_invIB;
         this.m_K.col1.x = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
         this.m_K.col1.y = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
         this.m_K.col2.x = this.m_K.col1.y;
         this.m_K.col2.y = m1 + m2 + i1 * this.m_a1 * this.m_a1 + i2 * this.m_a2 * this.m_a2;
      }
      if (this.m_enableLimit) {
         var jointTransition = this.m_axis.x * dX + this.m_axis.y * dY;
         if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
            this.m_limitState = b2Joint.e_equalLimits;
         }
         else if (jointTransition <= this.m_lowerTranslation) {
            if (this.m_limitState !== b2Joint.e_atLowerLimit) {
               this.m_limitState = b2Joint.e_atLowerLimit;
               this.m_impulse.y = 0.0;
            }
         }
         else if (jointTransition >= this.m_upperTranslation) {
            if (this.m_limitState !== b2Joint.e_atUpperLimit) {
               this.m_limitState = b2Joint.e_atUpperLimit;
               this.m_impulse.y = 0.0;
            }
         }
         else {
            this.m_limitState = b2Joint.e_inactiveLimit;
            this.m_impulse.y = 0.0;
         }
      }
      else {
         this.m_limitState = b2Joint.e_inactiveLimit;
      }
      if (this.m_enableMotor === false) {
         this.m_motorImpulse = 0.0;
      }
      if (step.warmStarting) {
         this.m_impulse.x *= step.dtRatio;
         this.m_impulse.y *= step.dtRatio;
         this.m_motorImpulse *= step.dtRatio;
         var PX = this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.x,
          PY = this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.y,
          L1 = this.m_impulse.x * this.m_s1 + (this.m_motorImpulse + this.m_impulse.y) * this.m_a1,
          L2 = this.m_impulse.x * this.m_s2 + (this.m_motorImpulse + this.m_impulse.y) * this.m_a2;
         bA.m_linearVelocity.x -= this.m_invMassA * PX;
         bA.m_linearVelocity.y -= this.m_invMassA * PY;
         bA.m_angularVelocity -= this.m_invIA * L1;
         bB.m_linearVelocity.x += this.m_invMassB * PX;
         bB.m_linearVelocity.y += this.m_invMassB * PY;
         bB.m_angularVelocity += this.m_invIB * L2;
      }
      else {
         this.m_impulse.SetZero();
         this.m_motorImpulse = 0.0;
      }
   };

   /**
    * SolveVelocityConstraints
    *
    * @param step
    *
    */
   b2LineJoint.prototype.SolveVelocityConstraints = function (step) {
      var bA = this.m_bodyA,
          bB = this.m_bodyB,
          v1 = bA.m_linearVelocity,
          w1 = bA.m_angularVelocity,
          v2 = bB.m_linearVelocity,
          w2 = bB.m_angularVelocity,
          PX = 0,
          PY = 0,
          L1 = 0,
          L2 = 0;
      if (this.m_enableMotor && this.m_limitState !== b2Joint.e_equalLimits) {
         var Cdot = this.m_axis.x * (v2.x - v1.x) + this.m_axis.y * (v2.y - v1.y) + this.m_a2 * w2 - this.m_a1 * w1,
          impulse = this.m_motorMass * (this.m_motorSpeed - Cdot),
          oldImpulse = this.m_motorImpulse,
          maxImpulse = step.dt * this.m_maxMotorForce;
         this.m_motorImpulse = b2Math.Clamp(this.m_motorImpulse + impulse, (-maxImpulse), maxImpulse);
         impulse = this.m_motorImpulse - oldImpulse;
         PX = impulse * this.m_axis.x;
         PY = impulse * this.m_axis.y;
         L1 = impulse * this.m_a1;
         L2 = impulse * this.m_a2;
         v1.x -= this.m_invMassA * PX;
         v1.y -= this.m_invMassA * PY;
         w1 -= this.m_invIA * L1;
         v2.x += this.m_invMassB * PX;
         v2.y += this.m_invMassB * PY;
         w2 += this.m_invIB * L2;
      }
      var Cdot1 = this.m_perp.x * (v2.x - v1.x) + this.m_perp.y * (v2.y - v1.y) + this.m_s2 * w2 - this.m_s1 * w1;
      if (this.m_enableLimit && this.m_limitState !== b2Joint.e_inactiveLimit) {
         var Cdot2 = this.m_axis.x * (v2.x - v1.x) + this.m_axis.y * (v2.y - v1.y) + this.m_a2 * w2 - this.m_a1 * w1,
          f1 = this.m_impulse.Copy(),
          df = this.m_K.Solve(new b2Vec2(0, 0), (-Cdot1), (-Cdot2));
         this.m_impulse.Add(df);
         if (this.m_limitState === b2Joint.e_atLowerLimit) {
            this.m_impulse.y = b2Math.Max(this.m_impulse.y, 0.0);
         }
         else if (this.m_limitState === b2Joint.e_atUpperLimit) {
            this.m_impulse.y = b2Math.Min(this.m_impulse.y, 0.0);
         }
         var b = (-Cdot1) - (this.m_impulse.y - f1.y) * this.m_K.col2.x,
          f2r = 0;
         if (this.m_K.col1.x !== 0.0) {
            f2r = b / this.m_K.col1.x + f1.x;
         }
         else {
            f2r = f1.x;
         }
         this.m_impulse.x = f2r;
         df.x = this.m_impulse.x - f1.x;
         df.y = this.m_impulse.y - f1.y;
         PX = df.x * this.m_perp.x + df.y * this.m_axis.x;
         PY = df.x * this.m_perp.y + df.y * this.m_axis.y;
         L1 = df.x * this.m_s1 + df.y * this.m_a1;
         L2 = df.x * this.m_s2 + df.y * this.m_a2;
         v1.x -= this.m_invMassA * PX;
         v1.y -= this.m_invMassA * PY;
         w1 -= this.m_invIA * L1;
         v2.x += this.m_invMassB * PX;
         v2.y += this.m_invMassB * PY;
         w2 += this.m_invIB * L2;
      }
      else {
         var df2 = 0;
         if (this.m_K.col1.x !== 0.0) {
            df2 = ((-Cdot1)) / this.m_K.col1.x;
         }
         else {
            df2 = 0.0;
         }
         this.m_impulse.x += df2;
         PX = df2 * this.m_perp.x;
         PY = df2 * this.m_perp.y;
         L1 = df2 * this.m_s1;
         L2 = df2 * this.m_s2;
         v1.x -= this.m_invMassA * PX;
         v1.y -= this.m_invMassA * PY;
         w1 -= this.m_invIA * L1;
         v2.x += this.m_invMassB * PX;
         v2.y += this.m_invMassB * PY;
         w2 += this.m_invIB * L2;
      }
      bA.m_linearVelocity.SetV(v1);
      bA.m_angularVelocity = w1;
      bB.m_linearVelocity.SetV(v2);
      bB.m_angularVelocity = w2;
   };

   /**
    * SolvePositionConstraints
    *
    * @param baumgarte
    *
    */
   b2LineJoint.prototype.SolvePositionConstraints = function (baumgarte) {
      baumgarte = baumgarte || 0;
      var limitC = 0,
          oldLimitImpulse = 0,
          bA = this.m_bodyA,
          bB = this.m_bodyB,
          c1 = bA.m_sweep.c,
          a1 = bA.m_sweep.a,
          c2 = bB.m_sweep.c,
          a2 = bB.m_sweep.a,
          tMat,
          tX = 0,
          m1 = 0,
          m2 = 0,
          i1 = 0,
          i2 = 0,
          linearError = 0.0,
          angularError = 0.0,
          active = false,
          C2 = 0.0,
          R1 = b2Mat22.FromAngle(a1),
          R2 = b2Mat22.FromAngle(a2);
      tMat = R1;
      var r1X = this.m_localAnchor1.x - this.m_localCenterA.x,
          r1Y = this.m_localAnchor1.y - this.m_localCenterA.y;
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      tMat = R2;
      var r2X = this.m_localAnchor2.x - this.m_localCenterB.x,
          r2Y = this.m_localAnchor2.y - this.m_localCenterB.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      var dX = c2.x + r2X - c1.x - r1X,
          dY = c2.y + r2Y - c1.y - r1Y;
      if (this.m_enableLimit) {
         this.m_axis = b2Math.MulMV(R1, this.m_localXAxis1);
         this.m_a1 = (dX + r1X) * this.m_axis.y - (dY + r1Y) * this.m_axis.x;
         this.m_a2 = r2X * this.m_axis.y - r2Y * this.m_axis.x;
         var translation = this.m_axis.x * dX + this.m_axis.y * dY;
         if (b2Math.Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2Settings.b2_linearSlop) {
            C2 = b2Math.Clamp(translation, (-b2Settings.b2_maxLinearCorrection), b2Settings.b2_maxLinearCorrection);
            linearError = b2Math.Abs(translation);
            active = true;
         }
         else if (translation <= this.m_lowerTranslation) {
            C2 = b2Math.Clamp(translation - this.m_lowerTranslation + b2Settings.b2_linearSlop, (-b2Settings.b2_maxLinearCorrection), 0.0);
            linearError = this.m_lowerTranslation - translation;
            active = true;
         }
         else if (translation >= this.m_upperTranslation) {
            C2 = b2Math.Clamp(translation - this.m_upperTranslation + b2Settings.b2_linearSlop, 0.0, b2Settings.b2_maxLinearCorrection);
            linearError = translation - this.m_upperTranslation;
            active = true;
         }
      }
      this.m_perp = b2Math.MulMV(R1, this.m_localYAxis1);
      this.m_s1 = (dX + r1X) * this.m_perp.y - (dY + r1Y) * this.m_perp.x;
      this.m_s2 = r2X * this.m_perp.y - r2Y * this.m_perp.x;
      var impulse = new b2Vec2(0, 0),
          C1 = this.m_perp.x * dX + this.m_perp.y * dY;
      linearError = b2Math.Max(linearError, b2Math.Abs(C1));
      angularError = 0.0;
      if (active) {
         m1 = this.m_invMassA;
         m2 = this.m_invMassB;
         i1 = this.m_invIA;
         i2 = this.m_invIB;
         this.m_K.col1.x = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2;
         this.m_K.col1.y = i1 * this.m_s1 * this.m_a1 + i2 * this.m_s2 * this.m_a2;
         this.m_K.col2.x = this.m_K.col1.y;
         this.m_K.col2.y = m1 + m2 + i1 * this.m_a1 * this.m_a1 + i2 * this.m_a2 * this.m_a2;
         this.m_K.Solve(impulse, (-C1), (-C2));
      }
      else {
         m1 = this.m_invMassA;
         m2 = this.m_invMassB;
         i1 = this.m_invIA;
         i2 = this.m_invIB;
         var k11 = m1 + m2 + i1 * this.m_s1 * this.m_s1 + i2 * this.m_s2 * this.m_s2,
          impulse1 = 0;
         if (k11 !== 0.0) {
            impulse1 = ((-C1)) / k11;
         }
         else {
            impulse1 = 0.0;
         }
         impulse.x = impulse1;
         impulse.y = 0.0;
      }
      var PX = impulse.x * this.m_perp.x + impulse.y * this.m_axis.x,
          PY = impulse.x * this.m_perp.y + impulse.y * this.m_axis.y,
          L1 = impulse.x * this.m_s1 + impulse.y * this.m_a1,
          L2 = impulse.x * this.m_s2 + impulse.y * this.m_a2;
      c1.x -= this.m_invMassA * PX;
      c1.y -= this.m_invMassA * PY;
      a1 -= this.m_invIA * L1;
      c2.x += this.m_invMassB * PX;
      c2.y += this.m_invMassB * PY;
      a2 += this.m_invIB * L2;
      bA.m_sweep.a = a1;
      bB.m_sweep.a = a2;
      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
      return linearError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;
   };

   /**
    * GetType
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetType = function () {
      return this.m_type;
   };

   /**
    * GetBodyA
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetBodyA = function () {
      return this.m_bodyA;
   };

   /**
    * GetBodyB
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetBodyB = function () {
      return this.m_bodyB;
   };

   /**
    * GetNext
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetNext = function () {
      return this.m_next;
   };

   /**
    * GetUserData
    *
    * @param 
    *
    */
   b2LineJoint.prototype.GetUserData = function () {
      return this.m_userData;
   };

   /**
    * SetUserData
    *
    * @param data
    *
    */
   b2LineJoint.prototype.SetUserData = function (data) {
      this.m_userData = data;
   };

   /**
    * IsActive
    *
    * @param 
    *
    */
   b2LineJoint.prototype.IsActive = function () {
      return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
   };

   /**
    * b2Joint
    *
    * @param def
    *
    */
   b2LineJoint.prototype.b2Joint = function (def) {
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
   b2LineJoint.prototype.FinalizeVelocityConstraints = function () {};