
   /**
    *  Class b2ContactSolver
    *
    * @param 
    *
    */
   b2ContactSolver = Box2D.Dynamics.Contacts.b2ContactSolver = function b2ContactSolver() {
      this.m_step = new b2TimeStep();
      this.m_constraints = [];

   };
   b2ContactSolver.constructor = b2ContactSolver;

   b2ContactSolver.prototype.m_step              = null;
   b2ContactSolver.prototype.m_constraints       = null;
   b2ContactSolver.prototype.m_allocator         = null;
   b2ContactSolver.prototype.m_constraintCount   = 0;

   b2ContactSolver.s_worldManifold = new b2WorldManifold();
   b2ContactSolver.s_psm = new b2PositionSolverManifold();

   /**
    * Initialize
    *
    * @param step
    * @param contacts
    * @param contactCount
    * @param allocator
    *
    */
   b2ContactSolver.prototype.Initialize = function (step, contacts, contactCount, allocator) {
      contactCount = contactCount || 0;
      var contact;
      this.m_step.Set(step);
      this.m_allocator = allocator;
      var i = 0,
          tVec,
          tMat;
      this.m_constraintCount = contactCount;
      while (this.m_constraints.length < this.m_constraintCount) {
         this.m_constraints[this.m_constraints.length] = new b2ContactConstraint();
      }
      for (i = 0; i < contactCount; ++i) {
         contact = contacts[i];
         var fixtureA = contact.m_fixtureA,
          fixtureB = contact.m_fixtureB,
          shapeA = fixtureA.m_shape,
          shapeB = fixtureB.m_shape,
          radiusA = shapeA.m_radius,
          radiusB = shapeB.m_radius,
          bodyA = fixtureA.m_body,
          bodyB = fixtureB.m_body,
          manifold = contact.GetManifold(),
          friction = b2Settings.b2MixFriction(fixtureA.GetFriction(), fixtureB.GetFriction()),
          restitution = b2Settings.b2MixRestitution(fixtureA.GetRestitution(), fixtureB.GetRestitution()),
          vAX = bodyA.m_linearVelocity.x,
          vAY = bodyA.m_linearVelocity.y,
          vBX = bodyB.m_linearVelocity.x,
          vBY = bodyB.m_linearVelocity.y,
          wA = bodyA.m_angularVelocity,
          wB = bodyB.m_angularVelocity;
         b2Assert(manifold.m_pointCount > 0);
         b2ContactSolver.s_worldManifold.Initialize(manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB);
         var normalX = b2ContactSolver.s_worldManifold.m_normal.x,
          normalY = b2ContactSolver.s_worldManifold.m_normal.y,
          cc = this.m_constraints[i];
         cc.bodyA = bodyA;
         cc.bodyB = bodyB;
         cc.manifold = manifold;
         cc.normal.x = normalX;
         cc.normal.y = normalY;
         cc.pointCount = manifold.m_pointCount;
         cc.friction = friction;
         cc.restitution = restitution;
         cc.localPlaneNormal.x = manifold.m_localPlaneNormal.x;
         cc.localPlaneNormal.y = manifold.m_localPlaneNormal.y;
         cc.localPoint.x = manifold.m_localPoint.x;
         cc.localPoint.y = manifold.m_localPoint.y;
         cc.radius = radiusA + radiusB;
         cc.type = manifold.m_type;
         for (var k = 0; k < cc.pointCount; ++k) {
            var cp = manifold.m_points[k],
          ccp = cc.points[k];
            ccp.normalImpulse = cp.m_normalImpulse;
            ccp.tangentImpulse = cp.m_tangentImpulse;
            ccp.localPoint.SetV(cp.m_localPoint);
            var rAX = ccp.rA.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyA.m_sweep.c.x,
          rAY = ccp.rA.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyA.m_sweep.c.y,
          rBX = ccp.rB.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyB.m_sweep.c.x,
          rBY = ccp.rB.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyB.m_sweep.c.y,
          rnA = rAX * normalY - rAY * normalX,
          rnB = rBX * normalY - rBY * normalX;
            rnA *= rnA;
            rnB *= rnB;
            var kNormal = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB;
            ccp.normalMass = 1.0 / kNormal;
            var kEqualized = bodyA.m_mass * bodyA.m_invMass + bodyB.m_mass * bodyB.m_invMass;
            kEqualized += bodyA.m_mass * bodyA.m_invI * rnA + bodyB.m_mass * bodyB.m_invI * rnB;
            ccp.equalizedMass = 1.0 / kEqualized;
            var tangentX = normalY,
          tangentY = (-normalX),
          rtA = rAX * tangentY - rAY * tangentX,
          rtB = rBX * tangentY - rBY * tangentX;
            rtA *= rtA;
            rtB *= rtB;
            var kTangent = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB;
            ccp.tangentMass = 1.0 / kTangent;
            ccp.velocityBias = 0.0;
            var tX = vBX + ((-wB * rBY)) - vAX - ((-wA * rAY)),
          tY = vBY + (wB * rBX) - vAY - (wA * rAX),
          vRel = cc.normal.x * tX + cc.normal.y * tY;
            if (vRel < (-b2Settings.b2_velocityThreshold)) {
               ccp.velocityBias += (-cc.restitution * vRel);
            }
         }
         if (cc.pointCount === 2) {
            var ccp1 = cc.points[0],
          ccp2 = cc.points[1],
          invMassA = bodyA.m_invMass,
          invIA = bodyA.m_invI,
          invMassB = bodyB.m_invMass,
          invIB = bodyB.m_invI,
          rn1A = ccp1.rA.x * normalY - ccp1.rA.y * normalX,
          rn1B = ccp1.rB.x * normalY - ccp1.rB.y * normalX,
          rn2A = ccp2.rA.x * normalY - ccp2.rA.y * normalX,
          rn2B = ccp2.rB.x * normalY - ccp2.rB.y * normalX,
          k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B,
          k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B,
          k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B,
          k_maxConditionNumber = 100.0;
            if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
               cc.K.col1.Set(k11, k12);
               cc.K.col2.Set(k12, k22);
               cc.K.GetInverse(cc.normalMass);
            }
            else {
               cc.pointCount = 1;
            }
         }
      }
   };

   /**
    * InitVelocityConstraints
    *
    * @param step
    *
    */
   b2ContactSolver.prototype.InitVelocityConstraints = function (step) {
      var tVec,
          tVec2,
          tMat;
      for (var i = 0; i < this.m_constraintCount; ++i) {
         var c = this.m_constraints[i],
          bodyA = c.bodyA,
          bodyB = c.bodyB,
          invMassA = bodyA.m_invMass,
          invIA = bodyA.m_invI,
          invMassB = bodyB.m_invMass,
          invIB = bodyB.m_invI,
          normalX = c.normal.x,
          normalY = c.normal.y,
          tangentX = normalY,
          tangentY = (-normalX),
          tX = 0,
          j = 0,
          tCount = 0;
         if (step.warmStarting) {
            tCount = c.pointCount;
            for (j = 0; j < tCount; ++j) {
               var ccp = c.points[j];
               ccp.normalImpulse *= step.dtRatio;
               ccp.tangentImpulse *= step.dtRatio;
               var PX = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX,
          PY = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;
               bodyA.m_angularVelocity -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
               bodyA.m_linearVelocity.x -= invMassA * PX;
               bodyA.m_linearVelocity.y -= invMassA * PY;
               bodyB.m_angularVelocity += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
               bodyB.m_linearVelocity.x += invMassB * PX;
               bodyB.m_linearVelocity.y += invMassB * PY;
            }
         }
         else {
            tCount = c.pointCount;
            for (j = 0; j < tCount; ++j) {
               var ccp2 = c.points[j];
               ccp2.normalImpulse = 0.0;
               ccp2.tangentImpulse = 0.0;
            }
         }
      }
   };

   /**
    * SolveVelocityConstraints
    *
    * @param 
    *
    */
   b2ContactSolver.prototype.SolveVelocityConstraints = function () {
      var j = 0,
          ccp,
          rAX = 0,
          rAY = 0,
          rBX = 0,
          rBY = 0,
          dvX = 0,
          dvY = 0,
          vn = 0,
          vt = 0,
          lambda = 0,
          maxFriction = 0,
          newImpulse = 0,
          PX = 0,
          PY = 0,
          dX = 0,
          dY = 0,
          P1X = 0,
          P1Y = 0,
          P2X = 0,
          P2Y = 0,
          tMat,
          tVec;
      for (var i = 0; i < this.m_constraintCount; ++i) {
         var c = this.m_constraints[i],
          bodyA = c.bodyA,
          bodyB = c.bodyB,
          wA = bodyA.m_angularVelocity,
          wB = bodyB.m_angularVelocity,
          vA = bodyA.m_linearVelocity,
          vB = bodyB.m_linearVelocity,
          invMassA = bodyA.m_invMass,
          invIA = bodyA.m_invI,
          invMassB = bodyB.m_invMass,
          invIB = bodyB.m_invI,
          normalX = c.normal.x,
          normalY = c.normal.y,
          tangentX = normalY,
          tangentY = (-normalX),
          friction = c.friction,
          tX = 0;
         for (j = 0; j < c.pointCount; j++) {
            ccp = c.points[j];
            dvX = vB.x - wB * ccp.rB.y - vA.x + wA * ccp.rA.y;
            dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;
            vt = dvX * tangentX + dvY * tangentY;
            lambda = ccp.tangentMass * (-vt);
            maxFriction = friction * ccp.normalImpulse;
            newImpulse = b2Math.Clamp(ccp.tangentImpulse + lambda, (-maxFriction), maxFriction);
            lambda = newImpulse - ccp.tangentImpulse;
            PX = lambda * tangentX;
            PY = lambda * tangentY;
            vA.x -= invMassA * PX;
            vA.y -= invMassA * PY;
            wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
            vB.x += invMassB * PX;
            vB.y += invMassB * PY;
            wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
            ccp.tangentImpulse = newImpulse;
         }
         var tCount = c.pointCount;
         if (c.pointCount === 1) {
            ccp = c.points[0];
            dvX = vB.x + ((-wB * ccp.rB.y)) - vA.x - ((-wA * ccp.rA.y));
            dvY = vB.y + (wB * ccp.rB.x) - vA.y - (wA * ccp.rA.x);
            vn = dvX * normalX + dvY * normalY;
            lambda = (-ccp.normalMass * (vn - ccp.velocityBias));
            newImpulse = ccp.normalImpulse + lambda;
            newImpulse = newImpulse > 0 ? newImpulse : 0.0;
            lambda = newImpulse - ccp.normalImpulse;
            PX = lambda * normalX;
            PY = lambda * normalY;
            vA.x -= invMassA * PX;
            vA.y -= invMassA * PY;
            wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
            vB.x += invMassB * PX;
            vB.y += invMassB * PY;
            wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
            ccp.normalImpulse = newImpulse;
         }
         else {
            var cp1 = c.points[0],
          cp2 = c.points[1],
          aX = cp1.normalImpulse,
          aY = cp2.normalImpulse,
          dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y,
          dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x,
          dv2X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y,
          dv2Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x,
          vn1 = dv1X * normalX + dv1Y * normalY,
          vn2 = dv2X * normalX + dv2Y * normalY,
          bX = vn1 - cp1.velocityBias,
          bY = vn2 - cp2.velocityBias;
            tMat = c.K;
            bX -= tMat.col1.x * aX + tMat.col2.x * aY;
            bY -= tMat.col1.y * aX + tMat.col2.y * aY;
            var k_errorTol = 0.001;
            for (;;) {
               tMat = c.normalMass;
               var xX = (-(tMat.col1.x * bX + tMat.col2.x * bY)),
          xY = (-(tMat.col1.y * bX + tMat.col2.y * bY));
               if (xX >= 0.0 && xY >= 0.0) {
                  dX = xX - aX;
                  dY = xY - aY;
                  P1X = dX * normalX;
                  P1Y = dX * normalY;
                  P2X = dY * normalX;
                  P2Y = dY * normalY;
                  vA.x -= invMassA * (P1X + P2X);
                  vA.y -= invMassA * (P1Y + P2Y);
                  wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                  vB.x += invMassB * (P1X + P2X);
                  vB.y += invMassB * (P1Y + P2Y);
                  wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                  cp1.normalImpulse = xX;
                  cp2.normalImpulse = xY;
                  break;
               }
               xX = (-cp1.normalMass * bX);
               xY = 0.0;
               vn1 = 0.0;
               vn2 = c.K.col1.y * xX + bY;
               if (xX >= 0.0 && vn2 >= 0.0) {
                  dX = xX - aX;
                  dY = xY - aY;
                  P1X = dX * normalX;
                  P1Y = dX * normalY;
                  P2X = dY * normalX;
                  P2Y = dY * normalY;
                  vA.x -= invMassA * (P1X + P2X);
                  vA.y -= invMassA * (P1Y + P2Y);
                  wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                  vB.x += invMassB * (P1X + P2X);
                  vB.y += invMassB * (P1Y + P2Y);
                  wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                  cp1.normalImpulse = xX;
                  cp2.normalImpulse = xY;
                  break;
               }
               xX = 0.0;
               xY = (-cp2.normalMass * bY);
               vn1 = c.K.col2.x * xY + bX;
               vn2 = 0.0;
               if (xY >= 0.0 && vn1 >= 0.0) {
                  dX = xX - aX;
                  dY = xY - aY;
                  P1X = dX * normalX;
                  P1Y = dX * normalY;
                  P2X = dY * normalX;
                  P2Y = dY * normalY;
                  vA.x -= invMassA * (P1X + P2X);
                  vA.y -= invMassA * (P1Y + P2Y);
                  wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                  vB.x += invMassB * (P1X + P2X);
                  vB.y += invMassB * (P1Y + P2Y);
                  wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                  cp1.normalImpulse = xX;
                  cp2.normalImpulse = xY;
                  break;
               }
               xX = 0.0;
               xY = 0.0;
               vn1 = bX;
               vn2 = bY;
               if (vn1 >= 0.0 && vn2 >= 0.0) {
                  dX = xX - aX;
                  dY = xY - aY;
                  P1X = dX * normalX;
                  P1Y = dX * normalY;
                  P2X = dY * normalX;
                  P2Y = dY * normalY;
                  vA.x -= invMassA * (P1X + P2X);
                  vA.y -= invMassA * (P1Y + P2Y);
                  wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
                  vB.x += invMassB * (P1X + P2X);
                  vB.y += invMassB * (P1Y + P2Y);
                  wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
                  cp1.normalImpulse = xX;
                  cp2.normalImpulse = xY;
                  break;
               }
               break;
            }
         }
         bodyA.m_angularVelocity = wA;
         bodyB.m_angularVelocity = wB;
      }
   };

   /**
    * FinalizeVelocityConstraints
    *
    * @param 
    *
    */
   b2ContactSolver.prototype.FinalizeVelocityConstraints = function () {
      for (var i = 0; i < this.m_constraintCount; ++i) {
         var c = this.m_constraints[i],
          m = c.manifold;
         for (var j = 0; j < c.pointCount; ++j) {
            var point1 = m.m_points[j],
          point2 = c.points[j];
            point1.m_normalImpulse = point2.normalImpulse;
            point1.m_tangentImpulse = point2.tangentImpulse;
         }
      }
   };

   /**
    * SolvePositionConstraints
    *
    * @param baumgarte
    *
    */
   b2ContactSolver.prototype.SolvePositionConstraints = function (baumgarte) {
      baumgarte = baumgarte || 0;
      var minSeparation = 0.0;
      for (var i = 0; i < this.m_constraintCount; i++) {
         var c = this.m_constraints[i],
          bodyA = c.bodyA,
          bodyB = c.bodyB,
          invMassA = bodyA.m_mass * bodyA.m_invMass,
          invIA = bodyA.m_mass * bodyA.m_invI,
          invMassB = bodyB.m_mass * bodyB.m_invMass,
          invIB = bodyB.m_mass * bodyB.m_invI;
         b2ContactSolver.s_psm.Initialize(c);
         var normal = b2ContactSolver.s_psm.m_normal;
         for (var j = 0; j < c.pointCount; j++) {
            var ccp = c.points[j],
          point = b2ContactSolver.s_psm.m_points[j],
          separation = b2ContactSolver.s_psm.m_separations[j],
          rAX = point.x - bodyA.m_sweep.c.x,
          rAY = point.y - bodyA.m_sweep.c.y,
          rBX = point.x - bodyB.m_sweep.c.x,
          rBY = point.y - bodyB.m_sweep.c.y;
            minSeparation = minSeparation < separation ? minSeparation : separation;
            var C = b2Math.Clamp(baumgarte * (separation + b2Settings.b2_linearSlop), (-b2Settings.b2_maxLinearCorrection), 0.0),
          impulse = (-ccp.equalizedMass * C),
          PX = impulse * normal.x,
          PY = impulse * normal.y;bodyA.m_sweep.c.x -= invMassA * PX;
            bodyA.m_sweep.c.y -= invMassA * PY;
            bodyA.m_sweep.a -= invIA * (rAX * PY - rAY * PX);
            bodyA.SynchronizeTransform();
            bodyB.m_sweep.c.x += invMassB * PX;
            bodyB.m_sweep.c.y += invMassB * PY;
            bodyB.m_sweep.a += invIB * (rBX * PY - rBY * PX);
            bodyB.SynchronizeTransform();
         }
      }
      return minSeparation > (-1.5 * b2Settings.b2_linearSlop);
   };