/**
 *  Class b2Body
 *
 * @param bd
 * @param world
 *
 */
b2Body = Box2D.Dynamics.b2Body = function b2Body(bd, world) {

    this.m_xf = new b2Transform();
    this.m_sweep = new b2Sweep();
    this.m_linearVelocity = new b2Vec2(0, 0);
    this.m_force = new b2Vec2(0, 0);
    if (bd.bullet) {
      this.m_flags |= b2Body.e_bulletFlag;
    }
    if (bd.fixedRotation) {
      this.m_flags |= b2Body.e_fixedRotationFlag;
    }
    if (bd.allowSleep) {
      this.m_flags |= b2Body.e_allowSleepFlag;
    }
    if (bd.awake) {
      this.m_flags |= b2Body.e_awakeFlag;
    }
    if (bd.active) {
      this.m_flags |= b2Body.e_activeFlag;
    }
    this.m_world = world;
    this.m_xf.position.SetV(bd.position);
    this.m_xf.R.Set(bd.angle);
    this.m_sweep.localCenter.SetZero();
    this.m_sweep.t0 = 1.0;
    this.m_sweep.a0 = this.m_sweep.a = bd.angle;
    var tMat = this.m_xf.R,
         tVec = this.m_sweep.localCenter;
    this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    this.m_sweep.c.x += this.m_xf.position.x;
    this.m_sweep.c.y += this.m_xf.position.y;
    this.m_sweep.c0.SetV(this.m_sweep.c);
    this.m_linearVelocity.SetV(bd.linearVelocity);
    this.m_angularVelocity = bd.angularVelocity;
    this.m_linearDamping = bd.linearDamping;
    this.m_angularDamping = bd.angularDamping;
    this.m_force.Set(0.0, 0.0);
    this.m_type = bd.type;
    if (this.m_type === b2Body.b2_dynamicBody) {
      this.m_mass = 1.0;
      this.m_invMass = 1.0;
    }
    this.m_inertiaScale = bd.inertiaScale;
    this.m_userData = bd.userData;
};

b2Body.s_xf1 = new b2Transform();
b2Body.e_islandFlag = 0x0001;
b2Body.e_awakeFlag = 0x0002;
b2Body.e_allowSleepFlag = 0x0004;
b2Body.e_bulletFlag = 0x0008;
b2Body.e_fixedRotationFlag = 0x0010;
b2Body.e_activeFlag = 0x0020;
b2Body.b2_staticBody = 0;
b2Body.b2_kinematicBody = 1;
b2Body.b2_dynamicBody = 2;
b2Body.constructor = b2Body;
b2Body.prototype = {
     m_xf: null,
     m_sweep: null,
     m_flags: 0,
     m_world: null,
     m_jointList: null,
     m_controllerList: null,
     m_contactList: null,
     m_controllerCount: 0,
     m_prev: null,
     m_next: null,
     m_linearVelocity: null,
     m_angularVelocity: null,
     m_linearDamping: null,
     m_angularDamping: null,
     m_force: null,
     m_torque: 0.0,
     m_sleepTime: 0.0,
     m_type: 0,
     m_mass: 0.0,
     m_invMass: 0.0,
     m_I: 0.0,
     m_invI: 0.0,
     m_inertiaScale: 0,
     m_userData: null,
     m_fixtureList: null,
     m_fixtureCount: 0,

     /**
      * connectEdges
      *
      * @param s1
      * @param s2
      * @param angle1
      *
      */
     connectEdges: function (s1, s2, angle1) {
          angle1 = angle1 || 0;
          var angle2 = Math.atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x),
                coreOffset = Math.tan((angle2 - angle1) * 0.5),
                core = b2Math.MulFV(coreOffset, s2.GetDirectionVector());
          core = b2Math.SubtractVV(core, s2.GetNormalVector());
          core = b2Math.MulFV(b2Settings.b2_toiSlop, core);
          core = b2Math.AddVV(core, s2.GetVertex1());
          var cornerDir = b2Math.AddVV(s1.GetDirectionVector(), s2.GetDirectionVector());
          cornerDir.Normalize();
          var convex = b2Math.Dot(s1.GetDirectionVector(), s2.GetNormalVector()) > 0.0;
          s1.SetNextEdge(s2, core, cornerDir, convex);
          s2.SetPrevEdge(s1, core, cornerDir, convex);
          return angle2;
     },

     /**
      * CreateFixture
      *
      * @param def
      * @return object
      *
      */
     CreateFixture: function (def) {
          if (this.m_world.IsLocked() === true) {
                return null;
          }
          var fixture = new b2Fixture();
          fixture.Create(this, this.m_xf, def);
          if (this.m_flags & b2Body.e_activeFlag) {
                var broadPhase = this.m_world.m_contactManager.m_broadPhase;
                fixture.CreateProxy(broadPhase, this.m_xf);
          }
          fixture.m_next = this.m_fixtureList;
          this.m_fixtureList = fixture;
          ++this.m_fixtureCount;
          fixture.m_body = this;
          if (fixture.m_density > 0.0) {
                this.ResetMassData();
          }
          this.m_world.m_flags |= b2World.e_newFixture;
          return fixture;
     },

     /**
      * CreateFixture2
      *
      * @param shape
      * @param density
      *
      */
     CreateFixture2: function (shape, density) {
          density = density || 0.0;
          var def = new b2FixtureDef();
          def.shape = shape;
          def.density = density;
          return this.CreateFixture(def);
     },

     /**
      * DestroyFixture
      *
      * @param fixture
      *
      */
     DestroyFixture: function (fixture) {
          if (this.m_world.IsLocked() === true) {
                return;
          }
          var node = this.m_fixtureList,
                ppF = null,
                found = false;
          while (node != null) {
                if (node === fixture) {
                     if (ppF) ppF.m_next = fixture.m_next;
                     else this.m_fixtureList = fixture.m_next;
                     found = true;
                     break;
                }
                ppF = node;
                node = node.m_next;
          }
          var edge = this.m_contactList;
          while (edge) {
                var c = edge.contact;
                edge = edge.next;
                var fixtureA = c.GetFixtureA(),
                     fixtureB = c.GetFixtureB();
                if (fixture === fixtureA || fixture === fixtureB) {
                     this.m_world.m_contactManager.Destroy(c);
                }
          }
          if (this.m_flags & b2Body.e_activeFlag) {
                var broadPhase = this.m_world.m_contactManager.m_broadPhase;
                fixture.DestroyProxy(broadPhase);
          }
          else {
          }
          fixture.Destroy();
          fixture.m_body = null;
          fixture.m_next = null;
          --this.m_fixtureCount;
          this.ResetMassData();
     },

     /**
      * SetPositionAndAngle
      *
      * @param position
      * @param angle
      *
      */
     SetPositionAndAngle: function (position, angle) {
          angle = angle || 0;
          var f;
          if (this.m_world.IsLocked() === true) {
                return;
          }
          this.m_xf.R.Set(angle);
          this.m_xf.position.SetV(position);
          var tMat = this.m_xf.R,
                tVec = this.m_sweep.localCenter;
          this.m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          this.m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
          this.m_sweep.c.x += this.m_xf.position.x;
          this.m_sweep.c.y += this.m_xf.position.y;
          this.m_sweep.c0.SetV(this.m_sweep.c);
          this.m_sweep.a0 = this.m_sweep.a = angle;
          var broadPhase = this.m_world.m_contactManager.m_broadPhase;
          for (f = this.m_fixtureList; f; f = f.m_next) {
                f.Synchronize(broadPhase, this.m_xf, this.m_xf);
          }
          this.m_world.m_contactManager.FindNewContacts();
     },

     /**
      * SetTransform
      *
      * @param xf
      *
      */
     SetTransform: function (xf) {
          this.SetPositionAndAngle(xf.position, xf.GetAngle());
     },

     /**
      * GetTransform
      *
      * @param
      *
      */
     GetTransform: function () {
          return this.m_xf;
     },

     /**
      * GetPosition
      *
      * @param
      *
      */
     GetPosition: function () {
          return this.m_xf.position;
     },

     /**
      * SetPosition
      *
      * @param position
      *
      */
     SetPosition: function (position) {
          this.SetPositionAndAngle(position, this.GetAngle());
     },

     /**
      * GetAngle
      *
      * @param
      *
      */
     GetAngle: function () {
          return this.m_sweep.a;
     },

     /**
      * SetAngle
      *
      * @param angle
      *
      */
     SetAngle: function (angle) {
          angle = angle || 0;
          this.SetPositionAndAngle(this.GetPosition(), angle);
     },

     /**
      * GetWorldCenter
      *
      * @param
      *
      */
     GetWorldCenter: function () {
          return this.m_sweep.c;
     },

     /**
      * GetLocalCenter
      *
      * @param
      *
      */
     GetLocalCenter: function () {
          return this.m_sweep.localCenter;
     },

     /**
      * SetLinearVelocity
      *
      * @param v
      *
      */
     SetLinearVelocity: function (v) {
          if (this.m_type === b2Body.b2_staticBody) {
                return;
          }
          this.m_linearVelocity.SetV(v);
     },

     /**
      * GetLinearVelocity
      *
      * @param
      *
      */
     GetLinearVelocity: function () {
          return this.m_linearVelocity;
     },

     /**
      * SetAngularVelocity
      *
      * @param omega
      *
      */
     SetAngularVelocity: function (omega) {
          if (this.m_type === b2Body.b2_staticBody) {
                return;
          }
          this.m_angularVelocity = omega;
     },

     /**
      * GetAngularVelocity
      *
      * @param
      *
      */
     GetAngularVelocity: function () {
          return this.m_angularVelocity;
     },

     /**
      * GetDefinition
      *
      * @param
      *
      */
     GetDefinition: function () {
          var bd = new b2BodyDef();
          bd.type = this.GetType();
          bd.allowSleep = (this.m_flags & b2Body.e_allowSleepFlag) === b2Body.e_allowSleepFlag;
          bd.angle = this.GetAngle();
          bd.angularDamping = this.m_angularDamping;
          bd.angularVelocity = this.m_angularVelocity;
          bd.fixedRotation = (this.m_flags & b2Body.e_fixedRotationFlag) === b2Body.e_fixedRotationFlag;
          bd.bullet = (this.m_flags & b2Body.e_bulletFlag) === b2Body.e_bulletFlag;
          bd.awake = (this.m_flags & b2Body.e_awakeFlag) === b2Body.e_awakeFlag;
          bd.linearDamping = this.m_linearDamping;
          bd.linearVelocity.SetV(this.GetLinearVelocity());
          bd.position = this.GetPosition();
          bd.userData = this.GetUserData();
          return bd;
     },

     /**
      * ApplyForce
      *
      * @param force
      * @param point
      *
      */
     ApplyForce: function (force, point) {
          if (this.m_type !== b2Body.b2_dynamicBody) {
                return;
          }
          if (this.IsAwake() === false) {
                this.SetAwake(true);
          }
          this.m_force.x += force.x;
          this.m_force.y += force.y;
          this.m_torque += ((point.x - this.m_sweep.c.x) * force.y - (point.y - this.m_sweep.c.y) * force.x);
     },

     /**
      * ApplyTorque
      *
      * @param torque
      *
      */
     ApplyTorque: function (torque) {
          if (this.m_type !== b2Body.b2_dynamicBody) {
                return;
          }
          if (this.IsAwake() === false) {
                this.SetAwake(true);
          }
          this.m_torque += torque;
     },

     /**
      * ApplyImpulse
      *
      * @param impulse
      * @param point
      *
      */
     ApplyImpulse: function (impulse, point) {
          if (this.m_type !== b2Body.b2_dynamicBody) {
                return;
          }
          if (this.IsAwake() === false) {
                this.SetAwake(true);
          }
          this.m_linearVelocity.x += this.m_invMass * impulse.x;
          this.m_linearVelocity.y += this.m_invMass * impulse.y;
          this.m_angularVelocity += this.m_invI * ((point.x - this.m_sweep.c.x) * impulse.y - (point.y - this.m_sweep.c.y) * impulse.x);
     },

     /**
      * Split
      *
      * @param callback
      *
      */
     Split: function (callback) {
          var linearVelocity = this.GetLinearVelocity().Copy(),
                angularVelocity = this.GetAngularVelocity(),
                center = this.GetWorldCenter(),
                body1 = this,
                body2 = this.m_world.CreateBody(this.GetDefinition()),
                prev;
          for (var f = body1.m_fixtureList; f;) {
                if (callback(f)) {
                     var next = f.m_next;
                     if (prev) {
                          prev.m_next = next;
                     }
                     else {
                          body1.m_fixtureList = next;
                     }
                     body1.m_fixtureCount--;
                     f.m_next = body2.m_fixtureList;
                     body2.m_fixtureList = f;
                     body2.m_fixtureCount++;
                     f.m_body = body2;
                     f = next;
                }
                else {
                     prev = f;
                     f = f.m_next;
                }
          }
          body1.ResetMassData();
          body2.ResetMassData();
          var center1 = body1.GetWorldCenter(),
                center2 = body2.GetWorldCenter(),
                velocity1 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center1, center))),
                velocity2 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center2, center)));
          body1.SetLinearVelocity(velocity1);
          body2.SetLinearVelocity(velocity2);
          body1.SetAngularVelocity(angularVelocity);
          body2.SetAngularVelocity(angularVelocity);
          body1.SynchronizeFixtures();
          body2.SynchronizeFixtures();
          return body2;
     },

     /**
      * Merge
      *
      * @param other
      *
      */
     Merge: function (other) {
          var f;
          for (f = other.m_fixtureList; f;) {
                var next = f.m_next;
                other.m_fixtureCount--;
                f.m_next = this.m_fixtureList;
                this.m_fixtureList = f;
                this.m_fixtureCount++;
                f.m_body = body2;
                f = next;
          }
          body1.m_fixtureCount = 0;
          var body1 = this,
                body2 = other,
                center1 = body1.GetWorldCenter(),
                center2 = body2.GetWorldCenter(),
                velocity1 = body1.GetLinearVelocity().Copy(),
                velocity2 = body2.GetLinearVelocity().Copy(),
                angular1 = body1.GetAngularVelocity(),
                angular = body2.GetAngularVelocity();
          body1.ResetMassData();
          this.SynchronizeFixtures();
     },

     /**
      * GetMass
      *
      * @param
      * @return mass
      *
      */
     GetMass: function () {
          return this.m_mass;
     },

     /**
      * GetInertia
      *
      * @param
      * @return inertia
      *
      */
     GetInertia: function () {
          return this.m_I;
     },

     /**
      * GetMassData
      *
      * @param data
      *
      */
     GetMassData: function (data) {
          data.mass = this.m_mass;
          data.I = this.m_I;
          data.center.SetV(this.m_sweep.localCenter);
     },

     /**
      * SetMassData
      *
      * @param massData
      *
      */
     SetMassData: function (massData) {
          b2Assert(this.m_world.IsLocked() === false);
          if (this.m_world.IsLocked() === true) {
                return;
          }
          if (this.m_type !== b2Body.b2_dynamicBody) {
                return;
          }
          this.m_invMass = 0.0;
          this.m_I = 0.0;
          this.m_invI = 0.0;
          this.m_mass = massData.mass;
          if (this.m_mass <= 0.0) {
                this.m_mass = 1.0;
          }
          this.m_invMass = 1.0 / this.m_mass;
          if (massData.I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) === 0) {
                this.m_I = massData.I - this.m_mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y);
                this.m_invI = 1.0 / this.m_I;
          }
          var oldCenter = this.m_sweep.c.Copy();
          this.m_sweep.localCenter.SetV(massData.center);
          this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
          this.m_sweep.c.SetV(this.m_sweep.c0);
          this.m_linearVelocity.x += this.m_angularVelocity * (-(this.m_sweep.c.y - oldCenter.y));
          this.m_linearVelocity.y += this.m_angularVelocity * (+(this.m_sweep.c.x - oldCenter.x));
     },

     /**
      * ResetMassData
      *
      * @param
      *
      */
     ResetMassData: function () {
          this.m_mass = 0.0;
          this.m_invMass = 0.0;
          this.m_I = 0.0;
          this.m_invI = 0.0;
          this.m_sweep.localCenter.SetZero();
          if (this.m_type === b2Body.b2_staticBody || this.m_type === b2Body.b2_kinematicBody) {
                return;
          }
          var center = b2Vec2.Make(0, 0);
          for (var f = this.m_fixtureList; f; f = f.m_next) {
                if (f.m_density === 0.0) {
                     continue;
                }
                var massData = f.GetMassData();
                this.m_mass += massData.mass;
                center.x += massData.center.x * massData.mass;
                center.y += massData.center.y * massData.mass;
                this.m_I += massData.I;
          }
          if (this.m_mass > 0.0) {
                this.m_invMass = 1.0 / this.m_mass;
                center.x *= this.m_invMass;
                center.y *= this.m_invMass;
          }
          else {
                this.m_mass = 1.0;
                this.m_invMass = 1.0;
          }
          if (this.m_I > 0.0 && (this.m_flags & b2Body.e_fixedRotationFlag) === 0) {
                this.m_I -= this.m_mass * (center.x * center.x + center.y * center.y);
                this.m_I *= this.m_inertiaScale;
                b2Assert(this.m_I > 0);
                this.m_invI = 1.0 / this.m_I;
          }
          else {
                this.m_I = 0.0;
                this.m_invI = 0.0;
          }
          var oldCenter = this.m_sweep.c.Copy();
          this.m_sweep.localCenter.SetV(center);
          this.m_sweep.c0.SetV(b2Math.MulX(this.m_xf, this.m_sweep.localCenter));
          this.m_sweep.c.SetV(this.m_sweep.c0);
          this.m_linearVelocity.x += this.m_angularVelocity * (-(this.m_sweep.c.y - oldCenter.y));
          this.m_linearVelocity.y += this.m_angularVelocity * (+(this.m_sweep.c.x - oldCenter.x));
     },

     /**
      * GetWorldPoint
      *
      * @param localPoint
      *
      */
     GetWorldPoint: function (localPoint) {
          var A = this.m_xf.R,
                u = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
          u.x += this.m_xf.position.x;
          u.y += this.m_xf.position.y;
          return u;
     },

     /**
      * GetWorldVector
      *
      * @param localVector
      *
      */
     GetWorldVector: function (localVector) {
          return b2Math.MulMV(this.m_xf.R, localVector);
     },

     /**
      * GetLocalPoint
      *
      * @param worldPoint
      *
      */
     GetLocalPoint: function (worldPoint) {
          return b2Math.MulXT(this.m_xf, worldPoint);
     },

     /**
      * GetLocalVector
      *
      * @param worldVector
      *
      */
     GetLocalVector: function (worldVector) {
          return b2Math.MulTMV(this.m_xf.R, worldVector);
     },

     /**
      * GetLinearVelocityFromWorldPoint
      *
      * @param worldPoint
      *
      */
     GetLinearVelocityFromWorldPoint: function (worldPoint) {
          return new b2Vec2(this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
     },

     /**
      * GetLinearVelocityFromLocalPoint
      *
      * @param localPoint
      *
      */
     GetLinearVelocityFromLocalPoint: function (localPoint) {
          var A = this.m_xf.R,
                worldPoint = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
          worldPoint.x += this.m_xf.position.x;
          worldPoint.y += this.m_xf.position.y;
          return new b2Vec2(this.m_linearVelocity.x - this.m_angularVelocity * (worldPoint.y - this.m_sweep.c.y), this.m_linearVelocity.y + this.m_angularVelocity * (worldPoint.x - this.m_sweep.c.x));
     },

     /**
      * GetLinearDamping
      *
      * @param
      *
      */
     GetLinearDamping: function () {
          return this.m_linearDamping;
     },

     /**
      * SetLinearDamping
      *
      * @param linearDamping
      *
      */
     SetLinearDamping: function (linearDamping) {
          linearDamping = linearDamping || 0;
          this.m_linearDamping = linearDamping;
     },

     /**
      * GetAngularDamping
      *
      * @param
      *
      */
     GetAngularDamping: function () {
          return this.m_angularDamping;
     },

     /**
      * SetAngularDamping
      *
      * @param angularDamping
      *
      */
     SetAngularDamping: function (angularDamping) {
          angularDamping = angularDamping || 0;
          this.m_angularDamping = angularDamping;
     },

     /**
      * SetType
      *
      * @param type
      *
      */
     SetType: function (type) {
          type = type || 0;
          if (this.m_type === type) {
                return;
          }
          this.m_type = type;
          this.ResetMassData();
          if (this.m_type === b2Body.b2_staticBody) {
                this.m_linearVelocity.SetZero();
                this.m_angularVelocity = 0.0;
          }
          this.SetAwake(true);
          this.m_force.SetZero();
          this.m_torque = 0.0;
          for (var ce = this.m_contactList; ce; ce = ce.next) {
                ce.contact.FlagForFiltering();
          }
     },

     /**
      * GetType
      *
      * @param
      *
      */
     GetType: function () {
          return this.m_type;
     },

     /**
      * SetBullet
      *
      * @param flag
      *
      */
     SetBullet: function (flag) {
          if (flag) {
                this.m_flags |= b2Body.e_bulletFlag;
          }
          else {
                this.m_flags &= ~b2Body.e_bulletFlag;
          }
     },

     /**
      * IsBullet
      *
      * @param
      *
      */
     IsBullet: function () {
          return (this.m_flags & b2Body.e_bulletFlag) === b2Body.e_bulletFlag;
     },

     /**
      * SetSleepingAllowed
      *
      * @param flag
      *
      */
     SetSleepingAllowed: function (flag) {
          if (flag) {
                this.m_flags |= b2Body.e_allowSleepFlag;
          }
          else {
                this.m_flags &= ~b2Body.e_allowSleepFlag;
                this.SetAwake(true);
          }
     },

     /**
      * SetAwake
      *
      * @param flag
      *
      */
     SetAwake: function (flag) {
          if (flag) {
                this.m_flags |= b2Body.e_awakeFlag;
                this.m_sleepTime = 0.0;
          }
          else {
                this.m_flags &= ~b2Body.e_awakeFlag;
                this.m_sleepTime = 0.0;
                this.m_linearVelocity.SetZero();
                this.m_angularVelocity = 0.0;
                this.m_force.SetZero();
                this.m_torque = 0.0;
          }
     },

     /**
      * IsAwake
      *
      * @param
      *
      */
     IsAwake: function () {
          return (this.m_flags & b2Body.e_awakeFlag) === b2Body.e_awakeFlag;
     },

     /**
      * SetFixedRotation
      *
      * @param fixed
      *
      */
     SetFixedRotation: function (fixed) {
          if (fixed) {
                this.m_flags |= b2Body.e_fixedRotationFlag;
          }
          else {
                this.m_flags &= ~b2Body.e_fixedRotationFlag;
          }
          this.ResetMassData();
     },

     /**
      * IsFixedRotation
      *
      * @param
      *
      */
     IsFixedRotation: function () {
          return (this.m_flags & b2Body.e_fixedRotationFlag) === b2Body.e_fixedRotationFlag;
     },

     /**
      * SetActive
      *
      * @param flag
      *
      */
     SetActive: function (flag) {
          if (flag === this.IsActive()) {
                return;
          }
          var broadPhase,
                f;
          if (flag) {
                this.m_flags |= b2Body.e_activeFlag;
                broadPhase = this.m_world.m_contactManager.m_broadPhase;
                for (f = this.m_fixtureList; f; f = f.m_next) {
                     f.CreateProxy(broadPhase, this.m_xf);
                }
          }
          else {
                this.m_flags &= ~b2Body.e_activeFlag;
                broadPhase = this.m_world.m_contactManager.m_broadPhase;
                for (f = this.m_fixtureList; f; f = f.m_next) {
                     f.DestroyProxy(broadPhase);
                }
                var ce = this.m_contactList;
                while (ce) {
                     var ce0 = ce;
                     ce = ce.next;
                     this.m_world.m_contactManager.Destroy(ce0.contact);
                }
                this.m_contactList = null;
          }
     },

     /**
      * IsActive
      *
      * @param
      *
      */
     IsActive: function () {
          return (this.m_flags & b2Body.e_activeFlag) === b2Body.e_activeFlag;
     },

     /**
      * IsSleepingAllowed
      *
      * @param
      *
      */
     IsSleepingAllowed: function () {
          return (this.m_flags & b2Body.e_allowSleepFlag) === b2Body.e_allowSleepFlag;
     },

     /**
      * GetFixtureList
      *
      * @param
      *
      */
     GetFixtureList: function () {
          return this.m_fixtureList;
     },

     /**
      * GetJointList
      *
      * @param
      *
      */
     GetJointList: function () {
          return this.m_jointList;
     },

     /**
      * GetControllerList
      *
      * @param
      *
      */
     GetControllerList: function () {
          return this.m_controllerList;
     },

     /**
      * GetContactList
      *
      * @param
      *
      */
     GetContactList: function () {
          return this.m_contactList;
     },

     /**
      * GetNext
      *
      * @param
      *
      */
     GetNext: function () {
          return this.m_next;
     },

     /**
      * GetUserData
      *
      * @param
      *
      */
     GetUserData: function () {
          return this.m_userData;
     },

     /**
      * SetUserData
      *
      * @param data
      *
      */
     SetUserData: function (data) {
          this.m_userData = data;
     },

     /**
      * GetWorld
      *
      * @param
      *
      */
     GetWorld: function () {
          return this.m_world;
     },

     /**
      * SynchronizeFixtures
      *
      * @param
      *
      */
     SynchronizeFixtures: function () {
          var xf1 = b2Body.s_xf1;
          xf1.R.Set(this.m_sweep.a0);
          var tMat = xf1.R,
                tVec = this.m_sweep.localCenter;
          xf1.position.x = this.m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          xf1.position.y = this.m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
          var f,
                broadPhase = this.m_world.m_contactManager.m_broadPhase;
          for (f = this.m_fixtureList; f; f = f.m_next) {
                f.Synchronize(broadPhase, xf1, this.m_xf);
          }
     },

     /**
      * SynchronizeTransform
      *
      * @param
      *
      */
     SynchronizeTransform: function () {
          this.m_xf.R.Set(this.m_sweep.a);
          var tMat = this.m_xf.R,
                tVec = this.m_sweep.localCenter;
          this.m_xf.position.x = this.m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
          this.m_xf.position.y = this.m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
     },

     /**
      * ShouldCollide
      *
      * @param other
      *
      */
     ShouldCollide: function (other) {
          if (this.m_type !== b2Body.b2_dynamicBody && other.m_type !== b2Body.b2_dynamicBody) {
                return false;
          }
          for (var jn = this.m_jointList; jn; jn = jn.next) {
                if (jn.other === other) if (jn.joint.m_collideConnected === false) {
                     return false;
                }
          }
          return true;
     },

     /**
      * Advance
      *
      * @param t
      *
      */
     Advance: function (t) {
          t = t || 0;
          this.m_sweep.Advance(t);
          this.m_sweep.c.SetV(this.m_sweep.c0);
          this.m_sweep.a = this.m_sweep.a0;
          this.SynchronizeTransform();
     }
};