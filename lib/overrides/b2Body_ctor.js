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
     var tMat = this.m_xf.R;
     var tVec = this.m_sweep.localCenter;
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
     if (this.m_type == b2Body.b2_dynamicBody) {
        this.m_mass = 1.0;
        this.m_invMass = 1.0;
     }
     this.m_inertiaScale = bd.inertiaScale;
     this.m_userData = bd.userData;
   };
   /**
    * prototype properties
    *
    */
   b2Body.prototype.m_xf = null;
   b2Body.prototype.m_sweep = null;
   b2Body.prototype.m_linearVelocity = null;
   b2Body.prototype.m_force = null;
   b2Body.prototype.m_flags = 0;
   b2Body.prototype.m_world = null;
   b2Body.prototype.m_jointList = null;
   b2Body.prototype.m_controllerList = null;
   b2Body.prototype.m_contactList = null;
   b2Body.prototype.m_controllerCount = 0;
   b2Body.prototype.m_prev = null;
   b2Body.prototype.m_next = null;
   b2Body.prototype.m_linearVelocity = null;
   b2Body.prototype.m_angularVelocity = null;
   b2Body.prototype.m_linearDamping = null;
   b2Body.prototype.m_angularDamping = null;
   b2Body.prototype.m_force = null;
   b2Body.prototype.m_torque = 0.0;
   b2Body.prototype.m_sleepTime = 0.0;
   b2Body.prototype.m_type = 0;
   b2Body.prototype.m_mass = 0.0;
   b2Body.prototype.m_invMass = 0.0;
   b2Body.prototype.m_I = 0.0;
   b2Body.prototype.m_invI = 0.0;
   b2Body.prototype.m_inertiaScale = 0;
   b2Body.prototype.m_userData = null;
   b2Body.prototype.m_fixtureList = null;
   b2Body.prototype.m_fixtureCount = 0;
