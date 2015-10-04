
   /**
    *  Class b2Joint
    *
    * @param def
    *
    */
   b2Joint = Box2D.Dynamics.Joints.b2Joint = function b2Joint(def) {
      this.m_edgeA = new b2JointEdge();
      this.m_edgeB = new b2JointEdge();
      this.m_localCenterA = new b2Vec2(0, 0);
      this.m_localCenterB = new b2Vec2(0, 0);
      b2Assert(def.bodyA !== def.bodyB);
      this.m_type = def.type;
      this.m_bodyA = def.bodyA;
      this.m_bodyB = def.bodyB;
      this.m_collideConnected = def.collideConnected;
      this.m_userData = def.userData;
   };
   b2Joint.constructor = b2Joint;

   b2Joint.e_unknownJoint = 0;
   b2Joint.e_revoluteJoint = 1;
   b2Joint.e_prismaticJoint = 2;
   b2Joint.e_distanceJoint = 3;
   b2Joint.e_pulleyJoint = 4;
   b2Joint.e_mouseJoint = 5;
   b2Joint.e_gearJoint = 6;
   b2Joint.e_lineJoint = 7;
   b2Joint.e_weldJoint = 8;
   b2Joint.e_frictionJoint = 9;
   b2Joint.e_inactiveLimit = 0;
   b2Joint.e_atLowerLimit = 1;
   b2Joint.e_atUpperLimit = 2;
   b2Joint.e_equalLimits = 3;

   b2Joint.prototype.m_type                  = b2Joint.e_unknownJoint;
   b2Joint.prototype.m_edgeA                 = null;
   b2Joint.prototype.m_edgeB                 = null;
   b2Joint.prototype.m_localAnchor1          = null;
   b2Joint.prototype.m_localAnchor2          = null;
   b2Joint.prototype.m_localCenterA          = null;
   b2Joint.prototype.m_localCenterB          = null;
   b2Joint.prototype.m_prev                  = null;
   b2Joint.prototype.m_next                  = null;
   b2Joint.prototype.m_bodyA                 = null;
   b2Joint.prototype.m_bodyB                 = null;
   b2Joint.prototype.m_collideConnected      = false;
   b2Joint.prototype.m_islandFlag            = false;
   b2Joint.prototype.m_userData              = null;

   /**
    * Static Create
    *
    * @param def
    * @param allocator
    *
    */
   b2Joint.Create = function (def, allocator) {
      var joint = null;
      switch (def.type) {
      case b2Joint.e_distanceJoint:
         {
            joint = new b2DistanceJoint((def instanceof b2DistanceJointDef ? def : null));
         }
         break;
      case b2Joint.e_mouseJoint:
         {
            joint = new b2MouseJoint((def instanceof b2MouseJointDef ? def : null));
         }
         break;
      case b2Joint.e_prismaticJoint:
         {
            joint = new b2PrismaticJoint((def instanceof b2PrismaticJointDef ? def : null));
         }
         break;
      case b2Joint.e_revoluteJoint:
         {
            joint = new b2RevoluteJoint((def instanceof b2RevoluteJointDef ? def : null));
         }
         break;
      case b2Joint.e_pulleyJoint:
         {
            joint = new b2PulleyJoint((def instanceof b2PulleyJointDef ? def : null));
         }
         break;
      case b2Joint.e_gearJoint:
         {
            joint = new b2GearJoint((def instanceof b2GearJointDef ? def : null));
         }
         break;
      case b2Joint.e_lineJoint:
         {
            joint = new b2LineJoint((def instanceof b2LineJointDef ? def : null));
         }
         break;
      case b2Joint.e_weldJoint:
         {
            joint = new b2WeldJoint((def instanceof b2WeldJointDef ? def : null));
         }
         break;
      case b2Joint.e_frictionJoint:
         {
            joint = new b2FrictionJoint((def instanceof b2FrictionJointDef ? def : null));
         }
         break;
      default:
         break;
      }
      return joint;
   };

   /**
    * Static Destroy
    *
    * @param joint
    * @param allocator
    *
    */
   b2Joint.Destroy = function (joint, allocator) {};

   /**
    * GetType
    *
    * @param 
    *
    */
   b2Joint.prototype.GetType = function () {
      return this.m_type;
   };

   /**
    * GetAnchorA
    *
    * @param 
    *
    */
   b2Joint.prototype.GetAnchorA = function () {
      return null;
   };

   /**
    * GetAnchorB
    *
    * @param 
    *
    */
   b2Joint.prototype.GetAnchorB = function () {
      return null;
   };

   /**
    * GetReactionForce
    *
    * @param inv_dt
    *
    */
   b2Joint.prototype.GetReactionForce = function (inv_dt) {
      inv_dt = inv_dt || 0;
      return null;
   };

   /**
    * GetReactionTorque
    *
    * @param inv_dt
    *
    */
   b2Joint.prototype.GetReactionTorque = function (inv_dt) {
      inv_dt = inv_dt || 0;
      return 0.0;
   };

   /**
    * GetBodyA
    *
    * @param 
    *
    */
   b2Joint.prototype.GetBodyA = function () {
      return this.m_bodyA;
   };

   /**
    * GetBodyB
    *
    * @param 
    *
    */
   b2Joint.prototype.GetBodyB = function () {
      return this.m_bodyB;
   };

   /**
    * GetNext
    *
    * @param 
    *
    */
   b2Joint.prototype.GetNext = function () {
      return this.m_next;
   };

   /**
    * GetUserData
    *
    * @param 
    *
    */
   b2Joint.prototype.GetUserData = function () {
      return this.m_userData;
   };

   /**
    * SetUserData
    *
    * @param data
    *
    */
   b2Joint.prototype.SetUserData = function (data) {
      this.m_userData = data;
   };

   /**
    * IsActive
    *
    * @param 
    *
    */
   b2Joint.prototype.IsActive = function () {
      return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
   };

   /**
    * InitVelocityConstraints
    *
    * @param step
    *
    */
   b2Joint.prototype.InitVelocityConstraints = function (step) {};

   /**
    * SolveVelocityConstraints
    *
    * @param step
    *
    */
   b2Joint.prototype.SolveVelocityConstraints = function (step) {};

   /**
    * FinalizeVelocityConstraints
    *
    * @param 
    *
    */
   b2Joint.prototype.FinalizeVelocityConstraints = function () {};

   /**
    * SolvePositionConstraints
    *
    * @param baumgarte
    *
    */
   b2Joint.prototype.SolvePositionConstraints = function (baumgarte) {
      baumgarte = baumgarte || 0;
      return false;
   };