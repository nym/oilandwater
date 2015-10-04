
   /**
    *  Class b2PrismaticJointDef
    *
    * @param 
    *
    */
   b2PrismaticJointDef = Box2D.Dynamics.Joints.b2PrismaticJointDef = function b2PrismaticJointDef() {

      this.localAnchorA = new b2Vec2(0, 0);
      this.localAnchorB = new b2Vec2(0, 0);
      this.localAxisA = new b2Vec2(0, 0);
      b2JointDef.call(this);
      this.type = b2Joint.e_prismaticJoint;
      this.localAxisA.Set(1.0, 0.0);
   };
   b2PrismaticJointDef.constructor = b2PrismaticJointDef;
   b2PrismaticJointDef.prototype = Object.create(b2JointDef.prototype );
   b2PrismaticJointDef.prototype.type                = b2Joint.e_prismaticJoint;
   b2PrismaticJointDef.prototype.localAnchorA        = null;
   b2PrismaticJointDef.prototype.localAnchorB        = null;
   b2PrismaticJointDef.prototype.localAxisA          = null;
   b2PrismaticJointDef.prototype.referenceAngle      = 0.0;
   b2PrismaticJointDef.prototype.enableLimit         = false;
   b2PrismaticJointDef.prototype.lowerTranslation    = 0.0;
   b2PrismaticJointDef.prototype.upperTranslation    = 0.0;
   b2PrismaticJointDef.prototype.enableMotor         = false;
   b2PrismaticJointDef.prototype.maxMotorForce       = 0.0;
   b2PrismaticJointDef.prototype.motorSpeed          = 0.0;

   /**
    * Initialize
    *
    * @param bA
    * @param bB
    * @param anchor
    * @param axis
    *
    */
   b2PrismaticJointDef.prototype.Initialize = function (bA, bB, anchor, axis) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
      this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
      this.localAxisA = this.bodyA.GetLocalVector(axis);
      this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
   };

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2PrismaticJointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };