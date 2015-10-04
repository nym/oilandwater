
   /**
    *  Class b2RevoluteJointDef
    *
    * @param 
    *
    */
   b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef = function b2RevoluteJointDef() {

      this.localAnchorA = new b2Vec2(0, 0);
      this.localAnchorB = new b2Vec2(0, 0);
      b2JointDef.call(this);
      this.type = b2Joint.e_revoluteJoint;
      this.localAnchorA.Set(0.0, 0.0);
      this.localAnchorB.Set(0.0, 0.0);
   };
   b2RevoluteJointDef.constructor = b2RevoluteJointDef;
   b2RevoluteJointDef.prototype = Object.create(b2JointDef.prototype );
   b2RevoluteJointDef.prototype.type                = b2Joint.e_revoluteJoint;
   b2RevoluteJointDef.prototype.localAnchorA        = null;
   b2RevoluteJointDef.prototype.localAnchorB        = null;
   b2RevoluteJointDef.prototype.referenceAngle      = 0.0;
   b2RevoluteJointDef.prototype.lowerAngle          = 0.0;
   b2RevoluteJointDef.prototype.upperAngle          = 0.0;
   b2RevoluteJointDef.prototype.maxMotorTorque      = 0.0;
   b2RevoluteJointDef.prototype.motorSpeed          = 0.0;
   b2RevoluteJointDef.prototype.enableLimit         = false;
   b2RevoluteJointDef.prototype.enableMotor         = false;

   /**
    * Initialize
    *
    * @param bA
    * @param bB
    * @param anchor
    *
    */
   b2RevoluteJointDef.prototype.Initialize = function (bA, bB, anchor) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
      this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
      this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
   };

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2RevoluteJointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };