
   /**
    *  Class b2LineJointDef
    *
    * @param 
    *
    */
   b2LineJointDef = Box2D.Dynamics.Joints.b2LineJointDef = function b2LineJointDef() {

      this.localAnchorA = new b2Vec2(0, 0);
      this.localAnchorB = new b2Vec2(0, 0);
      this.localAxisA = new b2Vec2(0, 0);
      this.localAxisA.Set(1.0, 0.0);
   };
   b2LineJointDef.constructor = b2LineJointDef;
   b2LineJointDef.prototype = Object.create(b2JointDef.prototype );
   b2LineJointDef.prototype.type                = b2Joint.e_lineJoint;
   b2LineJointDef.prototype.localAnchorA        = null;
   b2LineJointDef.prototype.localAnchorB        = null;
   b2LineJointDef.prototype.localAxisA          = null;
   b2LineJointDef.prototype.enableLimit         = false;
   b2LineJointDef.prototype.lowerTranslation    = 0.0;
   b2LineJointDef.prototype.upperTranslation    = 0.0;
   b2LineJointDef.prototype.enableMotor         = false;
   b2LineJointDef.prototype.maxMotorForce       = 0.0;
   b2LineJointDef.prototype.motorSpeed          = 0.0;

   /**
    * Initialize
    *
    * @param bA
    * @param bB
    * @param anchor
    * @param axis
    *
    */
   b2LineJointDef.prototype.Initialize = function (bA, bB, anchor, axis) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
      this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
      this.localAxisA = this.bodyA.GetLocalVector(axis);
   };

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2LineJointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };