
   /**
    *  Class b2MouseJointDef
    *
    * @param 
    *
    */
   b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef = function b2MouseJointDef() {
      this.target = new b2Vec2(0, 0);
   };

   b2MouseJointDef.constructor = b2MouseJointDef;
   b2MouseJointDef.prototype = Object.create(b2JointDef.prototype );
   b2MouseJointDef.prototype.type            = b2Joint.e_mouseJoint;
   b2MouseJointDef.prototype.target          = null;
   b2MouseJointDef.prototype.maxForce        = 0.0;
   b2MouseJointDef.prototype.frequencyHz     = 5.0;
   b2MouseJointDef.prototype.dampingRatio    = 0.7;

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2MouseJointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };