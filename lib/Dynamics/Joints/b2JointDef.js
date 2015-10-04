
   /**
    *  Class b2JointDef
    *
    * @param 
    *
    */
   b2JointDef = Box2D.Dynamics.Joints.b2JointDef = function b2JointDef() {};
   b2JointDef.constructor = b2JointDef;

   b2JointDef.prototype.type                = b2Joint.e_unknownJoint;
   b2JointDef.prototype.userData            = null;
   b2JointDef.prototype.bodyA               = null;
   b2JointDef.prototype.bodyB               = null;
   b2JointDef.prototype.collideConnected    = false;
