
   /**
    *  Class b2GearJointDef
    *
    * @param 
    *
    */
   b2GearJointDef = Box2D.Dynamics.Joints.b2GearJointDef = function b2GearJointDef() {};
   b2GearJointDef.constructor = b2GearJointDef;
   b2GearJointDef.prototype = Object.create(b2JointDef.prototype );
   b2GearJointDef.prototype.type              = b2Joint.e_gearJoint;
   b2GearJointDef.prototype.joint1            = null;
   b2GearJointDef.prototype.joint2            = null;
   b2GearJointDef.prototype.ratio             = 1.0;

