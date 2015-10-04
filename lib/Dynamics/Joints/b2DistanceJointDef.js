
   /**
    *  Class b2DistanceJointDef
    *
    * @param 
    *
    */
   b2DistanceJointDef = Box2D.Dynamics.Joints.b2DistanceJointDef = function b2DistanceJointDef() {
      this.localAnchorA = new b2Vec2(0, 0);
      this.localAnchorB = new b2Vec2(0, 0);
   };
   b2DistanceJointDef.constructor = b2DistanceJointDef;
   b2DistanceJointDef.prototype = Object.create(b2JointDef.prototype );

   b2DistanceJointDef.prototype.type              = b2Joint.e_distanceJoint;;
   b2DistanceJointDef.prototype.localAnchorA      = null;
   b2DistanceJointDef.prototype.localAnchorB      = null;
   b2DistanceJointDef.prototype.length            = 1.0;
   b2DistanceJointDef.prototype.frequencyHz       = 0.0;
   b2DistanceJointDef.prototype.dampingRatio      = 0.0;


   /**
    * Initialize
    *
    * @param bA
    * @param bB
    * @param anchorA
    * @param anchorB
    *
    */
   b2DistanceJointDef.prototype.Initialize = function (bA, bB, anchorA, anchorB) {
      this.bodyA = bA;
      this.bodyB = bB;
      this.localAnchorA.SetV(this.bodyA.GetLocalPoint(anchorA));
      this.localAnchorB.SetV(this.bodyB.GetLocalPoint(anchorB));
      var dX = anchorB.x - anchorA.x,
          dY = anchorB.y - anchorA.y;
      this.length = Math.sqrt(dX * dX + dY * dY);
      this.frequencyHz = 0.0;
      this.dampingRatio = 0.0;
   };

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2DistanceJointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };