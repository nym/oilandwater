
   /**
    *  Class b2PulleyJointDef
    *
    * @param 
    *
    */
   b2PulleyJointDef = Box2D.Dynamics.Joints.b2PulleyJointDef = function b2PulleyJointDef() {

      this.groundAnchorA = new b2Vec2(0, 0);
      this.groundAnchorB = new b2Vec2(0, 0);
      this.localAnchorA = new b2Vec2(0, 0);
      this.localAnchorB = new b2Vec2(0, 0);
      this.groundAnchorA.Set((-1.0), 1.0);
      this.groundAnchorB.Set(1.0, 1.0);
      this.localAnchorA.Set((-1.0), 0.0);
      this.localAnchorB.Set(1.0, 0.0);
   };
   b2PulleyJointDef.constructor = b2PulleyJointDef;
   b2PulleyJointDef.prototype = Object.create(b2JointDef.prototype );
   b2PulleyJointDef.prototype.type                = b2Joint.e_pulleyJoint;
   b2PulleyJointDef.prototype.groundAnchorA       = null;
   b2PulleyJointDef.prototype.groundAnchorB       = null;
   b2PulleyJointDef.prototype.localAnchorA        = null;
   b2PulleyJointDef.prototype.localAnchorB        = null;
   b2PulleyJointDef.prototype.lengthA             = 0.0;
   b2PulleyJointDef.prototype.maxLengthA          = 0.0;
   b2PulleyJointDef.prototype.lengthB             = 0.0;
   b2PulleyJointDef.prototype.maxLengthB          = 0.0;
   b2PulleyJointDef.prototype.ratio               = 1.0;
   b2PulleyJointDef.prototype.collideConnected    = true;

   /**
    * Initialize
    *
    * @param bA
    * @param bB
    * @param gaA
    * @param gaB
    * @param anchorA
    * @param anchorB
    * @param r
    *
    */
   b2PulleyJointDef.prototype.Initialize = function (bA, bB, gaA, gaB, anchorA, anchorB, r) {
      r = r || 0;
      this.bodyA = bA;
      this.bodyB = bB;
      this.groundAnchorA.SetV(gaA);
      this.groundAnchorB.SetV(gaB);
      this.localAnchorA = this.bodyA.GetLocalPoint(anchorA);
      this.localAnchorB = this.bodyB.GetLocalPoint(anchorB);
      var d1X = anchorA.x - gaA.x,
          d1Y = anchorA.y - gaA.y;
      this.lengthA = Math.sqrt(d1X * d1X + d1Y * d1Y);
      var d2X = anchorB.x - gaB.x,
          d2Y = anchorB.y - gaB.y;
      this.lengthB = Math.sqrt(d2X * d2X + d2Y * d2Y);
      this.ratio = r;
      var C = this.lengthA + this.ratio * this.lengthB;
      this.maxLengthA = C - this.ratio * b2PulleyJoint.b2_minPulleyLength;
      this.maxLengthB = (C - b2PulleyJoint.b2_minPulleyLength) / this.ratio;
   };

   /**
    * b2JointDef
    *
    * @param 
    *
    */
   b2PulleyJointDef.prototype.b2JointDef = function () {
      this.type = b2Joint.e_unknownJoint;
      this.userData = null;
      this.bodyA = null;
      this.bodyB = null;
      this.collideConnected = false;
   };