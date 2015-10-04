   /**
    *  Class b2BodyDef
    *
    * @param
    *
    */
   b2BodyDef = Box2D.Dynamics.b2BodyDef = function b2BodyDef() {

      this.position = new b2Vec2(0, 0);
      this.linearVelocity = new b2Vec2(0, 0);
      this.position.Set(0.0, 0.0);
      this.linearVelocity.Set(0, 0);
   };
   /**
    * prototype properties
    *
    */
   b2BodyDef.prototype.position            = null;
   b2BodyDef.prototype.linearVelocity      = null;
   b2BodyDef.prototype.userData            = null;
   b2BodyDef.prototype.angle               = 0.0;
   b2BodyDef.prototype.linearVelocity      = null;
   b2BodyDef.prototype.angularVelocity     = 0.0;
   b2BodyDef.prototype.linearDamping       = 0.0;
   b2BodyDef.prototype.angularDamping      = 0.0;
   b2BodyDef.prototype.allowSleep          = true;
   b2BodyDef.prototype.awake               = true;
   b2BodyDef.prototype.fixedRotation       = false;
   b2BodyDef.prototype.bullet              = false;
   b2BodyDef.prototype.type                = b2Body.b2_staticBody;
   b2BodyDef.prototype.active              = true;
   b2BodyDef.prototype.inertiaScale        = 1.0;

