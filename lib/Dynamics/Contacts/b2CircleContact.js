
   /**
    *  Class b2CircleContact
    *
    * @param 
    *
    */
   b2CircleContact = Box2D.Dynamics.Contacts.b2CircleContact = function b2CircleContact() {

      b2Contact.apply(this, arguments);
   };
   b2CircleContact.constructor = b2CircleContact;
   b2CircleContact.prototype = Object.create(b2Contact.prototype );

   /**
    * Static Create
    *
    * @param allocator
    *
    */
   b2CircleContact.Create = function (allocator) {
      return new b2CircleContact();
   };

   /**
    * Static Destroy
    *
    * @param contact
    * @param allocator
    *
    */
   b2CircleContact.Destroy = function (contact, allocator) {};

   /**
    * Reset
    *
    * @param fixtureA
    * @param fixtureB
    *
    */
   b2CircleContact.prototype.Reset = function (fixtureA, fixtureB) {
      b2Contact.prototype.Reset.call(this, fixtureA, fixtureB);
   };

   /**
    * Evaluate
    *
    * @param 
    *
    */
   b2CircleContact.prototype.Evaluate = function () {
      var bA = this.m_fixtureA.GetBody(),
          bB = this.m_fixtureB.GetBody();
      b2Collision.CollideCircles(this.m_manifold, (this.m_fixtureA.GetShape() instanceof b2CircleShape ? this.m_fixtureA.GetShape() : null), bA.m_xf, (this.m_fixtureB.GetShape() instanceof b2CircleShape ? this.m_fixtureB.GetShape() : null), bB.m_xf);
   };

   /**
    * GetManifold
    *
    * @param 
    *
    */
   b2CircleContact.prototype.GetManifold = function () {
      return this.m_manifold;
   };

   /**
    * GetWorldManifold
    *
    * @param worldManifold
    *
    */
   b2CircleContact.prototype.GetWorldManifold = function (worldManifold) {
      var bodyA = this.m_fixtureA.GetBody(),
          bodyB = this.m_fixtureB.GetBody(),
          shapeA = this.m_fixtureA.GetShape(),
          shapeB = this.m_fixtureB.GetShape();
      worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
   };

   /**
    * IsTouching
    *
    * @param 
    *
    */
   b2CircleContact.prototype.IsTouching = function () {
      return (this.m_flags & b2Contact.e_touchingFlag) === b2Contact.e_touchingFlag;
   };

   /**
    * IsContinuous
    *
    * @param 
    *
    */
   b2CircleContact.prototype.IsContinuous = function () {
      return (this.m_flags & b2Contact.e_continuousFlag) === b2Contact.e_continuousFlag;
   };

   /**
    * SetSensor
    *
    * @param sensor
    *
    */
   b2CircleContact.prototype.SetSensor = function (sensor) {
      if (sensor) {
         this.m_flags |= b2Contact.e_sensorFlag;
      }
      else {
         this.m_flags &= ~b2Contact.e_sensorFlag;
      }
   };

   /**
    * IsSensor
    *
    * @param 
    *
    */
   b2CircleContact.prototype.IsSensor = function () {
      return (this.m_flags & b2Contact.e_sensorFlag) === b2Contact.e_sensorFlag;
   };

   /**
    * SetEnabled
    *
    * @param flag
    *
    */
   b2CircleContact.prototype.SetEnabled = function (flag) {
      if (flag) {
         this.m_flags |= b2Contact.e_enabledFlag;
      }
      else {
         this.m_flags &= ~b2Contact.e_enabledFlag;
      }
   };

   /**
    * IsEnabled
    *
    * @param 
    *
    */
   b2CircleContact.prototype.IsEnabled = function () {
      return (this.m_flags & b2Contact.e_enabledFlag) === b2Contact.e_enabledFlag;
   };

   /**
    * GetNext
    *
    * @param 
    *
    */
   b2CircleContact.prototype.GetNext = function () {
      return this.m_next;
   };

   /**
    * GetFixtureA
    *
    * @param 
    *
    */
   b2CircleContact.prototype.GetFixtureA = function () {
      return this.m_fixtureA;
   };

   /**
    * GetFixtureB
    *
    * @param 
    *
    */
   b2CircleContact.prototype.GetFixtureB = function () {
      return this.m_fixtureB;
   };

   /**
    * FlagForFiltering
    *
    * @param 
    *
    */
   b2CircleContact.prototype.FlagForFiltering = function () {
      this.m_flags |= b2Contact.e_filterFlag;
   };

   /**
    * b2Contact
    *
    * @param 
    *
    */
   b2CircleContact.prototype.b2Contact = function () {};

   /**
    * Update
    *
    * @param listener
    *
    */
   b2CircleContact.prototype.Update = function (listener) {
      var tManifold = this.m_oldManifold;
      this.m_oldManifold = this.m_manifold;
      this.m_manifold = tManifold;
      this.m_flags |= b2Contact.e_enabledFlag;
      var touching = false,
          wasTouching = (this.m_flags & b2Contact.e_touchingFlag) === b2Contact.e_touchingFlag,
          bodyA = this.m_fixtureA.m_body,
          bodyB = this.m_fixtureB.m_body,
          aabbOverlap = this.m_fixtureA.m_aabb.TestOverlap(this.m_fixtureB.m_aabb);
      if (this.m_flags & b2Contact.e_sensorFlag) {
         if (aabbOverlap) {
            var shapeA = this.m_fixtureA.GetShape(),
          shapeB = this.m_fixtureB.GetShape(),
          xfA = bodyA.GetTransform(),
          xfB = bodyB.GetTransform();
            touching = b2Shape.TestOverlap(shapeA, xfA, shapeB, xfB);
         }
         this.m_manifold.m_pointCount = 0;
      }
      else {
         if (bodyA.GetType() !== b2Body.b2_dynamicBody || bodyA.IsBullet() || bodyB.GetType() !== b2Body.b2_dynamicBody || bodyB.IsBullet()) {
            this.m_flags |= b2Contact.e_continuousFlag;
         }
         else {
            this.m_flags &= ~b2Contact.e_continuousFlag;
         }
         if (aabbOverlap) {
            this.Evaluate();
            touching = this.m_manifold.m_pointCount > 0;
            for (var i = 0; i < this.m_manifold.m_pointCount; ++i) {
               var mp2 = this.m_manifold.m_points[i];
               mp2.m_normalImpulse = 0.0;
               mp2.m_tangentImpulse = 0.0;
               var id2 = mp2.m_id;
               for (var j = 0; j < this.m_oldManifold.m_pointCount; ++j) {
                  var mp1 = this.m_oldManifold.m_points[j];
                  if (mp1.m_id.key === id2.key) {
                     mp2.m_normalImpulse = mp1.m_normalImpulse;
                     mp2.m_tangentImpulse = mp1.m_tangentImpulse;
                     break;
                  }
               }
            }
         }
         else {
            this.m_manifold.m_pointCount = 0;
         }
         if (touching !== wasTouching) {
            bodyA.SetAwake(true);
            bodyB.SetAwake(true);
         }
      }
      if (touching) {
         this.m_flags |= b2Contact.e_touchingFlag;
      }
      else {
         this.m_flags &= ~b2Contact.e_touchingFlag;
      }
      if (wasTouching === false && touching === true) {
         listener.BeginContact(this);
      }
      if (wasTouching === true && touching === false) {
         listener.EndContact(this);
      }
      if ((this.m_flags & b2Contact.e_sensorFlag) === 0) {
         listener.PreSolve(this, this.m_oldManifold);
      }
   };

   /**
    * ComputeTOI
    *
    * @param sweepA
    * @param sweepB
    *
    */
   b2CircleContact.prototype.ComputeTOI = function (sweepA, sweepB) {
      b2Contact.s_input.proxyA.Set(this.m_fixtureA.GetShape());
      b2Contact.s_input.proxyB.Set(this.m_fixtureB.GetShape());
      b2Contact.s_input.sweepA = sweepA;
      b2Contact.s_input.sweepB = sweepB;
      b2Contact.s_input.tolerance = b2Settings.b2_linearSlop;
      return b2TimeOfImpact.TimeOfImpact(b2Contact.s_input);
   };