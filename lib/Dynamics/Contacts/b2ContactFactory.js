
   /**
    *  Class b2ContactFactory
    *
    * @param allocator
    *
    */
   b2ContactFactory = Box2D.Dynamics.Contacts.b2ContactFactory = function b2ContactFactory(allocator) {

      this.m_allocator = allocator;
      this.InitializeRegisters();
   };
   b2ContactFactory.constructor = b2ContactFactory;
   b2ContactFactory.prototype.m_allocator       = null;
   b2ContactFactory.prototype.m_registers       = null;


   /**
    * AddType
    *
    * @param createFcn
    * @param destroyFcn
    * @param type1
    * @param type2
    *
    */
   b2ContactFactory.prototype.AddType = function (createFcn, destroyFcn, type1, type2) {
      this.m_registers[type1][type2].createFcn = createFcn;
      this.m_registers[type1][type2].destroyFcn = destroyFcn;
      this.m_registers[type1][type2].primary = true;
      if (type1 !== type2) {
         this.m_registers[type2][type1].createFcn = createFcn;
         this.m_registers[type2][type1].destroyFcn = destroyFcn;
         this.m_registers[type2][type1].primary = false;
      }
   };

   /**
    * InitializeRegisters
    *
    * @param 
    *
    */
   b2ContactFactory.prototype.InitializeRegisters = function () {
      this.m_registers = [];
      for (var i = 0; i < b2Shape.e_shapeTypeCount; i++) {
         this.m_registers.push([]);
         for (var j = 0; j < b2Shape.e_shapeTypeCount; j++) {
            this.m_registers[i].push(new b2ContactRegister());
         }
      }
      this.AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circleShape, b2Shape.e_circleShape);
      this.AddType(b2PolyAndCircleContact.Create, b2PolyAndCircleContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_circleShape);
      this.AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_polygonShape);
      this.AddType(b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.e_edgeShape, b2Shape.e_circleShape);
      this.AddType(b2PolyAndEdgeContact.Create, b2PolyAndEdgeContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_edgeShape);
   };

   /**
    * Create
    *
    * @param fixtureA
    * @param fixtureB
    *
    */
   b2ContactFactory.prototype.Create = function (fixtureA, fixtureB) {
      var type1 = fixtureA.GetType(),
          type2 = fixtureB.GetType(),
          reg = this.m_registers[type1][type2],
          c;
      if (reg.pool) {
         c = reg.pool;
         reg.pool = c.m_next;
         reg.poolCount--;
         c.Reset(fixtureA, fixtureB);
         return c;
      }
      var createFcn = reg.createFcn;
      if (createFcn != null) {
         if (reg.primary) {
            c = createFcn(this.m_allocator);
            c.Reset(fixtureA, fixtureB);
            return c;
         }
         else {
            c = createFcn(this.m_allocator);
            c.Reset(fixtureB, fixtureA);
            return c;
         }
      }
      else {
         return null;
      }
   };

   /**
    * Destroy
    *
    * @param contact
    *
    */
   b2ContactFactory.prototype.Destroy = function (contact) {
      if (contact.m_manifold.m_pointCount > 0) {
         contact.m_fixtureA.m_body.SetAwake(true);
         contact.m_fixtureB.m_body.SetAwake(true);
      }
      var type1 = contact.m_fixtureA.GetType(),
          type2 = contact.m_fixtureB.GetType(),
          reg = this.m_registers[type1][type2];
      reg.poolCount++;
      contact.m_next = reg.pool;
      reg.pool = contact;
      var destroyFcn = reg.destroyFcn;
      destroyFcn(contact, this.m_allocator);
   };