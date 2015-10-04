   /**
    *  Class b2BuoyancyController
    *
    * @param
    *
    */
   b2BuoyancyController = Box2D.Dynamics.Controllers.b2BuoyancyController = function b2BuoyancyController() {
      this.normal = new b2Vec2(0, (-1));
      this.velocity = new b2Vec2(0, 0);
   };
   b2BuoyancyController.constructor = b2BuoyancyController;
   b2BuoyancyController.prototype = Object.create(b2Controller.prototype );
   b2BuoyancyController.prototype.normal            = null;
   b2BuoyancyController.prototype.offset            = 0;
   b2BuoyancyController.prototype.density           = 0;
   b2BuoyancyController.prototype.velocity          = null;
   b2BuoyancyController.prototype.linearDrag        = 2;
   b2BuoyancyController.prototype.angularDrag       = 1;
   b2BuoyancyController.prototype.useDensity        = false;
   b2BuoyancyController.prototype.useWorldGravity   = true;
   b2BuoyancyController.prototype.gravity           = null;

   /**
    * Step
    *
    * @param step
    *
    */
   b2BuoyancyController.prototype.Step = function (step) {
      if (!this.m_bodyList) return;
      if (this.useWorldGravity) {
         this.gravity = this.GetWorld().GetGravity().Copy();
      }
      for (var i = this.m_bodyList; i; i = i.nextBody) {
         var body = i.body;
         if (body.IsAwake() === false) {
            continue;
         }
         var areac = new b2Vec2(0, 0),
          massc = new b2Vec2(0, 0),
          area = 0.0,
          mass = 0.0;
         for (var fixture = body.GetFixtureList(); fixture; fixture = fixture.GetNext()) {
            var sc = new b2Vec2(0, 0),
          sarea = fixture.GetShape().ComputeSubmergedArea(this.normal, this.offset, body.GetTransform(), sc);
            area += sarea;
            areac.x += sarea * sc.x;
            areac.y += sarea * sc.y;
            var shapeDensity = 0;
            if (this.useDensity) {
               shapeDensity = 1;
            }
            else {
               shapeDensity = 1;
            }
            mass += sarea * shapeDensity;
            massc.x += sarea * sc.x * shapeDensity;
            massc.y += sarea * sc.y * shapeDensity;
         }
         areac.x /= area;
         areac.y /= area;
         massc.x /= mass;
         massc.y /= mass;
         if (area < b2Settings.b2_epsilon) continue;
         var buoyancyForce = this.gravity.GetNegative();
         buoyancyForce.Multiply(this.density * area);
         body.ApplyForce(buoyancyForce, massc);
         var dragForce = body.GetLinearVelocityFromWorldPoint(areac);
         dragForce.Subtract(this.velocity);
         dragForce.Multiply((-this.linearDrag * area));
         body.ApplyForce(dragForce, areac);
         body.ApplyTorque((-body.GetInertia() / body.GetMass() * area * body.GetAngularVelocity() * this.angularDrag));
      }
   };

   /**
    * Draw
    *
    * @param debugDraw
    *
    */
   b2BuoyancyController.prototype.Draw = function (debugDraw) {
      var r = 1000,
          p1 = new b2Vec2(0, 0),
          p2 = new b2Vec2(0, 0);
      p1.x = this.normal.x * this.offset + this.normal.y * r;
      p1.y = this.normal.y * this.offset - this.normal.x * r;
      p2.x = this.normal.x * this.offset - this.normal.y * r;
      p2.y = this.normal.y * this.offset + this.normal.x * r;
      var color = new b2Color(0, 0, 1);
      debugDraw.DrawSegment(p1, p2, color);
   };

   /**
    * AddBody
    *
    * @param body
    *
    */
   b2BuoyancyController.prototype.AddBody = function (body) {
      var edge = new b2ControllerEdge();
      edge.controller = this;
      edge.body = body;
      edge.nextBody = this.m_bodyList;
      edge.prevBody = null;
      this.m_bodyList = edge;
      if (edge.nextBody) edge.nextBody.prevBody = edge;
      this.m_bodyCount++;
      edge.nextController = body.m_controllerList;
      edge.prevController = null;
      body.m_controllerList = edge;
      if (edge.nextController) edge.nextController.prevController = edge;
      body.m_controllerCount++;
   };

   /**
    * RemoveBody
    *
    * @param body
    *
    */
   b2BuoyancyController.prototype.RemoveBody = function (body) {
      var edge = body.m_controllerList;
      while (edge && edge.controller !== this)
      edge = edge.nextController;
      if (edge.prevBody) edge.prevBody.nextBody = edge.nextBody;
      if (edge.nextBody) edge.nextBody.prevBody = edge.prevBody;
      if (edge.nextController) edge.nextController.prevController = edge.prevController;
      if (edge.prevController) edge.prevController.nextController = edge.nextController;
      if (this.m_bodyList === edge) this.m_bodyList = edge.nextBody;
      if (body.m_controllerList === edge) body.m_controllerList = edge.nextController;
      body.m_controllerCount--;
      this.m_bodyCount--;
   };

   /**
    * Clear
    *
    * @param 
    *
    */
   b2BuoyancyController.prototype.Clear = function () {
      while (this.m_bodyList)
      this.RemoveBody(this.m_bodyList.body);
   };

   /**
    * GetNext
    *
    * @param 
    *
    */
   b2BuoyancyController.prototype.GetNext = function () {
      return this.m_next;
   };

   /**
    * GetWorld
    *
    * @param 
    *
    */
   b2BuoyancyController.prototype.GetWorld = function () {
      return this.m_world;
   };

   /**
    * GetBodyList
    *
    * @param 
    *
    */
   b2BuoyancyController.prototype.GetBodyList = function () {
      return this.m_bodyList;
   };