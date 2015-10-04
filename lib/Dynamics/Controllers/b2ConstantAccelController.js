
   /**
    *  Class b2ConstantAccelController
    *
    * @param 
    *
    */
   b2ConstantAccelController = Box2D.Dynamics.Controllers.b2ConstantAccelController = function b2ConstantAccelController() {
      this.A = new b2Vec2(0, 0);
   };
   b2ConstantAccelController.constructor = b2ConstantAccelController;
   b2ConstantAccelController.prototype = Object.create(b2Controller.prototype );
   b2ConstantAccelController.prototype.A = null;
   /**
    * Step
    *
    * @param step
    *
    */
   b2ConstantAccelController.prototype.Step = function (step) {
      var smallA = new b2Vec2(this.A.x * step.dt, this.A.y * step.dt);
      for (var i = this.m_bodyList; i; i = i.nextBody) {
         var body = i.body;
         if (!body.IsAwake()) continue;
         body.SetLinearVelocity(new b2Vec2(body.GetLinearVelocity().x + smallA.x, body.GetLinearVelocity().y + smallA.y));
      }
   };

   /**
    * Draw
    *
    * @param debugDraw
    *
    */
   b2ConstantAccelController.prototype.Draw = function (debugDraw) {};

   /**
    * AddBody
    *
    * @param body
    *
    */
   b2ConstantAccelController.prototype.AddBody = function (body) {
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
   b2ConstantAccelController.prototype.RemoveBody = function (body) {
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
   b2ConstantAccelController.prototype.Clear = function () {
      while (this.m_bodyList)
      this.RemoveBody(this.m_bodyList.body);
   };

   /**
    * GetNext
    *
    * @param 
    *
    */
   b2ConstantAccelController.prototype.GetNext = function () {
      return this.m_next;
   };

   /**
    * GetWorld
    *
    * @param 
    *
    */
   b2ConstantAccelController.prototype.GetWorld = function () {
      return this.m_world;
   };

   /**
    * GetBodyList
    *
    * @param 
    *
    */
   b2ConstantAccelController.prototype.GetBodyList = function () {
      return this.m_bodyList;
   };