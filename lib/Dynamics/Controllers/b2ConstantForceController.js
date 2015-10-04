
   /**
    *  Class b2ConstantForceController
    *
    * @param 
    *
    */
   b2ConstantForceController = Box2D.Dynamics.Controllers.b2ConstantForceController = function b2ConstantForceController() {
      this.F = new b2Vec2(0, 0);
   };
   b2ConstantForceController.constructor = b2ConstantForceController;
   b2ConstantForceController.prototype = Object.create(b2Controller.prototype );
   b2ConstantForceController.prototype.F = null;
   /**
    * Step
    *
    * @param step
    *
    */
   b2ConstantForceController.prototype.Step = function (step) {
      for (var i = this.m_bodyList; i; i = i.nextBody) {
         var body = i.body;
         if (!body.IsAwake()) continue;
         body.ApplyForce(this.F, body.GetWorldCenter());
      }
   };

   /**
    * Draw
    *
    * @param debugDraw
    *
    */
   b2ConstantForceController.prototype.Draw = function (debugDraw) {};

   /**
    * AddBody
    *
    * @param body
    *
    */
   b2ConstantForceController.prototype.AddBody = function (body) {
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
   b2ConstantForceController.prototype.RemoveBody = function (body) {
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
   b2ConstantForceController.prototype.Clear = function () {
      while (this.m_bodyList)
      this.RemoveBody(this.m_bodyList.body);
   };

   /**
    * GetNext
    *
    * @param 
    *
    */
   b2ConstantForceController.prototype.GetNext = function () {
      return this.m_next;
   };

   /**
    * GetWorld
    *
    * @param 
    *
    */
   b2ConstantForceController.prototype.GetWorld = function () {
      return this.m_world;
   };

   /**
    * GetBodyList
    *
    * @param 
    *
    */
   b2ConstantForceController.prototype.GetBodyList = function () {
      return this.m_bodyList;
   };