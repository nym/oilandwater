
   /**
    *  Class b2TensorDampingController
    *
    * @param 
    *
    */
   b2TensorDampingController = Box2D.Dynamics.Controllers.b2TensorDampingController = function b2TensorDampingController() {
      this.T = new b2Mat22();
   };
   b2TensorDampingController.constructor = b2TensorDampingController;
   b2TensorDampingController.prototype = Object.create(b2Controller.prototype );
   b2TensorDampingController.prototype.T            = null;
   b2TensorDampingController.prototype.maxTimestep  = 0;
   /**
    * SetAxisAligned
    *
    * @param xDamping
    * @param yDamping
    *
    */
   b2TensorDampingController.prototype.SetAxisAligned = function (xDamping, yDamping) {
      xDamping = xDamping || 0;
      yDamping = yDamping || 0;
      this.T.col1.x = (-xDamping);
      this.T.col1.y = 0;
      this.T.col2.x = 0;
      this.T.col2.y = (-yDamping);
      if (xDamping > 0 || yDamping > 0) {
         this.maxTimestep = 1 / Math.max(xDamping, yDamping);
      }
      else {
         this.maxTimestep = 0;
      }
   };

   /**
    * Step
    *
    * @param step
    *
    */
   b2TensorDampingController.prototype.Step = function (step) {
      var timestep = step.dt;
      if (timestep <= b2Settings.b2_epsilon) return;
      if (timestep > this.maxTimestep && this.maxTimestep > 0) timestep = this.maxTimestep;
      for (var i = this.m_bodyList; i; i = i.nextBody) {
         var body = i.body;
         if (!body.IsAwake()) {
            continue;
         }
         var damping = body.GetWorldVector(b2Math.MulMV(this.T, body.GetLocalVector(body.GetLinearVelocity())));
         body.SetLinearVelocity(new b2Vec2(body.GetLinearVelocity().x + damping.x * timestep, body.GetLinearVelocity().y + damping.y * timestep));
      }
   };

   /**
    * Draw
    *
    * @param debugDraw
    *
    */
   b2TensorDampingController.prototype.Draw = function (debugDraw) {};

   /**
    * AddBody
    *
    * @param body
    *
    */
   b2TensorDampingController.prototype.AddBody = function (body) {
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
   b2TensorDampingController.prototype.RemoveBody = function (body) {
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
   b2TensorDampingController.prototype.Clear = function () {
      while (this.m_bodyList)
      this.RemoveBody(this.m_bodyList.body);
   };

   /**
    * GetNext
    *
    * @param 
    *
    */
   b2TensorDampingController.prototype.GetNext = function () {
      return this.m_next;
   };

   /**
    * GetWorld
    *
    * @param 
    *
    */
   b2TensorDampingController.prototype.GetWorld = function () {
      return this.m_world;
   };

   /**
    * GetBodyList
    *
    * @param 
    *
    */
   b2TensorDampingController.prototype.GetBodyList = function () {
      return this.m_bodyList;
   };