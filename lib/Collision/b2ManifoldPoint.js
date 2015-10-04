
   /**
    *  Class b2ManifoldPoint
    *
    * @param 
    *
    */
   b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint = function b2ManifoldPoint() {
      this.m_localPoint = new b2Vec2(0, 0);
      this.m_id = new b2ContactID();
      this.Reset();
   };
   b2ManifoldPoint.constructor = b2ManifoldPoint;
   b2ManifoldPoint.prototype.m_localPoint          = null;
   b2ManifoldPoint.prototype.m_id                  = null;
   b2ManifoldPoint.prototype.m_localPoint          = null;
   b2ManifoldPoint.prototype.m_normalImpulse       = 0.0;
   b2ManifoldPoint.prototype.m_tangentImpulse      = 0.0;


   /**
    * Reset
    *
    * @param 
    *
    */
   b2ManifoldPoint.prototype.Reset = function () {
      this.m_localPoint.SetZero();
      this.m_normalImpulse = 0.0;
      this.m_tangentImpulse = 0.0;
      this.m_id.key = 0;
   };

   /**
    * Set
    *
    * @param m
    *
    */
   b2ManifoldPoint.prototype.Set = function (m) {
      this.m_localPoint.SetV(m.m_localPoint);
      this.m_normalImpulse = m.m_normalImpulse;
      this.m_tangentImpulse = m.m_tangentImpulse;
      this.m_id.Set(m.m_id);
   };