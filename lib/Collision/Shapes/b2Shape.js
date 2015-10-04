   /**
    *  Class b2Shape
    *
    * @param
    *
    */
   b2Shape = Box2D.Collision.Shapes.b2Shape = function b2Shape() {};
   b2Shape.prototype.m_type         = b2Shape.e_unknownShape;
   b2Shape.prototype.m_radius       = b2Settings.b2_linearSlop;



   b2Shape.e_unknownShape = -1;
   b2Shape.e_circleShape = 0;
   b2Shape.e_polygonShape = 1;
   b2Shape.e_edgeShape = 2;
   b2Shape.e_shapeTypeCount = 3;
   b2Shape.e_hitCollide = 1;
   b2Shape.e_missCollide = 0;
   b2Shape.e_startsInsideCollide = -1;

   /**
    * Static TestOverlap
    *
    * @param shape1
    * @param transform1
    * @param shape2
    * @param transform2
    *
    */
   b2Shape.TestOverlap = function (shape1, transform1, shape2, transform2) {
      var input = new b2DistanceInput();
      input.proxyA = new b2DistanceProxy();
      input.proxyA.Set(shape1);
      input.proxyB = new b2DistanceProxy();
      input.proxyB.Set(shape2);
      input.transformA = transform1;
      input.transformB = transform2;
      input.useRadii = true;
      var simplexCache = new b2SimplexCache();
      simplexCache.count = 0;
      var output = new b2DistanceOutput();
      b2Distance.Distance(output, simplexCache, input);
      return output.distance < 10.0 * b2Settings.b2_epsilon;
   };

   /**
    * Set
    *
    * @param other
    *
    */
   b2Shape.prototype.Set = function (other) {
      this.m_radius = other.m_radius;
   };

   /**
    * GetType
    *
    * @param 
    *
    */
   b2Shape.prototype.GetType = function () {
      return this.m_type;
   };

