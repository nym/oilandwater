
   /**
    *  Class b2Segment
    *
    * @param 
    *
    */
   b2Segment = Box2D.Collision.b2Segment = function b2Segment() {
      this.p1 = new b2Vec2(0, 0);
      this.p2 = new b2Vec2(0, 0);

   };
   b2Segment.constructor = b2Segment;
   b2Segment.prototype.p1 = null;
   b2Segment.prototype.p2 = null;

   /**
    * TestSegment
    *
    * @param lambda
    * @param normal
    * @param segment
    * @param maxLambda
    *
    */
   b2Segment.prototype.TestSegment = function (lambda, normal, segment, maxLambda) {
      maxLambda = maxLambda || 0;
      var s = segment.p1,
          rX = segment.p2.x - s.x,
          rY = segment.p2.y - s.y,
          dX = this.p2.x - this.p1.x,
          dY = this.p2.y - this.p1.y,
          nX = dY,
          nY = (-dX),
          k_slop = 100.0 * b2Settings.b2_epsilon,
          denom = (-(rX * nX + rY * nY));
      if (denom > k_slop) {
         var bX = s.x - this.p1.x,
          bY = s.y - this.p1.y,
          a = (bX * nX + bY * nY);
         if (0.0 <= a && a <= maxLambda * denom) {
            var mu2 = (-rX * bY) + rY * bX;
            if ((-k_slop * denom) <= mu2 && mu2 <= denom * (1.0 + k_slop)) {
               a /= denom;
               var nLen = Math.sqrt(nX * nX + nY * nY);
               nX /= nLen;
               nY /= nLen;
               lambda[0] = a;
               normal.Set(nX, nY);
               return true;
            }
         }
      }
      return false;
   };

   /**
    * Extend
    *
    * @param aabb
    *
    */
   b2Segment.prototype.Extend = function (aabb) {
      this.ExtendForward(aabb);
      this.ExtendBackward(aabb);
   };

   /**
    * ExtendForward
    *
    * @param aabb
    *
    */
   b2Segment.prototype.ExtendForward = function (aabb) {
      var dX = this.p2.x - this.p1.x,
          dY = this.p2.y - this.p1.y;
      var lambda = Math.min(dX > 0 ? (aabb.upperBound.x - this.p1.x) / dX : dX < 0 ? (aabb.lowerBound.x - this.p1.x) / dX : Number.POSITIVE_INFINITY,
      dY > 0 ? (aabb.upperBound.y - this.p1.y) / dY : dY < 0 ? (aabb.lowerBound.y - this.p1.y) / dY : Number.POSITIVE_INFINITY);
      this.p2.x = this.p1.x + dX * lambda;
      this.p2.y = this.p1.y + dY * lambda;
   };

   /**
    * ExtendBackward
    *
    * @param aabb
    *
    */
   b2Segment.prototype.ExtendBackward = function (aabb) {
      var dX = (-this.p2.x) + this.p1.x,
          dY = (-this.p2.y) + this.p1.y;
      var lambda = Math.min(dX > 0 ? (aabb.upperBound.x - this.p2.x) / dX : dX < 0 ? (aabb.lowerBound.x - this.p2.x) / dX : Number.POSITIVE_INFINITY,
      dY > 0 ? (aabb.upperBound.y - this.p2.y) / dY : dY < 0 ? (aabb.lowerBound.y - this.p2.y) / dY : Number.POSITIVE_INFINITY);
      this.p1.x = this.p2.x + dX * lambda;
      this.p1.y = this.p2.y + dY * lambda;
   };