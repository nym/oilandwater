
   /**
    *  Class b2AABB
    *
    * @param 
    *
    */
   b2AABB = Box2D.Collision.b2AABB = function b2AABB() {
      this.lowerBound = new b2Vec2(0, 0);
      this.upperBound = new b2Vec2(0, 0);

   };
   b2AABB.constructor = b2AABB;
   b2AABB.prototype.lowerBound = null;
   b2AABB.prototype.upperBound = null;

   /**
    * Static Combine
    *
    * @param aabb1
    * @param aabb2
    *
    */
   b2AABB.Combine = function (aabb1, aabb2) {
      var aabb = new b2AABB();
      aabb.Combine(aabb1, aabb2);
      return aabb;
   };

   /**
    * IsValid
    *
    * @param 
    *
    */
   b2AABB.prototype.IsValid = function () {
      var dX = this.upperBound.x - this.lowerBound.x,
          dY = this.upperBound.y - this.lowerBound.y,
          valid = dX >= 0.0 && dY >= 0.0;
      valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
      return valid;
   };

   /**
    * GetCenter
    *
    * @param 
    *
    */
   b2AABB.prototype.GetCenter = function () {
      return new b2Vec2((this.lowerBound.x + this.upperBound.x) / 2, (this.lowerBound.y + this.upperBound.y) / 2);
   };

   /**
    * GetExtents
    *
    * @param 
    *
    */
   b2AABB.prototype.GetExtents = function () {
      return new b2Vec2((this.upperBound.x - this.lowerBound.x) / 2, (this.upperBound.y - this.lowerBound.y) / 2);
   };

   /**
    * Contains
    *
    * @param aabb
    *
    */
   b2AABB.prototype.Contains = function (aabb) {
      var result = true;
      result = result && this.lowerBound.x <= aabb.lowerBound.x;
      result = result && this.lowerBound.y <= aabb.lowerBound.y;
      result = result && aabb.upperBound.x <= this.upperBound.x;
      result = result && aabb.upperBound.y <= this.upperBound.y;
      return result;
   };

   /**
    * RayCast
    *
    * @param output
    * @param input
    *
    */
   b2AABB.prototype.RayCast = function (output, input) {
      var tmin = (-b2Settings.b2_maxFloat),
          tmax = b2Settings.b2_maxFloat,
          pX = input.p1.x,
          pY = input.p1.y,
          dX = input.p2.x - input.p1.x,
          dY = input.p2.y - input.p1.y,
          absDX = Math.abs(dX),
          absDY = Math.abs(dY),
          normal = output.normal,
          inv_d = 0,
          t1 = 0,
          t2 = 0,
          t3 = 0;
      var s = 0; {
         if (absDX < b2Settings.b2_epsilon) {
            if (pX < this.lowerBound.x || this.upperBound.x < pX) return false;
         }
         else {
            inv_d = 1.0 / dX;
            t1 = (this.lowerBound.x - pX) * inv_d;
            t2 = (this.upperBound.x - pX) * inv_d;
            s = (-1.0);
            if (t1 > t2) {
               t3 = t1;
               t1 = t2;
               t2 = t3;
               s = 1.0;
            }
            if (t1 > tmin) {
               normal.x = s;
               normal.y = 0;
               tmin = t1;
            }
            tmax = Math.min(tmax, t2);
            if (tmin > tmax) return false;
         }
      } {
         if (absDY < b2Settings.b2_epsilon) {
            if (pY < this.lowerBound.y || this.upperBound.y < pY) return false;
         }
         else {
            inv_d = 1.0 / dY;
            t1 = (this.lowerBound.y - pY) * inv_d;
            t2 = (this.upperBound.y - pY) * inv_d;
            s = (-1.0);
            if (t1 > t2) {
               t3 = t1;
               t1 = t2;
               t2 = t3;
               s = 1.0;
            }
            if (t1 > tmin) {
               normal.y = s;
               normal.x = 0;
               tmin = t1;
            }
            tmax = Math.min(tmax, t2);
            if (tmin > tmax) return false;
         }
      }
      output.fraction = tmin;
      return true;
   };

   /**
    * TestOverlap
    *
    * @param other
    *
    */
   b2AABB.prototype.TestOverlap = function (other) {
      var d1X = other.lowerBound.x - this.upperBound.x,
          d1Y = other.lowerBound.y - this.upperBound.y,
          d2X = this.lowerBound.x - other.upperBound.x,
          d2Y = this.lowerBound.y - other.upperBound.y;
      if (d1X > 0.0 || d1Y > 0.0) return false;
      if (d2X > 0.0 || d2Y > 0.0) return false;
      return true;
   };

   /**
    * Combine
    *
    * @param aabb1
    * @param aabb2
    *
    */
   b2AABB.prototype.Combine = function (aabb1, aabb2) {
      this.lowerBound.x = Math.min(aabb1.lowerBound.x, aabb2.lowerBound.x);
      this.lowerBound.y = Math.min(aabb1.lowerBound.y, aabb2.lowerBound.y);
      this.upperBound.x = Math.max(aabb1.upperBound.x, aabb2.upperBound.x);
      this.upperBound.y = Math.max(aabb1.upperBound.y, aabb2.upperBound.y);
   };