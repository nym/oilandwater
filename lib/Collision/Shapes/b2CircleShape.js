
   /**
    *  Class b2CircleShape
    *
    * @param radius
    *
    */
   b2CircleShape = Box2D.Collision.Shapes.b2CircleShape = function b2CircleShape(radius) {

       this.m_p = new b2Vec2(0, 0);
       radius = radius || 0;
       //b2Shape.call(this);
       this.m_type = b2Shape.e_circleShape;
       this.m_radius = radius;
   };

   b2CircleShape.prototype                  = Object.create(b2Shape.prototype );
   b2CircleShape.prototype.m_type           = b2Shape.e_circleShape;
   b2CircleShape.prototype.m_p              = null;


   /**
    * Copy
    *
    * @param 
    *
    */
   b2CircleShape.prototype.Copy = function () {
      var s = new b2CircleShape();
      s.Set(this);
      return s;
   };

   /**
    * Set
    *
    * @param other
    *
    */
   b2CircleShape.prototype.Set = function (other) {
      b2Shape.prototype.Set.call(this, other);
      if (other instanceof b2CircleShape) {
         var other2 = (other instanceof b2CircleShape ? other : null);
         this.m_p.SetV(other2.m_p);
      }
   };

   /**
    * TestPoint
    *
    * @param transform
    * @param p
    *
    */
   b2CircleShape.prototype.TestPoint = function (transform, p) {
      var tMat = transform.R,
          dX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y),
          dY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
      dX = p.x - dX;
      dY = p.y - dY;
      return (dX * dX + dY * dY) <= this.m_radius * this.m_radius;
   };

   /**
    * RayCast
    *
    * @param output
    * @param input
    * @param transform
    *
    */
   b2CircleShape.prototype.RayCast = function (output, input, transform) {
      var tMat = transform.R,
          positionX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y),
          positionY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y),
          sX = input.p1.x - positionX,
          sY = input.p1.y - positionY,
          b = (sX * sX + sY * sY) - this.m_radius * this.m_radius,
          rX = input.p2.x - input.p1.x,
          rY = input.p2.y - input.p1.y,
          c = (sX * rX + sY * rY),
          rr = (rX * rX + rY * rY),
          sigma = c * c - rr * b;
      if (sigma < 0.0 || rr < b2Settings.b2_epsilon) {
         return false;
      }
      var a = (-(c + Math.sqrt(sigma)));
      if (0.0 <= a && a <= input.maxFraction * rr) {
         a /= rr;
         output.fraction = a;
         output.normal.x = sX + a * rX;
         output.normal.y = sY + a * rY;
         output.normal.Normalize();
         return true;
      }
      return false;
   };

   /**
    * ComputeAABB
    *
    * @param aabb
    * @param transform
    *
    */
   b2CircleShape.prototype.ComputeAABB = function (aabb, transform) {
      var tMat = transform.R,
          pX = transform.position.x + (tMat.col1.x * this.m_p.x + tMat.col2.x * this.m_p.y),
          pY = transform.position.y + (tMat.col1.y * this.m_p.x + tMat.col2.y * this.m_p.y);
      aabb.lowerBound.Set(pX - this.m_radius, pY - this.m_radius);
      aabb.upperBound.Set(pX + this.m_radius, pY + this.m_radius);
   };

   /**
    * ComputeMass
    *
    * @param massData
    * @param density
    *
    */
   b2CircleShape.prototype.ComputeMass = function (massData, density) {
      density = density || 0;
      massData.mass = density * b2Settings.b2_pi * this.m_radius * this.m_radius;
      massData.center.SetV(this.m_p);
      massData.I = massData.mass * (0.5 * this.m_radius * this.m_radius + (this.m_p.x * this.m_p.x + this.m_p.y * this.m_p.y));
   };

   /**
    * ComputeSubmergedArea
    *
    * @param normal
    * @param offset
    * @param xf
    * @param c
    *
    */
   b2CircleShape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
      offset = offset || 0;
      var p = b2Math.MulX(xf, this.m_p),
          l = (-(b2Math.Dot(normal, p) - offset));
      if (l < (-this.m_radius) + b2Settings.b2_epsilon) {
         return 0;
      }
      if (l > this.m_radius) {
         c.SetV(p);
         return Math.PI * this.m_radius * this.m_radius;
      }
      var r2 = this.m_radius * this.m_radius,
          l2 = l * l,
          area = r2 * (Math.asin(l / this.m_radius) + Math.PI / 2) + l * Math.sqrt(r2 - l2),
          com = (-2 / 3 * Math.pow(r2 - l2, 1.5) / area);
      c.x = p.x + normal.x * com;
      c.y = p.y + normal.y * com;
      return area;
   };

   /**
    * GetLocalPosition
    *
    * @param 
    *
    */
   b2CircleShape.prototype.GetLocalPosition = function () {
      return this.m_p;
   };

   /**
    * SetLocalPosition
    *
    * @param position
    *
    */
   b2CircleShape.prototype.SetLocalPosition = function (position) {
      this.m_p.SetV(position);
   };

   /**
    * GetRadius
    *
    * @param 
    *
    */
   b2CircleShape.prototype.GetRadius = function () {
      return this.m_radius;
   };

   /**
    * SetRadius
    *
    * @param radius
    *
    */
   b2CircleShape.prototype.SetRadius = function (radius) {
      this.m_radius = radius;
   };
