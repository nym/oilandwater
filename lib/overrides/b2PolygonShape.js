
   /**
    *  Class b2PolygonShape
    *
    * @param 
    *
    */
   b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape = function b2PolygonShape() {

      b2Shape.call(this);
      this.m_type = b2Shape.e_polygonShape;
      this.m_centroid = new b2Vec2(0, 0);
      this.m_vertices = [];
      this.m_normals = [];
   };
   b2PolygonShape.constructor = b2PolygonShape;
   b2PolygonShape.prototype = Object.create(b2Shape.prototype );

   b2PolygonShape.s_mat = new b2Mat22();

   /**
    * Static AsArray
    *
    * @param vertices
    * @param vertexCount
    *
    */
   b2PolygonShape.AsArray = function (vertices, vertexCount) {
      vertexCount = vertexCount || 0;
      var polygonShape = new b2PolygonShape();
      polygonShape.SetAsArray(vertices, vertexCount);
      return polygonShape;
   };

   /**
    * Static AsVector
    *
    * @param vertices
    * @param vertexCount
    *
    */
   b2PolygonShape.AsVector = function (vertices, vertexCount) {
      vertexCount = vertexCount || 0;
      var polygonShape = new b2PolygonShape();
      polygonShape.SetAsVector(vertices, vertexCount);
      return polygonShape;
   };

   /**
    * Static AsBox
    *
    * @param hx
    * @param hy
    *
    */
   b2PolygonShape.AsBox = function (hx, hy) {
      hx = hx || 0;
      hy = hy || 0;
      var polygonShape = new b2PolygonShape();
      polygonShape.SetAsBox(hx, hy);
      return polygonShape;
   };

   /**
    * Static AsOrientedBox
    *
    * @param hx
    * @param hy
    * @param center
    * @param angle
    *
    */
   b2PolygonShape.AsOrientedBox = function (hx, hy, center, angle) {
      hx = hx || 0;
      hy = hy || 0;
      center = center || null;
      angle = angle || 0.0;
      var polygonShape = new b2PolygonShape();
      polygonShape.SetAsOrientedBox(hx, hy, center, angle);
      return polygonShape;
   };

   /**
    * Static AsEdge
    *
    * @param v1
    * @param v2
    *
    */
   b2PolygonShape.AsEdge = function (v1, v2) {
      var polygonShape = new b2PolygonShape();
      polygonShape.SetAsEdge(v1, v2);
      return polygonShape;
   };

   /**
    * Static ComputeCentroid
    *
    * @param vs
    * @param count
    *
    */
   b2PolygonShape.ComputeCentroid = function (vs, count) {
      count = count || 0;
      var c = new b2Vec2(0, 0),
          area = 0.0,
          p1X = 0.0,
          p1Y = 0.0,
          inv3 = 1.0 / 3.0;
      for (var i = 0; i < count; ++i) {
         var p2 = vs[i],
          p3 = i + 1 < count ? vs[parseInt(i + 1)] : vs[0],
          e1X = p2.x - p1X,
          e1Y = p2.y - p1Y,
          e2X = p3.x - p1X,
          e2Y = p3.y - p1Y,
          D = (e1X * e2Y - e1Y * e2X),
          triangleArea = 0.5 * D;area += triangleArea;
         c.x += triangleArea * inv3 * (p1X + p2.x + p3.x);
         c.y += triangleArea * inv3 * (p1Y + p2.y + p3.y);
      }
      c.x *= 1.0 / area;
      c.y *= 1.0 / area;
      return c;
   };

   /**
    * Static ComputeOBB
    *
    * @param obb
    * @param vs
    * @param count
    *
    */
   b2PolygonShape.ComputeOBB = function (obb, vs, count) {
      count = count || 0;
      var i = 0,
          p = new Array(count + 1);
      for (i = 0;
      i < count; ++i) {
         p[i] = vs[i];
      }
      p[count] = p[0];
      var minArea = b2Settings.b2_maxFloat;
      for (i = 1;
      i <= count; ++i) {
         var root = p[parseInt(i - 1)],
          uxX = p[i].x - root.x,
          uxY = p[i].y - root.y,
          length = Math.sqrt(uxX * uxX + uxY * uxY);
         uxX /= length;
         uxY /= length;
         var uyX = (-uxY),
          uyY = uxX,
          lowerX = b2Settings.b2_maxFloat,
          lowerY = b2Settings.b2_maxFloat,
          upperX = (-b2Settings.b2_maxFloat),
          upperY = (-b2Settings.b2_maxFloat);
         for (var j = 0; j < count; ++j) {
            var dX = p[j].x - root.x,
          dY = p[j].y - root.y,
          rX = (uxX * dX + uxY * dY),
          rY = (uyX * dX + uyY * dY);
            if (rX < lowerX) lowerX = rX;
            if (rY < lowerY) lowerY = rY;
            if (rX > upperX) upperX = rX;
            if (rY > upperY) upperY = rY;
         }
         var area = (upperX - lowerX) * (upperY - lowerY);
         if (area < 0.95 * minArea) {
            minArea = area;
            obb.R.col1.x = uxX;
            obb.R.col1.y = uxY;
            obb.R.col2.x = uyX;
            obb.R.col2.y = uyY;
            var centerX = 0.5 * (lowerX + upperX),
          centerY = 0.5 * (lowerY + upperY),
          tMat = obb.R;
            obb.center.x = root.x + (tMat.col1.x * centerX + tMat.col2.x * centerY);
            obb.center.y = root.y + (tMat.col1.y * centerX + tMat.col2.y * centerY);
            obb.extents.x = 0.5 * (upperX - lowerX);
            obb.extents.y = 0.5 * (upperY - lowerY);
         }
      }
   };

   /**
    * Copy
    *
    * @param 
    *
    */
   b2PolygonShape.prototype.Copy = function () {
      var s = new b2PolygonShape();
      s.Set(this);
      return s;
   };

   /**
    * Set
    *
    * @param other
    *
    */
   b2PolygonShape.prototype.Set = function (other) {
      b2Shape.prototype.Set.call(this, other);
      if (Box2D.is(other, b2PolygonShape)) {
         var other2 = (other instanceof b2PolygonShape ? other : null);
         this.m_centroid.SetV(other2.m_centroid);
         this.m_vertexCount = other2.m_vertexCount;
         this.Reserve(this.m_vertexCount);
         for (var i = 0; i < this.m_vertexCount; i++) {
            this.m_vertices[i].SetV(other2.m_vertices[i]);
            this.m_normals[i].SetV(other2.m_normals[i]);
         }
      }
   };

   /**
    * SetAsArray
    *
    * @param vertices
    * @param vertexCount
    *
    */
   b2PolygonShape.prototype.SetAsArray = function (vertices, vertexCount) {
      vertexCount = vertexCount || 0;
      var v = [];
      var i = 0,
         tVec;
      for (i = 0;
      i < vertices.length; ++i) {
         tVec = vertices[i];
         v.push(tVec);
      }
      this.SetAsVector(v, vertexCount);
   };

   /**
    * SetAsVector
    *
    * @param vertices
    * @param vertexCount
    *
    */
   b2PolygonShape.prototype.SetAsVector = function (vertices, vertexCount) {
      vertexCount = vertexCount || 0;
      if (vertexCount === 0) vertexCount = vertices.length;
      b2Assert(2 <= vertexCount);
      this.m_vertexCount = vertexCount;
      this.Reserve(vertexCount);
      var i = 0;
      for (i = 0;
      i < this.m_vertexCount; i++) {
         this.m_vertices[i].SetV(vertices[i]);
      }
      for (i = 0;
      i < this.m_vertexCount; ++i) {
         var i1 = parseInt(i),
          i2 = parseInt(i + 1 < this.m_vertexCount ? i + 1 : 0),
          edge = b2Math.SubtractVV(this.m_vertices[i2], this.m_vertices[i1]);
         b2Assert(edge.LengthSquared() > b2Settings.b2_epsilon);
         this.m_normals[i].SetV(b2Math.CrossVF(edge, 1.0));
         this.m_normals[i].Normalize();
      }
      this.m_centroid = b2PolygonShape.ComputeCentroid(this.m_vertices, this.m_vertexCount);
   };

   /**
    * SetAsBox
    *
    * @param hx
    * @param hy
    *
    */
   b2PolygonShape.prototype.SetAsBox = function (hx, hy) {
      hx = hx || 0;
      hy = hy || 0;
      this.m_vertexCount = 4;
      this.Reserve(4);
      this.m_vertices[0].Set((-hx), (-hy));
      this.m_vertices[1].Set(hx, (-hy));
      this.m_vertices[2].Set(hx, hy);
      this.m_vertices[3].Set((-hx), hy);
      this.m_normals[0].Set(0.0, (-1.0));
      this.m_normals[1].Set(1.0, 0.0);
      this.m_normals[2].Set(0.0, 1.0);
      this.m_normals[3].Set((-1.0), 0.0);
      this.m_centroid.SetZero();
   };

   /**
    * SetAsOrientedBox
    *
    * @param hx
    * @param hy
    * @param center
    * @param angle
    *
    */
   b2PolygonShape.prototype.SetAsOrientedBox = function (hx, hy, center, angle) {
      hx = hx || 0;
      hy = hy || 0;
      center = center || null;
      angle = angle || 0.0;
      this.m_vertexCount = 4;
      this.Reserve(4);
      this.m_vertices[0].Set((-hx), (-hy));
      this.m_vertices[1].Set(hx, (-hy));
      this.m_vertices[2].Set(hx, hy);
      this.m_vertices[3].Set((-hx), hy);
      this.m_normals[0].Set(0.0, (-1.0));
      this.m_normals[1].Set(1.0, 0.0);
      this.m_normals[2].Set(0.0, 1.0);
      this.m_normals[3].Set((-1.0), 0.0);
      this.m_centroid = center;
      var xf = new b2Transform();
      xf.position = center;
      xf.R.Set(angle);
      for (var i = 0; i < this.m_vertexCount; ++i) {
         this.m_vertices[i] = b2Math.MulX(xf, this.m_vertices[i]);
         this.m_normals[i] = b2Math.MulMV(xf.R, this.m_normals[i]);
      }
   };

   /**
    * SetAsEdge
    *
    * @param v1
    * @param v2
    *
    */
   b2PolygonShape.prototype.SetAsEdge = function (v1, v2) {
      this.m_vertexCount = 2;
      this.Reserve(2);
      this.m_vertices[0].SetV(v1);
      this.m_vertices[1].SetV(v2);
      this.m_centroid.x = 0.5 * (v1.x + v2.x);
      this.m_centroid.y = 0.5 * (v1.y + v2.y);
      this.m_normals[0] = b2Math.CrossVF(b2Math.SubtractVV(v2, v1), 1.0);
      this.m_normals[0].Normalize();
      this.m_normals[1].x = (-this.m_normals[0].x);
      this.m_normals[1].y = (-this.m_normals[0].y);
   };

   /**
    * TestPoint
    *
    * @param xf
    * @param p
    *
    */
   b2PolygonShape.prototype.TestPoint = function (xf, p) {
      var tVec,
          tMat = xf.R,
          tX = p.x - xf.position.x,
          tY = p.y - xf.position.y,
          pLocalX = (tX * tMat.col1.x + tY * tMat.col1.y),
          pLocalY = (tX * tMat.col2.x + tY * tMat.col2.y);
      for (var i = 0; i < this.m_vertexCount; ++i) {
         tVec = this.m_vertices[i];
         tX = pLocalX - tVec.x;
         tY = pLocalY - tVec.y;
         tVec = this.m_normals[i];
         var dot = (tVec.x * tX + tVec.y * tY);
         if (dot > 0.0) {
            return false;
         }
      }
      return true;
   };

   /**
    * RayCast
    *
    * @param output
    * @param input
    * @param transform
    *
    */
   b2PolygonShape.prototype.RayCast = function (output, input, transform) {
      var lower = 0.0,
          upper = input.maxFraction,
          tX = 0,
          tY = 0,
          tMat,
          tVec;
      tX = input.p1.x - transform.position.x;
      tY = input.p1.y - transform.position.y;
      tMat = transform.R;
      var p1X = (tX * tMat.col1.x + tY * tMat.col1.y),
          p1Y = (tX * tMat.col2.x + tY * tMat.col2.y);
      tX = input.p2.x - transform.position.x;
      tY = input.p2.y - transform.position.y;
      tMat = transform.R;
      var p2X = (tX * tMat.col1.x + tY * tMat.col1.y),
          p2Y = (tX * tMat.col2.x + tY * tMat.col2.y),
          dX = p2X - p1X,
          dY = p2Y - p1Y,
          index = parseInt((-1));
      for (var i = 0; i < this.m_vertexCount; ++i) {
         tVec = this.m_vertices[i];
         tX = tVec.x - p1X;
         tY = tVec.y - p1Y;
         tVec = this.m_normals[i];
         var numerator = (tVec.x * tX + tVec.y * tY),
          denominator = (tVec.x * dX + tVec.y * dY);
         if (denominator === 0.0) {
            if (numerator < 0.0) {
               return false;
            }
         }
         else {
            if (denominator < 0.0 && numerator < lower * denominator) {
               lower = numerator / denominator;
               index = i;
            }
            else if (denominator > 0.0 && numerator < upper * denominator) {
               upper = numerator / denominator;
            }
         }
         if (upper < lower - b2Settings.b2_epsilon) {
            return false;
         }
      }
      if (index >= 0) {
         output.fraction = lower;
         tMat = transform.R;
         tVec = this.m_normals[index];
         output.normal.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
         output.normal.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         return true;
      }
      return false;
   };

   /**
    * ComputeAABB
    *
    * @param aabb
    * @param xf
    *
    */
   b2PolygonShape.prototype.ComputeAABB = function (aabb, xf) {
      var tMat = xf.R,
          tVec = this.m_vertices[0],
          lowerX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y),
          lowerY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y),
          upperX = lowerX,
          upperY = lowerY;
      for (var i = 1; i < this.m_vertexCount; ++i) {
         tVec = this.m_vertices[i];
         var vX = xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y),
          vY = xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
         lowerX = lowerX < vX ? lowerX : vX;
         lowerY = lowerY < vY ? lowerY : vY;
         upperX = upperX > vX ? upperX : vX;
         upperY = upperY > vY ? upperY : vY;
      }
      aabb.lowerBound.x = lowerX - this.m_radius;
      aabb.lowerBound.y = lowerY - this.m_radius;
      aabb.upperBound.x = upperX + this.m_radius;
      aabb.upperBound.y = upperY + this.m_radius;
   };

   /**
    * ComputeMass
    *
    * @param massData
    * @param density
    *
    */
   b2PolygonShape.prototype.ComputeMass = function (massData, density) {
      if (this.m_vertexCount === 2) {
         massData.center.x = 0.5 * (this.m_vertices[0].x + this.m_vertices[1].x);
         massData.center.y = 0.5 * (this.m_vertices[0].y + this.m_vertices[1].y);
         massData.mass = 0.0;
         massData.I = 0.0;
         return;
      }
      var centerX = 0.0,
          centerY = 0.0,
          area = 0.0,
          I = 0.0,
          p1X = 0.0,
          p1Y = 0.0,
          k_inv3 = 1.0 / 3.0;
      for (var i = 0; i < this.m_vertexCount; ++i) {
         var p2 = this.m_vertices[i],
          p3 = i + 1 < this.m_vertexCount ? this.m_vertices[parseInt(i + 1)] : this.m_vertices[0],
          e1X = p2.x - p1X,
          e1Y = p2.y - p1Y,
          e2X = p3.x - p1X,
          e2Y = p3.y - p1Y,
          D = e1X * e2Y - e1Y * e2X,
          triangleArea = 0.5 * D;area += triangleArea;
         centerX += triangleArea * k_inv3 * (p1X + p2.x + p3.x);
         centerY += triangleArea * k_inv3 * (p1Y + p2.y + p3.y);
         var px = p1X,
          py = p1Y,
          ex1 = e1X,
          ey1 = e1Y,
          ex2 = e2X,
          ey2 = e2Y,
          intx2 = k_inv3 * (0.25 * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2)) + 0.5 * px * px,
          inty2 = k_inv3 * (0.25 * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2)) + 0.5 * py * py;I += D * (intx2 + inty2);
      }
      massData.mass = density * area;
      centerX *= 1.0 / area;
      centerY *= 1.0 / area;
      massData.center.Set(centerX, centerY);
      massData.I = density * I;
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
   b2PolygonShape.prototype.ComputeSubmergedArea = function (normal, offset, xf, c) {
      offset = offset || 0;
      var normalL = b2Math.MulTMV(xf.R, normal),
          offsetL = offset - b2Math.Dot(normal, xf.position),
          depths = [],
          diveCount = 0,
          intoIndex = parseInt((-1)),
          outoIndex = parseInt((-1)),
          lastSubmerged = false,
          i = 0;
      for (i = 0;
      i < this.m_vertexCount; ++i) {
         depths[i] = b2Math.Dot(normalL, this.m_vertices[i]) - offsetL;
         var isSubmerged = depths[i] < (-b2Settings.b2_epsilon);
         if (i > 0) {
            if (isSubmerged) {
               if (!lastSubmerged) {
                  intoIndex = i - 1;
                  diveCount++;
               }
            }
            else {
               if (lastSubmerged) {
                  outoIndex = i - 1;
                  diveCount++;
               }
            }
         }
         lastSubmerged = isSubmerged;
      }
      switch (diveCount) {
      case 0:
         if (lastSubmerged) {
            var md = new b2MassData();
            this.ComputeMass(md, 1);
            c.SetV(b2Math.MulX(xf, md.center));
            return md.mass;
         }
         else {
            return 0;
         }
         break;
      case 1:
         if (intoIndex === (-1)) {
            intoIndex = this.m_vertexCount - 1;
         }
         else {
            outoIndex = this.m_vertexCount - 1;
         }
         break;
      }
      var intoIndex2 = parseInt((intoIndex + 1) % this.m_vertexCount),
          outoIndex2 = parseInt((outoIndex + 1) % this.m_vertexCount),
          intoLamdda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]),
          outoLamdda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]),
          intoVec = new b2Vec2(this.m_vertices[intoIndex].x * (1 - intoLamdda) + this.m_vertices[intoIndex2].x * intoLamdda, this.m_vertices[intoIndex].y * (1 - intoLamdda) + this.m_vertices[intoIndex2].y * intoLamdda),
          outoVec = new b2Vec2(this.m_vertices[outoIndex].x * (1 - outoLamdda) + this.m_vertices[outoIndex2].x * outoLamdda, this.m_vertices[outoIndex].y * (1 - outoLamdda) + this.m_vertices[outoIndex2].y * outoLamdda),
          area = 0,
          center = new b2Vec2(0, 0),
          p2 = this.m_vertices[intoIndex2],
          p3;
      i = intoIndex2;
      while (i !== outoIndex2) {
         i = (i + 1) % this.m_vertexCount;
         if (i === outoIndex2) p3 = outoVec;
         else p3 = this.m_vertices[i];
         var triangleArea = 0.5 * ((p2.x - intoVec.x) * (p3.y - intoVec.y) - (p2.y - intoVec.y) * (p3.x - intoVec.x));
         area += triangleArea;
         center.x += triangleArea * (intoVec.x + p2.x + p3.x) / 3;
         center.y += triangleArea * (intoVec.y + p2.y + p3.y) / 3;
         p2 = p3;
      }
      center.Multiply(1 / area);
      c.SetV(b2Math.MulX(xf, center));
      return area;
   };

   /**
    * GetVertexCount
    *
    * @param 
    *
    */
   b2PolygonShape.prototype.GetVertexCount = function () {
      return this.m_vertexCount;
   };

   /**
    * GetVertices
    *
    * @param 
    *
    */
   b2PolygonShape.prototype.GetVertices = function () {
      return this.m_vertices;
   };

   /**
    * GetNormals
    *
    * @param 
    *
    */
   b2PolygonShape.prototype.GetNormals = function () {
      return this.m_normals;
   };

   /**
    * GetSupport
    *
    * @param d
    *
    */
   b2PolygonShape.prototype.GetSupport = function (d) {
      var bestIndex = 0,
          bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
      for (var i = 1; i < this.m_vertexCount; ++i) {
         var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
         if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
         }
      }
      return bestIndex;
   };

   /**
    * GetSupportVertex
    *
    * @param d
    *
    */
   b2PolygonShape.prototype.GetSupportVertex = function (d) {
      var bestIndex = 0,
          bestValue = this.m_vertices[0].x * d.x + this.m_vertices[0].y * d.y;
      for (var i = 1; i < this.m_vertexCount; ++i) {
         var value = this.m_vertices[i].x * d.x + this.m_vertices[i].y * d.y;
         if (value > bestValue) {
            bestIndex = i;
            bestValue = value;
         }
      }
      return this.m_vertices[bestIndex];
   };

   /**
    * Validate
    *
    * @param 
    *
    */
   b2PolygonShape.prototype.Validate = function () {
      return false;
   };

   /**
    * Reserve
    *
    * @param count
    *
    */
   b2PolygonShape.prototype.Reserve = function (count) {
      count = count || 0;
      for (var i = parseInt(this.m_vertices.length); i < count; i++) {
         this.m_vertices[i] = new b2Vec2(0, 0);
         this.m_normals[i] = new b2Vec2(0, 0);
      }
   };

   /**
    * GetType
    *
    * @param 
    *
    */
   b2PolygonShape.prototype.GetType = function () {
      return this.m_type;
   };

   /**
    * b2Shape
    *
    * @param 
    *
    */
   b2PolygonShape.prototype.b2Shape = function () {
      this.m_type = b2Shape.e_unknownShape;
      this.m_radius = b2Settings.b2_linearSlop;
   };