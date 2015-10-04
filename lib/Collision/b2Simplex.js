
   /**
    *  Class b2Simplex
    *
    * @param 
    *
    */
   b2Simplex = Box2D.Collision.b2Simplex = function b2Simplex() {
      this.m_v1 = new b2SimplexVertex();
      this.m_v2 = new b2SimplexVertex();
      this.m_v3 = new b2SimplexVertex();

      this.m_vertices = [this.m_v1, this.m_v2, this.m_v3];
   };
   b2Simplex.constructor = b2Simplex;

   b2Simplex.prototype.m_v1              = null;
   b2Simplex.prototype.m_v2              = null;
   b2Simplex.prototype.m_v3              = null;
   b2Simplex.prototype.m_vertices        = null;
   b2Simplex.prototype.m_count           = 0;


   /**
    * ReadCache
    *
    * @param cache
    * @param proxyA
    * @param transformA
    * @param proxyB
    * @param transformB
    *
    */
   b2Simplex.prototype.ReadCache = function (cache, proxyA, transformA, proxyB, transformB) {
      b2Assert(0 <= cache.count && cache.count <= 3);
      var wALocal,
          wBLocal;
      this.m_count = cache.count;
      var vertices = this.m_vertices;
      for (var i = 0; i < this.m_count; i++) {
         var v = vertices[i];
         v.indexA = cache.indexA[i];
         v.indexB = cache.indexB[i];
         wALocal = proxyA.GetVertex(v.indexA);
         wBLocal = proxyB.GetVertex(v.indexB);
         v.wA = b2Math.MulX(transformA, wALocal);
         v.wB = b2Math.MulX(transformB, wBLocal);
         v.w = b2Math.SubtractVV(v.wB, v.wA);
         v.a = 0;
      }
      if (this.m_count > 1) {
         var metric1 = cache.metric,
          metric2 = this.GetMetric();
         if (metric2 < 0.5 * metric1 || 2.0 * metric1 < metric2 || metric2 < b2Settings.b2_epsilon) {
            this.m_count = 0;
         }
      }
      if (this.m_count === 0) {
         v = vertices[0];
         v.indexA = 0;
         v.indexB = 0;
         wALocal = proxyA.GetVertex(0);
         wBLocal = proxyB.GetVertex(0);
         v.wA = b2Math.MulX(transformA, wALocal);
         v.wB = b2Math.MulX(transformB, wBLocal);
         v.w = b2Math.SubtractVV(v.wB, v.wA);
         this.m_count = 1;
      }
   };

   /**
    * WriteCache
    *
    * @param cache
    *
    */
   b2Simplex.prototype.WriteCache = function (cache) {
      cache.metric = this.GetMetric();
      cache.count = Math.abs(this.m_count);
      var vertices = this.m_vertices;
      for (var i = 0; i < this.m_count; i++) {
         cache.indexA[i] = Math.abs(vertices[i].indexA);
         cache.indexB[i] = Math.abs(vertices[i].indexB);
      }
   };

   /**
    * GetSearchDirection
    *
    * @param 
    *
    */
   b2Simplex.prototype.GetSearchDirection = function () {
      switch (this.m_count) {
      case 1:
         return this.m_v1.w.GetNegative();
      case 2:
         {
            var e12 = b2Math.SubtractVV(this.m_v2.w, this.m_v1.w),
          sgn = b2Math.CrossVV(e12, this.m_v1.w.GetNegative());
            if (sgn > 0.0) {
               return b2Math.CrossFV(1.0, e12);
            }
            else {
               return b2Math.CrossVF(e12, 1.0);
            }
         }
      default:
         b2Assert(false);
         return new b2Vec2(0, 0);
      }
   };

   /**
    * GetClosestPoint
    *
    * @param 
    *
    */
   b2Simplex.prototype.GetClosestPoint = function () {
      switch (this.m_count) {
      case 0:
         b2Assert(false);
         return new b2Vec2(0, 0);
      case 1:
         return this.m_v1.w;
      case 2:
         return new b2Vec2(this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x, this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y);
      default:
         b2Assert(false);
         return new b2Vec2(0, 0);
      }
   };

   /**
    * GetWitnessPoints
    *
    * @param pA
    * @param pB
    *
    */
   b2Simplex.prototype.GetWitnessPoints = function (pA, pB) {
      switch (this.m_count) {
      case 0:
         b2Assert(false);
         break;
      case 1:
         pA.SetV(this.m_v1.wA);
         pB.SetV(this.m_v1.wB);
         break;
      case 2:
         pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
         pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
         pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
         pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
         break;
      case 3:
         pB.x = pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x + this.m_v3.a * this.m_v3.wA.x;
         pB.y = pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y + this.m_v3.a * this.m_v3.wA.y;
         break;
      default:
         b2Assert(false);
         break;
      }
   };

   /**
    * GetMetric
    *
    * @param 
    *
    */
   b2Simplex.prototype.GetMetric = function () {
      switch (this.m_count) {
      case 0:
         b2Assert(false);
         return 0.0;
      case 1:
         return 0.0;
      case 2:
         return b2Math.SubtractVV(this.m_v1.w, this.m_v2.w).Length();
      case 3:
         return b2Math.CrossVV(b2Math.SubtractVV(this.m_v2.w, this.m_v1.w), b2Math.SubtractVV(this.m_v3.w, this.m_v1.w));
      default:
         b2Assert(false);
         return 0.0;
      }
   };

   /**
    * Solve2
    *
    * @param 
    *
    */
   b2Simplex.prototype.Solve2 = function () {
      var w1 = this.m_v1.w,
          w2 = this.m_v2.w,
          e12 = b2Math.SubtractVV(w2, w1),
          d12_2 = (-(w1.x * e12.x + w1.y * e12.y));
      if (d12_2 <= 0.0) {
         this.m_v1.a = 1.0;
         this.m_count = 1;
         return;
      }
      var d12_1 = (w2.x * e12.x + w2.y * e12.y);
      if (d12_1 <= 0.0) {
         this.m_v2.a = 1.0;
         this.m_count = 1;
         this.m_v1.Set(this.m_v2);
         return;
      }
      var inv_d12 = 1.0 / (d12_1 + d12_2);
      this.m_v1.a = d12_1 * inv_d12;
      this.m_v2.a = d12_2 * inv_d12;
      this.m_count = 2;
   };

   /**
    * Solve3
    *
    * @param 
    *
    */
   b2Simplex.prototype.Solve3 = function () {
      var w1 = this.m_v1.w,
          w2 = this.m_v2.w,
          w3 = this.m_v3.w,
          e12 = b2Math.SubtractVV(w2, w1),
          w1e12 = b2Math.Dot(w1, e12),
          w2e12 = b2Math.Dot(w2, e12),
          d12_1 = w2e12,
          d12_2 = (-w1e12),
          e13 = b2Math.SubtractVV(w3, w1),
          w1e13 = b2Math.Dot(w1, e13),
          w3e13 = b2Math.Dot(w3, e13),
          d13_1 = w3e13,
          d13_2 = (-w1e13),
          e23 = b2Math.SubtractVV(w3, w2),
          w2e23 = b2Math.Dot(w2, e23),
          w3e23 = b2Math.Dot(w3, e23),
          d23_1 = w3e23,
          d23_2 = (-w2e23),
          n123 = b2Math.CrossVV(e12, e13),
          d123_1 = n123 * b2Math.CrossVV(w2, w3),
          d123_2 = n123 * b2Math.CrossVV(w3, w1),
          d123_3 = n123 * b2Math.CrossVV(w1, w2);
      if (d12_2 <= 0.0 && d13_2 <= 0.0) {
         this.m_v1.a = 1.0;
         this.m_count = 1;
         return;
      }
      if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0) {
         var inv_d12 = 1.0 / (d12_1 + d12_2);
         this.m_v1.a = d12_1 * inv_d12;
         this.m_v2.a = d12_2 * inv_d12;
         this.m_count = 2;
         return;
      }
      if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0) {
         var inv_d13 = 1.0 / (d13_1 + d13_2);
         this.m_v1.a = d13_1 * inv_d13;
         this.m_v3.a = d13_2 * inv_d13;
         this.m_count = 2;
         this.m_v2.Set(this.m_v3);
         return;
      }
      if (d12_1 <= 0.0 && d23_2 <= 0.0) {
         this.m_v2.a = 1.0;
         this.m_count = 1;
         this.m_v1.Set(this.m_v2);
         return;
      }
      if (d13_1 <= 0.0 && d23_1 <= 0.0) {
         this.m_v3.a = 1.0;
         this.m_count = 1;
         this.m_v1.Set(this.m_v3);
         return;
      }
      if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0) {
         var inv_d23 = 1.0 / (d23_1 + d23_2);
         this.m_v2.a = d23_1 * inv_d23;
         this.m_v3.a = d23_2 * inv_d23;
         this.m_count = 2;
         this.m_v1.Set(this.m_v3);
         return;
      }
      var inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
      this.m_v1.a = d123_1 * inv_d123;
      this.m_v2.a = d123_2 * inv_d123;
      this.m_v3.a = d123_3 * inv_d123;
      this.m_count = 3;
   };