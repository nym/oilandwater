
   /**
    *  Class b2Distance
    *
    * @param 
    *
    */
   b2Distance = Box2D.Collision.b2Distance = function b2Distance() {};
   b2Distance.constructor = b2Distance;

   b2Distance.s_simplex = new b2Simplex();
   b2Distance.s_saveA = [0, 0, 0];
   b2Distance.s_saveB = [0, 0, 0];

   /**
    * Static Distance
    *
    * @param output
    * @param cache
    * @param input
    *
    */
   b2Distance.Distance = function (output, cache, input) {
      //++b2Distance.b2_gjkCalls;
      var proxyA = input.proxyA,
          proxyB = input.proxyB,
          transformA = input.transformA,
          transformB = input.transformB,
          simplex = b2Distance.s_simplex;
      simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
      var vertices = simplex.m_vertices,
          k_maxIters = 20,
          saveA = b2Distance.s_saveA,
          saveB = b2Distance.s_saveB,
          saveCount = 0,
          closestPoint = simplex.GetClosestPoint(),
          distanceSqr1 = closestPoint.LengthSquared(),
          distanceSqr2 = distanceSqr1,
          i = 0,
          p,
          iter = 0;
      while (iter < k_maxIters) {
         saveCount = simplex.m_count;
         for (i = 0; i < saveCount; i++) {
            saveA[i] = vertices[i].indexA;
            saveB[i] = vertices[i].indexB;
         }
         switch (simplex.m_count) {
         case 1:
            break;
         case 2:
            simplex.Solve2();
            break;
         case 3:
            simplex.Solve3();
            break;
         default:
            b2Assert(false);
         }
         if (simplex.m_count === 3) {
            break;
         }
         p = simplex.GetClosestPoint();
         distanceSqr2 = p.LengthSquared();
         if (distanceSqr2 > distanceSqr1) {
             //break;
         }
         distanceSqr1 = distanceSqr2;
         var d = simplex.GetSearchDirection();
         if (d.LengthSquared() < b2Settings.b2_epsilon * b2Settings.b2_epsilon) {
            break;
         }
         var vertex = vertices[simplex.m_count];
         vertex.indexA = proxyA.GetSupport(b2Math.MulTMV(transformA.R, d.GetNegative()));
         vertex.wA = b2Math.MulX(transformA, proxyA.GetVertex(vertex.indexA));
         vertex.indexB = proxyB.GetSupport(b2Math.MulTMV(transformB.R, d));
         vertex.wB = b2Math.MulX(transformB, proxyB.GetVertex(vertex.indexB));
         vertex.w = b2Math.SubtractVV(vertex.wB, vertex.wA);
         ++iter;
         ++b2Distance.b2_gjkIters;
         var duplicate = false;
         for (i = 0; i < saveCount; i++) {
            if (vertex.indexA === saveA[i] && vertex.indexB === saveB[i]) {
               duplicate = true;
               break;
            }
         }
         if (duplicate) {
            break;
         }++simplex.m_count;
      }
      b2Distance.b2_gjkMaxIters = b2Math.Max(b2Distance.b2_gjkMaxIters, iter);
      simplex.GetWitnessPoints(output.pointA, output.pointB);
      output.distance = b2Math.SubtractVV(output.pointA, output.pointB).Length();
      output.iterations = iter;
      simplex.WriteCache(cache);
      if (input.useRadii) {
         var rA = proxyA.m_radius,
          rB = proxyB.m_radius;
         if (output.distance > rA + rB && output.distance > b2Settings.b2_epsilon) {
            output.distance -= rA + rB;
            var normal = b2Math.SubtractVV(output.pointB, output.pointA);
            normal.Normalize();
            output.pointA.x += rA * normal.x;
            output.pointA.y += rA * normal.y;
            output.pointB.x -= rB * normal.x;
            output.pointB.y -= rB * normal.y;
         }
         else {
            p = new b2Vec2(0, 0);
            p.x = .5 * (output.pointA.x + output.pointB.x);
            p.y = .5 * (output.pointA.y + output.pointB.y);
            output.pointA.x = output.pointB.x = p.x;
            output.pointA.y = output.pointB.y = p.y;
            output.distance = 0.0;
         }
      }
   };