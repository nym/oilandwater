/*
 * Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
var b2PolygonShape = (function() {
    function b2PolygonShape() {
        this.m_centroid = new b2Vec2(0, 0);
        this.m_vertices = [];
        this.m_normals = [];
    }

    b2PolygonShape.constructor = b2PolygonShape;
    b2PolygonShape.prototype = Object.create(b2Shape);

    b2PolygonShape.ComputeCentroid = function(){};

    /// Implement b2Shape.
    b2PolygonShape.prototype.Clone = function() {
        var clone = new b2PolygonShape();
        clone.m_p.x = this.m_p.x;
        clone.m_p.y = this.m_p.y;
        clone.m_radius = this.m_radius;
        return clone;
    };

    /// Copy vertices. This assumes the vertices define a convex polygon.
    /// It is assumed that the exterior is the the right of each edge.
    b2PolygonShape.prototype.Set = function(vertices, count) {
        var i;
        b2Assert(2 <= count && count <= b2_maxPolygonVertices);
        this.m_vertexCount = count;

        // Copy vertices.
        for (i = 0; i < this.m_vertexCount; ++i) {
            this.m_vertices[i] = this.vertices[i];
        }

        // Compute normals. Ensure the edges have non-zero length.
        for (i = 0; i < this.m_vertexCount; ++i)
        {
            var i1 = i;
            var i2 = i + 1 < this.m_vertexCount ? i + 1 : 0;
            var edge = b2Sub.b2Vec2(this.m_vertices[i2], this.m_vertices[i1]);
            b2Assert(edge.LengthSquared() > b2_epsilon * b2_epsilon);
            b2Cross.b2Vec2_float(this.m_normals[i], edge, 1);
            this.m_normals[i].Normalize();
        }

        // Compute the polygon centroid.
        this.m_centroid = b2PolygonShape.ComputeCentroid(this.m_vertices, this.m_vertexCount);

    };

    /// Build vertices to represent an axis-aligned box.
    /// @param hx the half-width.
    /// @param hy the half-height.
    /// @param center the center of the box in local coordinates.
    /// @param angle the rotation of the box in local coordinates.
    b2PolygonShape.prototype.SetAsBox = function(hx, hy, center, angle) {
        this.m_vertexCount = 4;
        this.m_vertices[0].Set(-hx, -hy);
        this.m_vertices[1].Set( hx, -hy);
        this.m_vertices[2].Set( hx,  hy);
        this.m_vertices[3].Set(-hx,  hy);
        this.m_normals[0].Set(0, -1);
        this.m_normals[1].Set(1, 0);
        this.m_normals[2].Set(0, 1);
        this.m_normals[3].Set(-1, 0);
        if (!center) {
            this.m_centroid.SetZero();
        } else {
            var xf = new b2Transform();
            xf.position.Set(center.x, center.y);
            xf.R.SetAngle(angle);

            // Transform vertices and normals.
            for (var i = 0; i < this.m_vertexCount; ++i) {
                b2Mul.b2Transform_b2Vec2(xf, this.m_vertices[i], this.m_vertices[i]);
                b2Mul.b2Mat22_b2Vec2(xf.R, this.m_normals[i], this.m_normals[i]);
            }
        }
    };

    /// Set this as a single edge.
    b2PolygonShape.prototype.SetAsEdge = function(v1, v2) {

        this.m_vertexCount = 2;
        this.m_vertices[0].Copy(v1);
        this.m_vertices[1].Copy(v2);
        b2Add.b2Vec2(v1, v2, m_centroid).Mul(0.5);
        b2Cross.b2Vec2_float(b2Sub.b2Vec2(v2 ,v1), 1, this.m_normals[0]);
        this.m_normals[0].Normalize();
        this.m_normals[1].Copy(this.m_normals[0].Neg());

    };


    /// @see b2Shape::TestPoint
    b2PolygonShape.prototype.TestPoint = function(xf, p) {
        var pLocal = b2MulT.b2Mat22_b2Vec2(xf.R, b2Sub.b2Vecs(p, xf.position));

        for (var i = 0; i < this.m_vertexCount; ++i) {
            var dot = b2Dot.b2Vec2(this.m_normals[i], b2Sub.b2Vec2(pLocal, this.m_vertices[i]));
            if (dot > 0) {
                return false;
            }
        }

        return true;

    };

    /// Implement b2Shape.
    b2PolygonShape.prototype.RayCast = function(output, input, xf) {
        // Put the ray into the polygon's frame of reference.
        var p1 = b2MulT.b2Mat22_b2Vec2(xf.R, b2Sub.b2Vec2(input.p1, xf.position));
        var p2 = b2MulT.b2Mat22_b2Vec2(xf.R, b2Sub.b2Vec2(input.p2, xf.position));
        var d = b2Sub.b2Vec2(p2, p1);

        if (this.m_vertexCount == 2)
        {
            var v1 = this.m_vertices[0];
            var v2 = this.m_vertices[1];
            var normal = this.m_normals[0];

            // q = p1 + t * d
            // dot(normal, q - v1) = 0
            // dot(normal, p1 - v1) + t * dot(normal, d) = 0

            var numerator = b2Dot.b2Vec2(normal, b2Sub.b2Vec2(v1, p1));
            var denominator = b2Dot.b2Vec2(normal, d);

            if (denominator === 0) {
                return false;
            }

            var t = numerator / denominator;
            if (t < 0 || 1 < t) {
                return false;
            }

            //var q = p1 + t * d;
            var q = b2Vec2.Copy(d).Mul(t).Add(p1);

            // q = v1 + s * r
            // s = dot(q - v1, r) / dot(r, r)
            var r = b2Sub.b2Vec2(v2, v1);
            var rr = b2Dot.b2Vec2(r, r);
            if (rr === 0) {
                return false;
            }

            var s = b2Dot.b2Vec2(b2Sub.b2Vec2(q, v1), r) / rr;
            if (s < 0 || 1 < s) {
                return false;
            }

            output.fraction = t;
            if (numerator > 0) {
                output.normal.Copy(normal).Neg();
            } else {
                output.normal.Copy(normal);
            }
            return true;
        } else {
            var lower = 0, upper = input.maxFraction;

            var index = -1;

            for (var i = 0; i < this.m_vertexCount; ++i) {
                // p = p1 + a * d
                // dot(normal, p - v) = 0
                // dot(normal, p1 - v) + a * dot(normal, d) = 0

                numerator = b2Dot.b2Vec2(this.m_normals[i], b2Sub.b2Vec2(this.m_vertices[i], p1));
                denominator = b2Dot.b2Vec2(this.m_normals[i], d);

                if (denominator === 0) {
                    if (numerator < 0){
                        return false;
                    }
                } else {
                    // Note: we want this predicate without division:
                    // lower < numerator / denominator, where denominator < 0
                    // Since denominator < 0, we have to flip the inequality:
                    // lower < numerator / denominator <==> denominator * lower > numerator.
                    if (denominator < 0 && numerator < lower * denominator) {
                        // Increase lower.
                        // The segment enters this half-space.
                        lower = numerator / denominator;
                        index = i;
                    } else if (denominator > 0 && numerator < upper * denominator){
                        // Decrease upper.
                        // The segment exits this half-space.
                        upper = numerator / denominator;
                    }
                }

                // The use of epsilon here causes the assert on lower to trip
                // in some cases. Apparently the use of epsilon was to make edge
                // shapes work, but now those are handled separately.
                //if (upper < lower - b2_epsilon)
                if (upper < lower) {
                    return false;
                }
            }

            b2Assert(0 <= lower && lower <= input.maxFraction);

            if (index >= 0) {
                output.fraction = lower;
                output.normal.Copy(b2Mul.b2Mat22_b2Vec2(xf.R, this.m_normals[index]));
                return true;
            }
        }

        return false;

    };

    /// @see b2Shape::ComputeAABB
    b2PolygonShape.prototype.ComputeAABB = function(aabb, xf) {
        var lower = b2Mul.b2Transform_b2Vec2(xf, this,m_vertices[0], aabb.lowerBound);
        var upper = aabb.upperBound.Copy(lower);

        var v = new b2Vec2();

        for (var i = 1; i < this.m_vertexCount; ++i) {
            v = b2Mul.b2Transform_b2Vec2(xf, this.m_vertices[i], v);
            b2Min.b2Vec2(lower, lower, v);
            b2Max.b2Vec2(upper, upper, v);
        }

        var r = v.Set(m_radius, m_radius);

        b2Sub.b2Vec2(aabb.lowerBound, lower, r);
        b2Add.b2Vec2(aabb.upperBound, upper, r);

    };

    /// @see b2Shape::ComputeMass
    b2PolygonShape.prototype.ComputeMass = function(massData, density) {
        // Polygon mass, centroid, and inertia.
        // Let rho be the polygon density in mass per unit area.
        // Then:
        // mass = rho * int(dA)
        // centroid.x = (1/mass) * rho * int(x * dA)
        // centroid.y = (1/mass) * rho * int(y * dA)
        // I = rho * int((x*x + y*y) * dA)
        //
        // We can compute these integrals by summing all the integrals
        // for each triangle of the polygon. To evaluate the integral
        // for a single triangle, we make a change of variables to
        // the (u,v) coordinates of the triangle:
        // x = x0 + e1x * u + e2x * v
        // y = y0 + e1y * u + e2y * v
        // where 0 <= u && 0 <= v && u + v <= 1.
        //
        // We integrate u from [0,1-v] and then v from [0,1].
        // We also need to use the Jacobian of the transformation:
        // D = cross(e1, e2)
        //
        // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
        //
        // The rest of the derivation is handled by computer algebra.

        b2Assert(this.m_vertexCount >= 2);

        // A line segment has zero mass.
        if (this.m_vertexCount === 2)
        {

            b2Add.b2Vec2(this.m_vertices[0], this.m_vertices[1], massData.center).Mul(0.5);
            massData.mass = 0;
            massData.I = 0;
            return;
        }

        var center = new b2Vec2();
        var area = 0;
        var I = 0;

        // pRef is the reference point for forming triangles.
        // It's location doesn't change the result (except for rounding error).
        var pRef = new b2Vec2();
        var k_inv3 = 1 / 3;

        for (var i = 0; i < this.m_vertexCount; ++i)
        {
            // Triangle vertices.
            var p1 = pRef;
            var p2 = this.m_vertices[i];
            var p3 = i + 1 < this.m_vertexCount ? this.m_vertices[i+1] : this.m_vertices[0];

            var e1 = p2 - p1;
            var e2 = p3 - p1;

            var D = b2Cross.b2Vec2(e1, e2);

            var triangleArea = 0.5 * D;
            area += triangleArea;

            // Area weighted centroid
            center.Add(p1).Add(p2).Add(p3).Mul(triangleArea * k_inv3);

            var px = p1.x, py = p1.y;
            var ex1 = e1.x, ey1 = e1.y;
            var ex2 = e2.x, ey2 = e2.y;

            var intx2 = k_inv3 * (0.25 * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5*px*px;
            var inty2 = k_inv3 * (0.25 * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5*py*py;

            I += D * (intx2 + inty2);
        }

        // Total mass
        massData.mass = density * area;

        // Center of mass
        b2Assert(area > b2_epsilon);
        center.Mul(1 / area);
        massData.center.Copy(center);

        // Inertia tensor relative to the local origin.
        massData.I = density * I;

    };

    /// Get the supporting vertex index in the given direction.
    b2PolygonShape.prototype.GetSupport = function(d)  {
        var bestIndex = 0;
        var bestValue = b2Dot.b2Vec2(this.m_vertices[0], d);
        for (var i = 1; i < this.m_vertexCount; ++i) {
            var value = b2Dot.b2Vec2(this.m_vertices[i], d);
            if (value > bestValue) {
                bestIndex = i;
                bestValue = value;
            }
        }

        return bestIndex;
    };

    /// Get the supporting vertex in the given direction.
    b2PolygonShape.prototype.GetSupportVertex = function(d) {
        var bestIndex = 0;
        var bestValue = b2Dot.b2Vec2(this.m_vertices[0], d);
        for (var i = 1; i < this.m_vertexCount; ++i)
        {
            var value = b2Dot.b2Vec2(this.m_vertices[i], d);
            if (value > bestValue)
            {
                bestIndex = i;
                bestValue = value;
            }
        }

        return m_vertices[bestIndex];
    };

    /// Get the vertex count.
    b2PolygonShape.prototype.GetVertexCount = function() {
        return this.m_vertexCount;
    };
    /// Get a vertex by index. Used by b2Distance.
    b2PolygonShape.prototype.GetVertex = function(index) {
        b2Assert(0 <= index && index < this.m_vertexCount);
        return this.m_vertices[index];

    };


    b2PolygonShape.prototype.m_radius = b2_polygonRadius;
    b2PolygonShape.prototype.m_vertexCount = 0;
    b2PolygonShape.prototype.m_type = e_polygon;
    b2PolygonShape.prototype.m_centroid = null;
    b2PolygonShape.prototype.m_vertices = null;`
    b2PolygonShape.prototype.m_normals = null;
    b2PolygonShape.prototype.m_vertexCount = 0;

    return b2PolygonShape;
})();

