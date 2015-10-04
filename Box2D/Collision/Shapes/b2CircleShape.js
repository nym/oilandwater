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
var b2CircleShape = (function(){

    function b2CircleShape() {
        this.m_p = new b2Vec2(0, 0);
    }

    b2CircleShape.constructor = b2CircleShape;
    b2CircleShape.prototype = Object.create(b2Shape);

    /// Implement b2Shape.
    b2CircleShape.prototype.Clone = function() {
        var clone = new b2CircleShape();
        clone.m_p.x = this.m_p.x;
        clone.m_p.y = this.m_p.y;
        clone.m_radius = this.m_radius;
        return clone;
    };

    /// Implement b2Shape.
    b2CircleShape.prototype.TestPoint = function(transform, p) {

        var center = b2Mul.b2Transform_b2Vec2(transform, this.m_p);
        var d = b2Sub.b2Vec2(p, center);
        return b2Dot.b2Vec2(d, d) <= this.m_radius * this.m_radius;

    };

    // Collision Detection in Interactive 3D Environments by Gino van den Bergen
    // From Section 3.1.2
    // x = s + a * r
    // norm(x) = radius
    b2CircleShape.prototype.RayCast = function(output, input, transform) {

        var position = b2Mul.b2Transform_b2Vec2(transform, this.m_p);
        var s = b2Sub.b2Vec2(input.p1, position);
        var b = b2Dot.b2Vec2(s, s) - this.m_radius * this.m_radius;

        // Solve quadratic equation.
        var r = b2Sub.b2Vec2(input.p2, input.p1);
        var c =  b2Dot.b2Vec2(s, r);
        var rr = b2Dot.b2Vec2(r, r);
        var sigma = c * c - rr * b;

        // Check for negative discriminant and short segment.
        if (sigma < 0 || rr < b2_epsilon) {
            return false;
        }

        // Find the point of intersection of the line with the circle.
        var a = -(c + b2Sqrt(sigma));

        // Is the intersection point on the segment?
        if (0 <= a && a <= input.maxFraction * rr) {
            a /= rr;
            output.fraction = a;
            b2Mul.b2Vec2(a, r, output.normal).Add(s).Normalize();
            return true;
        }

        return false;

    };

    /// @see b2Shape::ComputeAABB
    b2CircleShape.prototype.ComputeAABB = function(aabb, transform) {

        var p = b2Mul.b2Transform_b2Vec2(transform, this.m_p);
        aabb.lowerBound.Set(p.x - this.m_radius, p.y - this.m_radius);
        aabb.upperBound.Set(p.x + this.m_radius, p.y + this.m_radius);
    };

    /// @see b2Shape::ComputeMass
    b2CircleShape.prototype.ComputeMass = function(massData, density) {
        massData.mass = density * b2_pi * this.m_radius * this.m_radius;
        massData.center = this.m_p;

        // inertia about the local origin
        massData.I = massData.mass * (0.5 * this.m_radius * this.m_radius + b2Dot.b2Vec2(this.m_p, this.m_p));

    };

    /// Get the supporting vertex index in the given direction.
    b2CircleShape.prototype.GetSupport = function(d)  {
        return 0;
    };

    /// Get the supporting vertex in the given direction.
    b2CircleShape.prototype.GetSupportVertex = function(d) {
        return this.m_p;
    };

    /// Get the vertex count.
    b2CircleShape.prototype.GetVertexCount = function() {
        return 1;
    };
    /// Get a vertex by index. Used by b2Distance.
    b2CircleShape.prototype.GetVertex = function(index) {
        b2Assert(index === 0);
        return this.m_p;

    };
    b2CircleShape.prototype.m_type = e_circle;
    b2CircleShape.prototype.m_radius = 0;
    /// Position
    b2CircleShape.prototype.m_p = null;

    return b2CircleShape;
})();
