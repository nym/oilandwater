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

//uint8 referenceEdge;	///< The edge that defines the outward contact normal.
//uint8 incidentEdge;		///< The edge most anti-parallel to the reference edge.
//uint8 incidentVertex;	///< The vertex (0 or 1) on the incident edge that was clipped.
//uint8 flip;				///< A value of 1 indicates that the reference edge is on shape2.
function b2ContactID() {}
b2ContactID.prototype = {
    referenceEdge: 0,       ///< The edge that defines the outward contact normal.
    incidentEdge: 0,        ///< The edge most anti-parallel to the reference edge.
    incidentVertex: 0,      ///< The vertex (0 or 1) on the incident edge that was clipped.
    flip: 0                 ///< A value of 1 indicates that the reference edge is on shape2.
};
Object.defineProperties(b2ContactID.prototype, {
    key: {                  ///< Used to quickly compare contact ids.
        get: function()  {
            var key = this.referenceEdge;
            key = (key << 8) | this.incidentEdge;
            key = (key << 8) | this.incidentVertex;
            key = (key << 8) | this.flip;
            return key;

        }
    }
});
/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
function b2ManifoldPoint() {
    this.localPoint = new b2Vec2(0,0);
    this.id = new b2ContactID();
}
b2ManifoldPoint.constructor = b2ManifoldPoint;
b2ManifoldPoint.prototype = {
    localPoint: null,		///< usage depends on manifold type
    normalImpulse: 0,	    ///< the non-penetration impulse
    tangentImpulse: 0,      ///< the friction impulse
    id: 0		            ///< uniquely identifies a contact point between two shapes
};
/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
function b2Manifold() {
    this.points = [];
    this.localNormal = new b2Vec2(0, 0);
    this.localPoint = new b2Vec2(0, 0);

}
b2Manifold.e_circles = 1;
b2Manifold.e_faceA = 2;
b2Manifold.e_faceB = 3;
b2Manifold.constructor = b2Manifold;
    b2Manifold.prototype = {
    points: null,	        ///< the points of contact
    localNormal: null, 	    ///< not use for Type::e_points
    localPoint: null,  	    ///< usage depends on manifold type
    type: 0,
    pointCount: 0			///< the number of manifold points
};

/// This is used to compute the current state of a contact manifold.
function b2WorldManifold() {
    this.points = [];
    this.normal = new b2Vec2(0, 0);
}
b2WorldManifold.constructor = b2WorldManifold;
b2WorldManifold.prototype = {
    /// Evaluate the manifold with supplied transforms. This assumes
    /// modest motion from the original state. This does not change the
    /// point count, impulses, etc. The radii must come from the shapes
    /// that generated the manifold.
    Initialize: function(manifold, xfA, radiusA, xfB, radiusB) {
        var i, cA, cB, clipPoint, planePoint;

        if (manifold.pointCount === 0) {
            return;
        }

        switch (manifold.type) {
            case b2Manifold.e_circles:
                this.normal.Set(1, 0);
                var pointA = b2Mul.b2Vec2(xfA, manifold.localPoint);
                var pointB = b2Mul.b2Vec2(xfB, manifold.points[0].localPoint);
                if (b2DistanceSquared(pointA, pointB) > b2_epsilon * b2_epsilon) {
                    b2Sub.b2Vec2(pointB, pointA, this.normal);
                    this.normal.Normalize();
                }

                cA = b2Vec2.Clone(this.normal).Mul(radiusA).Add(pointA);
                cB = b2Vec2.Clone(this.normal).Mul(radiusB).Add(pointB);

                b2Add.b2Vec2(cA, cB, this.points[0]).Mul(0.5);

                break;

            case b2Manifold.e_faceA:
                b2Mul.b2Vec2(xfA.R, manifold.localNormal, this.normal);
                planePoint = b2Mul.b2Transform_b2Vec2(xfA, manifold.localPoint);

                for (i = 0; i < manifold.pointCount; ++i) {

                    clipPoint = b2Mul.b2Transform_b2Vec2(xfB, manifold.points[i].localPoint);

                    cA = b2Vec2.Copy(this.normal)
                        .Mul(radiusA - b2Dot.b2Vec2(b2Sub(clipPoint, planePoint), this.normal))
                        .Add(clipPoint);
                    cB = b2Sub.b2Vec2(clipPoint, b2Vec2.Copy(this.normal).Mul(radiusB));
                    b2Add.b2Vec2(cA, cB, this.points[i]).Mul(0.5);
                }

                break;

            case b2Manifold.e_faceB:

                b2Mul.b2Vec2(xfB.R, manifold.localNormal, this.normal);
                planePoint = b2Mul.b2Transform_b2Vec2(xfB, manifold.localPoint);

                for (i = 0; i < manifold.pointCount; ++i) {
                    clipPoint = b2Mul.b2Transform_b2Vec2(xfA, manifold.points[i].localPoint);

                    cB = b2Vec2.Copy(this.normal)
                        .Mul(radiusB - b2Dot.b2Vec2(b2Sub(clipPoint, planePoint), this.normal))
                        .Add(clipPoint);
                    cA = b2Sub.b2Vec2(clipPoint, b2Vec2.Copy(this.normal).Mul(radiusA));
                    b2Add.b2Vec2(cA, cB, this.points[i]).Mul(0.5);

                }

                // Ensure normal points from A to B.
                this.normal.Neg();

                break;
        }

    },
    normal: null,   ///< world vector pointing from A to B
    points: null    ///< world contact point (point of intersection)
};


/// This is used for determining the state of contact points.
b2PointState = {
    b2_nullState: 0,		///< point does not exist
    b2_addState: 1,		    ///< point was added in the update
    b2_persistState: 2,	    ///< point persisted across the update
    b2_removeState: 3		///< point was removed in the update
};

/// Used for computing contact manifolds.
function b2ClipVertex() {
    this.v = new b2Vec2(0, 0);
    this.id = new b2ContactID();
}
b2ClipVertex.constructor = b2ClipVertex;
b2ClipVertex.prototype = {
    v: null,
    id: null
};

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
function b2RayCastInput() {
    this.p1 = new b2Vec2(0, 0);
    this.p2 = new b2Vec2(0, 0);
}
b2RayCastInput.constructor = b2RayCastInput;
b2RayCastInput.prototype = {
    p1: null,
    p2: null,
    maxFraction: 0
};

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
function b2RayCastOutput() {
    this.normal = new b2Vec2(0, 0);
}
b2RayCastOutput.constructor = b2RayCastOutput;
b2RayCastOutput.prototype = {
    normal: null,
    fraction: 0
};

var b2AABB = (function(){

    /// An axis aligned bounding box.
    function b2AABB() {
        this.lowerBound = new b2Vec2(0,0);
        this.upperBound = new b2Vec2(0,0);
        this._center = new b2Vec2();
        this._extents = new b2Vec2();

    }
    b2AABB.constructor = b2AABB;

    b2AABB.prototype = {
        /// Verify that the bounds are sorted.
        IsValid: function() {
            var d = b2Sub.b2Vec2(this.upperBound, this.lowerBound);
            var valid = d.x >= 0.0 && d.y >= 0.0;
            valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
            return valid;
        },

        /// Get the center of the AABB.
        GetCenter: function() {
            return b2Add.b2Vec2(this.lowerBound, this.upperBound, this._center).Mul(0.5);
        },

        /// Get the extents of the AABB (half-widths).
        GetExtents: function() {
            return b2Sub.b2Vec2(this.lowerBound, this.upperBound, this._extents).Mul(0.5);
        },

        /// Combine two AABBs into this one.
        Combine: function(aabb1, aabb2) {
            b2Min.b2Vec2(aabb1.lowerBound, aabb2.lowerBound, this.lowerBound);
            b2Max.b2Vec2(aabb1.upperBound, aabb2.upperBound, this.upperBound);
        },

        /// Does this aabb contain the provided AABB.
        Contains: function(aabb) {
            var result = true;
            result = result && this.lowerBound.x <= aabb.lowerBound.x;
            result = result && this.lowerBound.y <= aabb.lowerBound.y;
            result = result && aabb.upperBound.x <= this.upperBound.x;
            result = result && aabb.upperBound.y <= this.upperBound.y;
            return result;
        },

        RayCast: function(output, input) {
            var _i, _xy = ['x','y'], i, t3;

            var tmin = -b2_maxFloat;
            var tmax = b2_maxFloat;

            var p = input.p1;
            var d = b2Sub.v2Vec2(input.p2, input.p1);
            var absD = b2Abs.b2Vec2(d);

            var normal = new b2Vec2();

            for (_i = 0; _i < 2; ++_i) {
                i = _xy[_i];
                if (absD[i] < b2_epsilon) {
                    // Parallel.
                    if (p[i] < this.lowerBound[i] || this.upperBound[i] < p[i]) {
                        return false;
                    }
                } else {
                    var inv_d = 1 / d[i];
                    var t1 = (this.lowerBound[i] - p[i]) * inv_d;
                    var t2 = (this.upperBound[i] - p[i]) * inv_d;

                    // Sign of the normal vector.
                    var s = -1;

                    if (t1 > t2) { //swap
                        t3 = t1;
                        t1 = t2;
                        t2 = t3;
                        s = 1;
                    }

                    // Push the min up
                    if (t1 > tmin) {
                        normal.SetZero();
                        normal[i] = s;
                        tmin = t1;
                    }

                    // Pull the max down
                    tmax = b2Min.float(tmax, t2);

                    if (tmin > tmax) {
                        return false;
                    }
                }
            }

            // Does the ray start inside the box?
            // Does the ray intersect beyond the max fraction?
            if (tmin < 0 || input.maxFraction < tmin)
            {
                return false;
            }

            // Intersection.
            output.fraction = tmin;
            output.normal.Copy(normal);
            return true;

        },

        _center: null,
        _extents: null,


        lowerBound: null,       ///< the lower vertex
        upperBound: null     	///< the upper vertex


    };

    return b2AABB;
})();

// Sutherland-Hodgman clipping.
function b2ClipSegmentToLine(vOut, vIn, normal, offset) {
    // Start with no output points
    var numOut = 0;

    // Calculate the distance of end points to the line
    var distance0 = b2Dot.b2Vec2(normal, vIn[0].v) - offset;
    var distance1 = b2Dot.b2Vec2(normal, vIn[1].v) - offset;

    // If the points are behind the plane
    if (distance0 <= 0) vOut[numOut++] = vIn[0];
    if (distance1 <= 0) vOut[numOut++] = vIn[1];

    // If the points are on different sides of the plane
    if (distance0 * distance1 < 0) {
        // Find intersection point of edge and plane
        var interp = distance0 / (distance0 - distance1);
        vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
        if (distance0 > 0) {
            vOut[numOut].id = vIn[0].id;
        } else {
            vOut[numOut].id = vIn[1].id;
        }
        ++numOut;
    }

    return numOut;
}

function b2TestOverlap(shapeA, shapeB, xfA, xfB) {

    if (xfA) {

        var input = new b2DistanceInput();
        input.proxyA.Set(shapeA);
        input.proxyB.Set(shapeB);
        input.transformA = xfA;
        input.transformB = xfB;
        input.useRadii = true;

        var cache = new b2SimplexCache();
        cache.count = 0;

        var output = new b2DistanceOutput();

        b2Distance(output, cache, input);

        return output.distance < 10 * b2_epsilon;
    } else {

        var d1 = b2Sub.b2Vec2(shapeA.lowerBound, shapeA.upperBound);
        var d2 = b2Sub.b2Vec2(shapeA.lowerBound, shapeA.upperBound);

        if (d1.x > 0 || d1.y > 0) return false;
        if (d2.x > 0 || d2.y > 0) return false;
        return true;
    }
}

