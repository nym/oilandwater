/*
 * Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented, you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
var b2Settings = {
    b2_maxFloat: Number.MAX_VALUE,
    b2_epsilon: Number.MIN_VALUE,
    b2_pi: Math.PI,
    /// Global tuning constants based on meters-kilograms-seconds (MKS) units.
    ///

    // Collision

    /// The maximum number of contact points between two convex shapes.
    b2_maxManifoldPoints: 2,

    /// The maximum number of vertices on a convex polygon.
    b2_maxPolygonVertices: 4,

    /// This is used to fatten AABBs in the dynamic tree. This allows proxies
    /// to move by a small amount without triggering a tree adjustment.
    /// This is in meters.
    b2_aabbExtension: 0.1,

    /// This is used to fatten AABBs in the dynamic tree. This is used to predict
    /// the future position based on the current displacement.
    /// This is a dimensionless multiplier.
    b2_aabbMultiplier: 2.0,

    /// A small length used as a collision and constraint tolerance. Usually it is
    /// chosen to be numerically significant, but visually insignificant.
    b2_linearSlop: 0.005,

    /// A small angle used as a collision and constraint tolerance. Usually it is
    /// chosen to be numerically significant, but visually insignificant.
    b2_angularSlop: 2.0 / 180.0 * b2Settings.b2_pi,

    /// The radius of the polygon/edge shape skin. This should not be modified. Making
    /// this smaller means polygons will have an insufficient buffer for continuous collision.
    /// Making it larger may create artifacts for vertex collision.
    b2_polygonRadius: 2.0 * b2Settings.b2_linearSlop,


    // Dynamics

    /// Maximum number of contacts to be handled to solve a TOI impact.
    b2_maxTOIContacts: 32,

    /// A velocity threshold for elastic collisions. Any collision with a relative linear
    /// velocity below this threshold will be treated as inelastic.
    b2_velocityThreshold: 1.0,

    /// The maximum linear position correction used when solving constraints. This helps to
    /// prevent overshoot.
    b2_maxLinearCorrection: 0.2,

    /// The maximum angular position correction used when solving constraints. This helps to
    /// prevent overshoot.
    b2_maxAngularCorrection: 8.0 / 180.0 * b2Settings.b2_pi,

    /// The maximum linear velocity of a body. This limit is very large and is used
    /// to prevent numerical problems. You shouldn't need to adjust this.
    b2_maxTranslation: 2.0,
    b2_maxTranslationSquared: b2Settings.b2_maxTranslation * b2Settings.b2_maxTranslation,

    /// The maximum angular velocity of a body. This limit is very large and is used
    /// to prevent numerical problems. You shouldn't need to adjust this.
    b2_maxRotation: 0.5 * b2Settings.b2_pi,
    b2_maxRotationSquared: b2Settings.b2_maxRotation * b2Settings.b2_maxRotation,

    /// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
    /// that overlap is removed in one time step. However using values close to 1 often lead
    /// to overshoot.
    b2_contactBaumgarte: 0.2,

    // Sleep

    /// The time that a body must be still before it will go to sleep.
    b2_timeToSleep: 0.5,

    /// A body cannot sleep if its linear velocity is above this tolerance.
    b2_linearSleepTolerance: 0.01,

    /// A body cannot sleep if its angular velocity is above this tolerance.
    b2_angularSleepTolerance: 2.0 / 180.0 * b2Settings.b2_pi,

    b2_version: '2.1.2',

    b2MixFriction: function (friction1, friction2) {
        return Math.sqrt(friction1 * friction2);
    },

    b2MixRestitution: function (restitution1, restitution2) {
        return restitution1 > restitution2 ? restitution1 : restitution2;
    }
};

