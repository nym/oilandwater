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


function b2MassData() {}
b2MassData.prototype = {
    /// The mass of the shape, usually in kilograms.
    mass: 0.

    /// The position of the shape's centroid relative to the shape's origin.
    center: null,

    /// The rotational inertia of the shape about the local origin.
    I: 0

};

/// A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in b2World are created automatically when a b2Fixture
/// is created.
e_unknown= -1;
e_circle = 0;
e_polygon = 1;
e_typeCount = 2;

function b2Shape() {}
b2Shape.constructor = b2Shape;
b2Shape.prototype = {
    m_type: e_unknown,
    m_radius: 0,
    GetType: function() {
        return this.m_type;
    }
};