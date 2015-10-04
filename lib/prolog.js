/*
 * Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
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
 *//* Box2dWeb Redux Copyright (c) 2015 by DarkOverlordOfData
 *
 *      A redux of mikolalysenko's npm listing of
 *      Uli Hecht's port of Box2DFlash which is the
 *      flash port of Erin Catto's box2d library.
 */

var Box2D = (function() {
    'use strict';
var ClipVertex, Features, b2AABB, b2Body, b2BodyDef, b2Bound, b2BoundValues, b2BuoyancyController,
    b2CircleContact, b2CircleShape, b2Collision, b2Color, b2ConstantAccelController,
    b2ConstantForceController, b2Contact, b2ContactConstraint, b2ContactConstraintPoint, b2ContactEdge,
    b2ContactFactory, b2ContactFilter, b2ContactID, b2ContactImpulse, b2ContactListener, b2ContactManager,
    b2ContactPoint, b2ContactRegister, b2ContactResult, b2ContactSolver, b2Controller, b2ControllerEdge,
    b2DebugDraw, b2DestructionListener, b2Distance, b2DistanceInput, b2DistanceJoint, b2DistanceJointDef,
    b2DistanceOutput, b2DistanceProxy, b2DynamicTree, b2DynamicTreeBroadPhase, b2DynamicTreeNode,
    b2DynamicTreePair, b2EdgeAndCircleContact, b2EdgeChainDef, b2EdgeShape, b2FilterData, b2Fixture,
    b2FixtureDef, b2FrictionJoint, b2FrictionJointDef, b2GearJoint, b2GearJointDef, b2GravityController,
    b2Island, b2Jacobian, b2Joint, b2JointDef, b2JointEdge, b2LineJoint, b2LineJointDef, b2Manifold,
    b2ManifoldPoint, b2MassData, b2Mat22, b2Mat33, b2Math, b2MouseJoint, b2MouseJointDef, b2NullContact,
    b2Point, b2PolyAndCircleContact, b2PolyAndEdgeContact, b2PolygonContact, b2PolygonShape,
    b2PositionSolverManifold, b2PrismaticJoint, b2PrismaticJointDef, b2PulleyJoint, b2PulleyJointDef,
    b2RayCastInput, b2RayCastOutput, b2RevoluteJoint, b2RevoluteJointDef, b2Segment, b2SeparationFunction,
    b2Settings, b2Shape, b2Simplex, b2SimplexCache, b2SimplexVertex, b2Sweep, b2TOIInput,
    b2TensorDampingController, b2TimeOfImpact, b2TimeStep, b2Transform, b2Vec2, b2Vec3, b2WeldJoint,
    b2WeldJointDef, b2World, b2WorldManifold,
    b2Assert = function (a) {
        if (!a) {
            throw "Assertion Failed";
        }
    },
    Box2D = {
        Common: {
            Math: {}
        },
        Collision: {
            Shapes: {}
        },
        Dynamics: {
            Contacts: {},
            Controllers: {},
            Joints: {}
        }
    };
