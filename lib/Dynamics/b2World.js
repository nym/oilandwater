/**
*  Class b2World
*
* @param gravity
* @param doSleep
*
*/
b2World = Box2D.Dynamics.b2World = function b2World(gravity, doSleep) {
    this.m_stack = [];
    this.m_contactManager = new b2ContactManager();
    this.m_contactSolver = new b2ContactSolver();
    this.m_island = new b2Island();
    this.m_allowSleep = doSleep;
    this.m_gravity = gravity;
    this.m_contactManager.m_world = this;
    var bd = new b2BodyDef();
    this.m_groundBody = this.CreateBody(bd);
};

b2World.s_timestep2 = new b2TimeStep();
b2World.s_xf = new b2Transform();
b2World.s_backupA = new b2Sweep();
b2World.s_backupB = new b2Sweep();
b2World.s_timestep = new b2TimeStep();
b2World.s_queue = [];
b2World.s_jointColor = new b2Color(0.5, 0.8, 0.8);
b2World.e_newFixture = 0x0001;
b2World.e_locked = 0x0002;
b2World.constructor = b2World;
b2World.prototype = {
    m_stack: null,
    m_contactManager: null,
    m_contactSolver: null,
    m_island: null,
    m_destructionListener: null,
    m_debugDraw: null,
    m_bodyList: null,
    m_contactList: null,
    m_jointList: null,
    m_controllerList: null,
    m_bodyCount: 0,
    m_contactCount: 0,
    m_jointCount: 0,
    m_controllerCount: 0,
    m_allowSleep: null,
    m_gravity: null,
    m_inv_dt0: 0.0,
    m_groundBody: null,
    m_warmStarting: true,
    m_continuousPhysics: true,
    /**
     * SetDestructionListener
     *
     * @param listener
     *
     */
    SetDestructionListener: function (listener) {
        this.m_destructionListener = listener;
    },

    /**
     * SetContactFilter
     *
     * @param filter
     *
     */
    SetContactFilter: function (filter) {
        this.m_contactManager.m_contactFilter = filter;
    },

    /**
     * SetContactListener
     *
     * @param listener
     *
     */
    SetContactListener: function (listener) {
        this.m_contactManager.m_contactListener = listener;
    },

    /**
     * SetDebugDraw
     *
     * @param debugDraw
     *
     */
    SetDebugDraw: function (debugDraw) {
        this.m_debugDraw = debugDraw;
    },

    /**
     * SetBroadPhase
     *
     * @param broadPhase
     *
     */
    SetBroadPhase: function (broadPhase) {
        var oldBroadPhase = this.m_contactManager.m_broadPhase;
        this.m_contactManager.m_broadPhase = broadPhase;
        for (var b = this.m_bodyList; b; b = b.m_next) {
            for (var f = b.m_fixtureList; f; f = f.m_next) {
                f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f);
            }
        }
    },

    /**
     * Validate
     *
     * @param
     *
     */
    Validate: function () {
        this.m_contactManager.m_broadPhase.Validate();
    },

    /**
     * GetProxyCount
     *
     * @param
     *
     */
    GetProxyCount: function () {
        return this.m_contactManager.m_broadPhase.GetProxyCount();
    },

    /**
     * CreateBody
     *
     * @param def
     *
     */
    CreateBody: function (def) {
        if (this.IsLocked() === true) {
            return null;
        }
        var b = new b2Body(def, this);
        b.m_prev = null;
        b.m_next = this.m_bodyList;
        if (this.m_bodyList) {
            this.m_bodyList.m_prev = b;
        }
        this.m_bodyList = b;
        ++this.m_bodyCount;
        return b;
    },

    /**
     * DestroyBody
     *
     * @param b
     *
     */
    DestroyBody: function (b) {
        if (this.IsLocked() === true) {
            return;
        }
        var jn = b.m_jointList;
        while (jn) {
            var jn0 = jn;
            jn = jn.next;
            if (this.m_destructionListener) {
                this.m_destructionListener.SayGoodbyeJoint(jn0.joint);
            }
            this.DestroyJoint(jn0.joint);
        }
        var coe = b.m_controllerList;
        while (coe) {
            var coe0 = coe;
            coe = coe.nextController;
            coe0.controller.RemoveBody(b);
        }
        var ce = b.m_contactList;
        while (ce) {
            var ce0 = ce;
            ce = ce.next;
            this.m_contactManager.Destroy(ce0.contact);
        }
        b.m_contactList = null;
        var f = b.m_fixtureList;
        while (f) {
            var f0 = f;
            f = f.m_next;
            if (this.m_destructionListener) {
                this.m_destructionListener.SayGoodbyeFixture(f0);
            }
            f0.DestroyProxy(this.m_contactManager.m_broadPhase);
            f0.Destroy();
        }
        b.m_fixtureList = null;
        b.m_fixtureCount = 0;
        if (b.m_prev) {
            b.m_prev.m_next = b.m_next;
        }
        if (b.m_next) {
            b.m_next.m_prev = b.m_prev;
        }
        if (b === this.m_bodyList) {
            this.m_bodyList = b.m_next;
        }
        --this.m_bodyCount;
    },

    /**
     * CreateJoint
     *
     * @param def
     *
     */
    CreateJoint: function (def) {
        var j = b2Joint.Create(def, null);
        j.m_prev = null;
        j.m_next = this.m_jointList;
        if (this.m_jointList) {
            this.m_jointList.m_prev = j;
        }
        this.m_jointList = j;
        ++this.m_jointCount;
        j.m_edgeA.joint = j;
        j.m_edgeA.other = j.m_bodyB;
        j.m_edgeA.prev = null;
        j.m_edgeA.next = j.m_bodyA.m_jointList;
        if (j.m_bodyA.m_jointList) j.m_bodyA.m_jointList.prev = j.m_edgeA;
        j.m_bodyA.m_jointList = j.m_edgeA;
        j.m_edgeB.joint = j;
        j.m_edgeB.other = j.m_bodyA;
        j.m_edgeB.prev = null;
        j.m_edgeB.next = j.m_bodyB.m_jointList;
        if (j.m_bodyB.m_jointList) j.m_bodyB.m_jointList.prev = j.m_edgeB;
        j.m_bodyB.m_jointList = j.m_edgeB;
        var bodyA = def.bodyA,
            bodyB = def.bodyB;
        if (def.collideConnected === false) {
            var edge = bodyB.GetContactList();
            while (edge) {
                if (edge.other === bodyA) {
                    edge.contact.FlagForFiltering();
                }
                edge = edge.next;
            }
        }
        return j;
    },

    /**
     * DestroyJoint
     *
     * @param j
     *
     */
    DestroyJoint: function (j) {
        var collideConnected = j.m_collideConnected;
        if (j.m_prev) {
            j.m_prev.m_next = j.m_next;
        }
        if (j.m_next) {
            j.m_next.m_prev = j.m_prev;
        }
        if (j === this.m_jointList) {
            this.m_jointList = j.m_next;
        }
        var bodyA = j.m_bodyA,
            bodyB = j.m_bodyB;
        bodyA.SetAwake(true);
        bodyB.SetAwake(true);
        if (j.m_edgeA.prev) {
            j.m_edgeA.prev.next = j.m_edgeA.next;
        }
        if (j.m_edgeA.next) {
            j.m_edgeA.next.prev = j.m_edgeA.prev;
        }
        if (j.m_edgeA === bodyA.m_jointList) {
            bodyA.m_jointList = j.m_edgeA.next;
        }
        j.m_edgeA.prev = null;
        j.m_edgeA.next = null;
        if (j.m_edgeB.prev) {
            j.m_edgeB.prev.next = j.m_edgeB.next;
        }
        if (j.m_edgeB.next) {
            j.m_edgeB.next.prev = j.m_edgeB.prev;
        }
        if (j.m_edgeB === bodyB.m_jointList) {
            bodyB.m_jointList = j.m_edgeB.next;
        }
        j.m_edgeB.prev = null;
        j.m_edgeB.next = null;
        b2Joint.Destroy(j, null);
        --this.m_jointCount;
        if (collideConnected === false) {
            var edge = bodyB.GetContactList();
            while (edge) {
                if (edge.other === bodyA) {
                    edge.contact.FlagForFiltering();
                }
                edge = edge.next;
            }
        }
    },

    /**
     * AddController
     *
     * @param c
     *
     */
    AddController: function (c) {
        c.m_next = this.m_controllerList;
        c.m_prev = null;
        this.m_controllerList = c;
        c.m_world = this;
        this.m_controllerCount++;
        return c;
    },

    /**
     * RemoveController
     *
     * @param c
     *
     */
    RemoveController: function (c) {
        if (c.m_prev) c.m_prev.m_next = c.m_next;
        if (c.m_next) c.m_next.m_prev = c.m_prev;
        if (this.m_controllerList === c) this.m_controllerList = c.m_next;
        this.m_controllerCount--;
    },

    /**
     * CreateController
     *
     * @param controller
     *
     */
    CreateController: function (controller) {
        if (controller.m_world !== this) throw new Error("Controller can only be a member of one world");
        controller.m_next = this.m_controllerList;
        controller.m_prev = null;
        if (this.m_controllerList) this.m_controllerList.m_prev = controller;
        this.m_controllerList = controller;
        ++this.m_controllerCount;
        controller.m_world = this;
        return controller;
    },

    /**
     * DestroyController
     *
     * @param controller
     *
     */
    DestroyController: function (controller) {
        controller.Clear();
        if (controller.m_next) controller.m_next.m_prev = controller.m_prev;
        if (controller.m_prev) controller.m_prev.m_next = controller.m_next;
        if (controller === this.m_controllerList) this.m_controllerList = controller.m_next;
        --this.m_controllerCount;
    },

    /**
     * SetWarmStarting
     *
     * @param flag
     *
     */
    SetWarmStarting: function (flag) {
        this.m_warmStarting = flag;
    },

    /**
     * SetContinuousPhysics
     *
     * @param flag
     *
     */
    SetContinuousPhysics: function (flag) {
        this.m_continuousPhysics = flag;
    },

    /**
     * GetBodyCount
     *
     * @param
     *
     */
    GetBodyCount: function () {
        return this.m_bodyCount;
    },

    /**
     * GetJointCount
     *
     * @param
     *
     */
    GetJointCount: function () {
        return this.m_jointCount;
    },

    /**
     * GetContactCount
     *
     * @param
     *
     */
    GetContactCount: function () {
        return this.m_contactCount;
    },

    /**
     * SetGravity
     *
     * @param gravity
     *
     */
    SetGravity: function (gravity) {
        this.m_gravity = gravity;
    },

    /**
     * GetGravity
     *
     * @param
     *
     */
    GetGravity: function () {
        return this.m_gravity;
    },

    /**
     * GetGroundBody
     *
     * @param
     *
     */
    GetGroundBody: function () {
        return this.m_groundBody;
    },

    /**
     * Step
     *
     * @param dt
     * @param velocityIterations
     * @param positionIterations
     *
     */
    Step: function (dt, velocityIterations, positionIterations) {
        dt = dt || 0;
        velocityIterations = velocityIterations || 0;
        positionIterations = positionIterations || 0;
        if (this.m_flags & b2World.e_newFixture) {
            this.m_contactManager.FindNewContacts();
            this.m_flags &= ~b2World.e_newFixture;
        }
        this.m_flags |= b2World.e_locked;
        var step = b2World.s_timestep2;
        step.dt = dt;
        step.velocityIterations = velocityIterations;
        step.positionIterations = positionIterations;
        if (dt > 0.0) {
            step.inv_dt = 1.0 / dt;
        }
        else {
            step.inv_dt = 0.0;
        }
        step.dtRatio = this.m_inv_dt0 * dt;
        step.warmStarting = this.m_warmStarting;
        this.m_contactManager.Collide();
        if (step.dt > 0.0) {
            this.Solve(step);
        }
        if (this.m_continuousPhysics && step.dt > 0.0) {
            this.SolveTOI(step);
        }
        if (step.dt > 0.0) {
            this.m_inv_dt0 = step.inv_dt;
        }
        this.m_flags &= ~b2World.e_locked;
    },

    /**
     * ClearForces
     *
     * @param
     *
     */
    ClearForces: function () {
        for (var body = this.m_bodyList; body; body = body.m_next) {
            body.m_force.SetZero();
            body.m_torque = 0.0;
        }
    },

    /**
     * DrawDebugData
     *
     * @param
     *
     */
    DrawDebugData: function () {
        if (this.m_debugDraw == null) {
            return;
        }
        this.m_debugDraw.m_sprite.graphics.clear();
        var flags = this.m_debugDraw.GetFlags(),
            i = 0,
            b,
            f,
            s,
            j,
            bp,
            invQ = new b2Vec2,
            x1 = new b2Vec2,
            x2 = new b2Vec2,
            xf,
            b1 = new b2AABB(),
            b2 = new b2AABB(),
            vs = [new b2Vec2(0, 0), new b2Vec2(0, 0), new b2Vec2(0, 0), new b2Vec2(0, 0)],
            color = new b2Color(0, 0, 0);
        if (flags & b2DebugDraw.e_shapeBit) {
            for (b = this.m_bodyList;
                 b; b = b.m_next) {
                xf = b.m_xf;
                for (f = b.GetFixtureList();
                     f; f = f.m_next) {
                    s = f.GetShape();
                    if (b.IsActive() === false) {
                        color.Set(0.5, 0.5, 0.3);
                        this.DrawShape(s, xf, color);
                    }
                    else if (b.GetType() === b2Body.b2_staticBody) {
                        color.Set(0.5, 0.9, 0.5);
                        this.DrawShape(s, xf, color);
                    }
                    else if (b.GetType() === b2Body.b2_kinematicBody) {
                        color.Set(0.5, 0.5, 0.9);
                        this.DrawShape(s, xf, color);
                    }
                    else if (b.IsAwake() === false) {
                        color.Set(0.6, 0.6, 0.6);
                        this.DrawShape(s, xf, color);
                    }
                    else {
                        color.Set(0.9, 0.7, 0.7);
                        this.DrawShape(s, xf, color);
                    }
                }
            }
        }
        if (flags & b2DebugDraw.e_jointBit) {
            for (j = this.m_jointList;
                 j; j = j.m_next) {
                this.DrawJoint(j);
            }
        }
        if (flags & b2DebugDraw.e_controllerBit) {
            for (var c = this.m_controllerList; c; c = c.m_next) {
                c.Draw(this.m_debugDraw);
            }
        }
        if (flags & b2DebugDraw.e_pairBit) {
            color.Set(0.3, 0.9, 0.9);
            for (var contact = this.m_contactManager.m_contactList; contact; contact = contact.GetNext()) {
                var fixtureA = contact.GetFixtureA(),
                    fixtureB = contact.GetFixtureB(),
                    cA = fixtureA.GetAABB().GetCenter(),
                    cB = fixtureB.GetAABB().GetCenter();
                this.m_debugDraw.DrawSegment(cA, cB, color);
            }
        }
        if (flags & b2DebugDraw.e_aabbBit) {
            bp = this.m_contactManager.m_broadPhase;
            vs = [new b2Vec2(0, 0), new b2Vec2(0, 0), new b2Vec2(0, 0), new b2Vec2(0, 0)];
            for (b = this.m_bodyList;
                 b; b = b.GetNext()) {
                if (b.IsActive() === false) {
                    continue;
                }
                for (f = b.GetFixtureList();
                     f; f = f.GetNext()) {
                    var aabb = bp.GetFatAABB(f.m_proxy);
                    vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
                    vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
                    vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
                    vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);
                    this.m_debugDraw.DrawPolygon(vs, 4, color);
                }
            }
        }
        if (flags & b2DebugDraw.e_centerOfMassBit) {
            for (b = this.m_bodyList;
                 b; b = b.m_next) {
                xf = b2World.s_xf;
                xf.R = b.m_xf.R;
                xf.position = b.GetWorldCenter();
                this.m_debugDraw.DrawTransform(xf);
            }
        }
    },

    /**
     * QueryAABB
     *
     * @param callback
     * @param aabb
     *
     */
    QueryAABB: function (callback, aabb) {
        var __this = this,
            broadPhase = __this.m_contactManager.m_broadPhase;

        function WorldQueryWrapper(proxy) {
            return callback(broadPhase.GetUserData(proxy));
        }

        broadPhase.Query(WorldQueryWrapper, aabb);
    },

    /**
     * QueryShape
     *
     * @param callback
     * @param shape
     * @param transform
     *
     */
    QueryShape: function (callback, shape, transform) {
        var __this = this;
        transform = transform || null;
        if (transform == null) {
            transform = new b2Transform();
            transform.SetIdentity();
        }
        var broadPhase = __this.m_contactManager.m_broadPhase;

        function WorldQueryWrapper(proxy) {
            var fixture = (broadPhase.GetUserData(proxy) instanceof b2Fixture ? broadPhase.GetUserData(proxy) : null);
            if (b2Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform())) return callback(fixture);
            return true;
        }

        var aabb = new b2AABB();
        shape.ComputeAABB(aabb, transform);
        broadPhase.Query(WorldQueryWrapper, aabb);
    },

    /**
     * QueryPoint
     *
     * @param callback
     * @param p
     *
     */
    QueryPoint: function (callback, p) {
        var __this = this,
            broadPhase = __this.m_contactManager.m_broadPhase;

        function WorldQueryWrapper(proxy) {
            var fixture = (broadPhase.GetUserData(proxy) instanceof b2Fixture ? broadPhase.GetUserData(proxy) : null);
            if (fixture.TestPoint(p)) return callback(fixture);
            return true;
        }

        var aabb = new b2AABB();
        aabb.lowerBound.Set(p.x - b2Settings.b2_linearSlop, p.y - b2Settings.b2_linearSlop);
        aabb.upperBound.Set(p.x + b2Settings.b2_linearSlop, p.y + b2Settings.b2_linearSlop);
        broadPhase.Query(WorldQueryWrapper, aabb);
    },

    /**
     * RayCast
     *
     * @param callback
     * @param point1
     * @param point2
     *
     */
    RayCast: function (callback, point1, point2) {
        var __this = this,
            broadPhase = __this.m_contactManager.m_broadPhase,
            output = new b2RayCastOutput;

        function RayCastWrapper(input, proxy) {
            var userData = broadPhase.GetUserData(proxy),
                fixture = (userData instanceof b2Fixture ? userData : null),
                hit = fixture.RayCast(output, input);
            if (hit) {
                var fraction = output.fraction,
                    point = new b2Vec2((1.0 - fraction) * point1.x + fraction * point2.x, (1.0 - fraction) * point1.y + fraction * point2.y);
                return callback(fixture, point, output.normal, fraction);
            }
            return input.maxFraction;
        }

        var input = new b2RayCastInput(point1, point2);
        broadPhase.RayCast(RayCastWrapper, input);
    },

    /**
     * RayCastOne
     *
     * @param point1
     * @param point2
     *
     */
    RayCastOne: function (point1, point2) {
        var __this = this,
            result;

        function RayCastOneWrapper(fixture, point, normal, fraction) {
            fraction = fraction || 0;
            result = fixture;
            return fraction;
        }

        __this.RayCast(RayCastOneWrapper, point1, point2);
        return result;
    },

    /**
     * RayCastAll
     *
     * @param point1
     * @param point2
     *
     */
    RayCastAll: function (point1, point2) {
        var __this = this,
            result = [];

        function RayCastAllWrapper(fixture, point, normal, fraction) {
            fraction = fraction || 0;
            result[result.length] = fixture;
            return 1;
        }

        __this.RayCast(RayCastAllWrapper, point1, point2);
        return result;
    },

    /**
     * GetBodyList
     *
     * @param
     *
     */
    GetBodyList: function () {
        return this.m_bodyList;
    },

    /**
     * GetJointList
     *
     * @param
     *
     */
    GetJointList: function () {
        return this.m_jointList;
    },

    /**
     * GetContactList
     *
     * @param
     *
     */
    GetContactList: function () {
        return this.m_contactList;
    },

    /**
     * IsLocked
     *
     * @param
     *
     */
    IsLocked: function () {
        return (this.m_flags & b2World.e_locked) > 0;
    },

    /**
     * Solve
     *
     * @param step
     *
     */
    Solve: function (step) {
        var b;
        for (var controller = this.m_controllerList; controller; controller = controller.m_next) {
            controller.Step(step);
        }
        var island = this.m_island;
        island.Initialize(this.m_bodyCount, this.m_contactCount, this.m_jointCount, null, this.m_contactManager.m_contactListener, this.m_contactSolver);
        for (b = this.m_bodyList;
             b; b = b.m_next) {
            b.m_flags &= ~b2Body.e_islandFlag;
        }
        for (var c = this.m_contactList; c; c = c.m_next) {
            c.m_flags &= ~b2Contact.e_islandFlag;
        }
        for (var j = this.m_jointList; j; j = j.m_next) {
            j.m_islandFlag = false;
        }
        var stackSize = this.m_bodyCount,
            stack = this.m_stack;
        for (var seed = this.m_bodyList; seed; seed = seed.m_next) {
            if (seed.m_flags & b2Body.e_islandFlag) {
                continue;
            }
            if (seed.IsAwake() === false || seed.IsActive() === false) {
                continue;
            }
            if (seed.GetType() === b2Body.b2_staticBody) {
                continue;
            }
            island.Clear();
            var stackCount = 0;
            stack[stackCount++] = seed;
            seed.m_flags |= b2Body.e_islandFlag;
            while (stackCount > 0) {
                b = stack[--stackCount];
                island.AddBody(b);
                if (b.IsAwake() === false) {
                    b.SetAwake(true);
                }
                if (b.GetType() === b2Body.b2_staticBody) {
                    continue;
                }
                var other;
                for (var ce = b.m_contactList; ce; ce = ce.next) {
                    if (ce.contact.m_flags & b2Contact.e_islandFlag) {
                        continue;
                    }
                    if (ce.contact.IsSensor() === true || ce.contact.IsEnabled() === false || ce.contact.IsTouching() === false) {
                        continue;
                    }
                    island.AddContact(ce.contact);
                    ce.contact.m_flags |= b2Contact.e_islandFlag;
                    other = ce.other;
                    if (other.m_flags & b2Body.e_islandFlag) {
                        continue;
                    }
                    stack[stackCount++] = other;
                    other.m_flags |= b2Body.e_islandFlag;
                }
                for (var jn = b.m_jointList; jn; jn = jn.next) {
                    if (jn.joint.m_islandFlag === true) {
                        continue;
                    }
                    other = jn.other;
                    if (other.IsActive() === false) {
                        continue;
                    }
                    island.AddJoint(jn.joint);
                    jn.joint.m_islandFlag = true;
                    if (other.m_flags & b2Body.e_islandFlag) {
                        continue;
                    }
                    stack[stackCount++] = other;
                    other.m_flags |= b2Body.e_islandFlag;
                }
            }
            island.Solve(step, this.m_gravity, this.m_allowSleep);
            for (var i = 0; i < island.m_bodyCount; ++i) {
                b = island.m_bodies[i];
                if (b.GetType() === b2Body.b2_staticBody) {
                    b.m_flags &= ~b2Body.e_islandFlag;
                }
            }
        }
        for (i = 0; i < stack.length; ++i) {
            if (!stack[i]) break;
            stack[i] = null;
        }
        for (b = this.m_bodyList; b; b = b.m_next) {
            if (b.IsAwake() === false || b.IsActive() === false) {
                continue;
            }
            if (b.GetType() === b2Body.b2_staticBody) {
                continue;
            }
            b.SynchronizeFixtures();
        }
        this.m_contactManager.FindNewContacts();
    },

    /**
     * SolveTOI
     *
     * @param step
     *
     */
    SolveTOI: function (step) {
        var b,
            fA,
            fB,
            bA,
            bB,
            cEdge,
            j,
            island = this.m_island;
        island.Initialize(this.m_bodyCount, b2Settings.b2_maxTOIContactsPerIsland, b2Settings.b2_maxTOIJointsPerIsland, null, this.m_contactManager.m_contactListener, this.m_contactSolver);
        var queue = b2World.s_queue;
        for (b = this.m_bodyList; b; b = b.m_next) {
            b.m_flags &= ~b2Body.e_islandFlag;
            b.m_sweep.t0 = 0.0;
        }
        var c;
        for (c = this.m_contactList; c; c = c.m_next) {
            c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
        }
        for (j = this.m_jointList; j; j = j.m_next) {
            j.m_islandFlag = false;
        }
        for (; ;) {
            var minContact = null,
                minTOI = 1.0;
            for (c = this.m_contactList; c; c = c.m_next) {
                if (c.IsSensor() === true || c.IsEnabled() === false || c.IsContinuous() === false) {
                    continue;
                }
                var toi = 1.0;
                if (c.m_flags & b2Contact.e_toiFlag) {
                    toi = c.m_toi;
                }
                else {
                    fA = c.m_fixtureA;
                    fB = c.m_fixtureB;
                    bA = fA.m_body;
                    bB = fB.m_body;
                    if ((bA.GetType() !== b2Body.b2_dynamicBody || bA.IsAwake() === false) && (bB.GetType() !== b2Body.b2_dynamicBody || bB.IsAwake() === false)) {
                        continue;
                    }
                    var t0 = bA.m_sweep.t0;
                    if (bA.m_sweep.t0 < bB.m_sweep.t0) {
                        t0 = bB.m_sweep.t0;
                        bA.m_sweep.Advance(t0);
                    }
                    else if (bB.m_sweep.t0 < bA.m_sweep.t0) {
                        t0 = bA.m_sweep.t0;
                        bB.m_sweep.Advance(t0);
                    }
                    toi = c.ComputeTOI(bA.m_sweep, bB.m_sweep);
                    b2Assert(0.0 <= toi && toi <= 1.0);
                    if (toi > 0.0 && toi < 1.0) {
                        toi = (1.0 - toi) * t0 + toi;
                        if (toi > 1) toi = 1;
                    }
                    c.m_toi = toi;
                    c.m_flags |= b2Contact.e_toiFlag;
                }
                if (b2Settings.b2_epsilon < toi && toi < minTOI) {
                    minContact = c;
                    minTOI = toi;
                }
            }
            if (minContact == null || 1.0 - 100.0 * b2Settings.b2_epsilon < minTOI) {
                break;
            }
            fA = minContact.m_fixtureA;
            fB = minContact.m_fixtureB;
            bA = fA.m_body;
            bB = fB.m_body;
            b2World.s_backupA.Set(bA.m_sweep);
            b2World.s_backupB.Set(bB.m_sweep);
            bA.Advance(minTOI);
            bB.Advance(minTOI);
            minContact.Update(this.m_contactManager.m_contactListener);
            minContact.m_flags &= ~b2Contact.e_toiFlag;
            if (minContact.IsSensor() === true || minContact.IsEnabled() === false) {
                bA.m_sweep.Set(b2World.s_backupA);
                bB.m_sweep.Set(b2World.s_backupB);
                bA.SynchronizeTransform();
                bB.SynchronizeTransform();
                continue;
            }
            if (minContact.IsTouching() === false) {
                continue;
            }
            var seed = bA;
            if (seed.GetType() !== b2Body.b2_dynamicBody) {
                seed = bB;
            }
            island.Clear();
            var queueStart = 0,
                queueSize = 0;
            queue[queueStart + queueSize++] = seed;
            seed.m_flags |= b2Body.e_islandFlag;
            while (queueSize > 0) {
                b = queue[queueStart++];
                --queueSize;
                island.AddBody(b);
                if (b.IsAwake() === false) {
                    b.SetAwake(true);
                }
                if (b.GetType() !== b2Body.b2_dynamicBody) {
                    continue;
                }
                for (cEdge = b.m_contactList;
                     cEdge; cEdge = cEdge.next) {
                    if (island.m_contactCount === island.m_contactCapacity) {
                        break;
                    }
                    if (cEdge.contact.m_flags & b2Contact.e_islandFlag) {
                        continue;
                    }
                    if (cEdge.contact.IsSensor() === true || cEdge.contact.IsEnabled() === false || cEdge.contact.IsTouching() === false) {
                        continue;
                    }
                    island.AddContact(cEdge.contact);
                    cEdge.contact.m_flags |= b2Contact.e_islandFlag;
                    var other = cEdge.other;
                    if (other.m_flags & b2Body.e_islandFlag) {
                        continue;
                    }
                    if (other.GetType() !== b2Body.b2_staticBody) {
                        other.Advance(minTOI);
                        other.SetAwake(true);
                    }
                    queue[queueStart + queueSize] = other;
                    ++queueSize;
                    other.m_flags |= b2Body.e_islandFlag;
                }
                for (var jEdge = b.m_jointList; jEdge; jEdge = jEdge.next) {
                    if (island.m_jointCount === island.m_jointCapacity) continue;
                    if (jEdge.joint.m_islandFlag === true) continue;
                    other = jEdge.other;
                    if (other.IsActive() === false) {
                        continue;
                    }
                    island.AddJoint(jEdge.joint);
                    jEdge.joint.m_islandFlag = true;
                    if (other.m_flags & b2Body.e_islandFlag) continue;
                    if (other.GetType() !== b2Body.b2_staticBody) {
                        other.Advance(minTOI);
                        other.SetAwake(true);
                    }
                    queue[queueStart + queueSize] = other;
                    ++queueSize;
                    other.m_flags |= b2Body.e_islandFlag;
                }
            }
            var subStep = b2World.s_timestep;
            subStep.warmStarting = false;
            subStep.dt = (1.0 - minTOI) * step.dt;
            subStep.inv_dt = 1.0 / subStep.dt;
            subStep.dtRatio = 0.0;
            subStep.velocityIterations = step.velocityIterations;
            subStep.positionIterations = step.positionIterations;
            island.SolveTOI(subStep);
            var i = 0;
            for (i = 0; i < island.m_bodyCount; ++i) {
                b = island.m_bodies[i];
                b.m_flags &= ~b2Body.e_islandFlag;
                if (b.IsAwake() === false) {
                    continue;
                }
                if (b.GetType() !== b2Body.b2_dynamicBody) {
                    continue;
                }
                b.SynchronizeFixtures();
                for (cEdge = b.m_contactList;
                     cEdge; cEdge = cEdge.next) {
                    cEdge.contact.m_flags &= ~b2Contact.e_toiFlag;
                }
            }
            for (i = 0; i < island.m_contactCount; ++i) {
                c = island.m_contacts[i];
                c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
            }
            for (i = 0; i < island.m_jointCount; ++i) {
                j = island.m_joints[i];
                j.m_islandFlag = false;
            }
            this.m_contactManager.FindNewContacts();
        }
    },

    /**
     * DrawJoint
     *
     * @param joint
     *
     */
    DrawJoint: function (joint) {
        var b1 = joint.GetBodyA(),
            b2 = joint.GetBodyB(),
            xf1 = b1.m_xf,
            xf2 = b2.m_xf,
            x1 = xf1.position,
            x2 = xf2.position,
            p1 = joint.GetAnchorA(),
            p2 = joint.GetAnchorB(),
            color = b2World.s_jointColor;
        switch (joint.m_type) {
            case b2Joint.e_distanceJoint:
                this.m_debugDraw.DrawSegment(p1, p2, color);
                break;
            case b2Joint.e_pulleyJoint:
            {
                var pulley = ((joint instanceof b2PulleyJoint ? joint : null)),
                    s1 = pulley.GetGroundAnchorA(),
                    s2 = pulley.GetGroundAnchorB();
                this.m_debugDraw.DrawSegment(s1, p1, color);
                this.m_debugDraw.DrawSegment(s2, p2, color);
                this.m_debugDraw.DrawSegment(s1, s2, color);
            }
                break;
            case b2Joint.e_mouseJoint:
                this.m_debugDraw.DrawSegment(p1, p2, color);
                break;
            default:
                if (b1 !== this.m_groundBody) this.m_debugDraw.DrawSegment(x1, p1, color);
                this.m_debugDraw.DrawSegment(p1, p2, color);
                if (b2 !== this.m_groundBody) this.m_debugDraw.DrawSegment(x2, p2, color);
        }
    },

    /**
     * DrawShape
     *
     * @param shape
     * @param xf
     * @param color
     *
     */
    DrawShape: function (shape, xf, color) {
        switch (shape.m_type) {
            case b2Shape.e_circleShape:
            {
                var circle = ((shape instanceof b2CircleShape ? shape : null)),
                    center = b2Math.MulX(xf, circle.m_p),
                    radius = circle.m_radius,
                    axis = xf.R.col1;
                this.m_debugDraw.DrawSolidCircle(center, radius, axis, color);
            }
                break;
            case b2Shape.e_polygonShape:
            {
                var i = 0,
                    poly = ((shape instanceof b2PolygonShape ? shape : null)),
                    vertexCount = poly.GetVertexCount(),
                    localVertices = poly.GetVertices(),
                    vertices = [];
                for (i = 0; i < vertexCount; ++i) {
                    vertices.push(b2Math.MulX(xf, localVertices[i]));
                }
                this.m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
            }
                break;
            case b2Shape.e_edgeShape:
            {
                var edge = (shape instanceof b2EdgeShape ? shape : null);
                this.m_debugDraw.DrawSegment(b2Math.MulX(xf, edge.GetVertex1()), b2Math.MulX(xf, edge.GetVertex2()), color);
            }
                break;
        }
    }
}