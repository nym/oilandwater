/**
*  Class b2Fixture
*
* @param 
*
*/
b2Fixture = Box2D.Dynamics.b2Fixture = function b2Fixture() {
    this.m_filter = new b2FilterData();
    this.m_aabb = new b2AABB();
};

b2Fixture.constructor = b2Fixture;
b2Fixture.prototype = {
    m_filter: null,
    m_shape: null,
    m_isSensor: false,
    m_body: null,
    m_next: null,
    m_userData: null,
    m_density: 0.0,
    m_friction: 0.0,
    m_restitution: 0.0,
    m_aabb: null,
    m_proxy: null,
    /**
     * GetType
     *
     * @param
     *
     */
    GetType: function () {
        return this.m_shape.GetType();
    },

    /**
     * GetShape
     *
     * @param
     *
     */
    GetShape: function () {
        return this.m_shape;
    },

    /**
     * SetSensor
     *
     * @param sensor
     *
     */
    SetSensor: function (sensor) {
        if (this.m_isSensor === sensor) return;
        this.m_isSensor = sensor;
        if (this.m_body == null) return;
        var edge = this.m_body.GetContactList();
        while (edge) {
            var contact = edge.contact,
                fixtureA = contact.GetFixtureA(),
                fixtureB = contact.GetFixtureB();
            if (fixtureA === this || fixtureB === this) contact.SetSensor(fixtureA.IsSensor() || fixtureB.IsSensor());
            edge = edge.next;
        }
    },

    /**
     * IsSensor
     *
     * @param
     *
     */
    IsSensor: function () {
        return this.m_isSensor;
    },

    /**
     * SetFilterData
     *
     * @param filter
     *
     */
    SetFilterData: function (filter) {
        this.m_filter = filter.Copy();
        if (this.m_body) return;
        var edge = this.m_body.GetContactList();
        while (edge) {
            var contact = edge.contact,
                fixtureA = contact.GetFixtureA(),
                fixtureB = contact.GetFixtureB();
            if (fixtureA === this || fixtureB === this) contact.FlagForFiltering();
            edge = edge.next;
        }
    },

    /**
     * GetFilterData
     *
     * @param
     *
     */
    GetFilterData: function () {
        return this.m_filter.Copy();
    },

    /**
     * GetBody
     *
     * @param
     *
     */
    GetBody: function () {
        return this.m_body;
    },

    /**
     * GetNext
     *
     * @param
     *
     */
    GetNext: function () {
        return this.m_next;
    },

    /**
     * GetUserData
     *
     * @param
     *
     */
    GetUserData: function () {
        return this.m_userData;
    },

    /**
     * SetUserData
     *
     * @param data
     *
     */
    SetUserData: function (data) {
        this.m_userData = data;
    },

    /**
     * TestPoint
     *
     * @param p
     *
     */
    TestPoint: function (p) {
        return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
    },

    /**
     * RayCast
     *
     * @param output
     * @param input
     *
     */
    RayCast: function (output, input) {
        return this.m_shape.RayCast(output, input, this.m_body.GetTransform());
    },

    /**
     * GetMassData
     *
     * @param massData
     *
     */
    GetMassData: function (massData) {
        massData = massData || null;
        if (massData == null) {
            massData = new b2MassData();
        }
        this.m_shape.ComputeMass(massData, this.m_density);
        return massData;
    },

    /**
     * SetDensity
     *
     * @param density
     *
     */
    SetDensity: function (density) {
        density = density || 0;
        this.m_density = density;
    },

    /**
     * GetDensity
     *
     * @param
     *
     */
    GetDensity: function () {
        return this.m_density;
    },

    /**
     * GetFriction
     *
     * @param
     *
     */
    GetFriction: function () {
        return this.m_friction;
    },

    /**
     * SetFriction
     *
     * @param friction
     *
     */
    SetFriction: function (friction) {
        friction = friction || 0;
        this.m_friction = friction;
    },

    /**
     * GetRestitution
     *
     * @param
     *
     */
    GetRestitution: function () {
        return this.m_restitution;
    },

    /**
     * SetRestitution
     *
     * @param restitution
     *
     */
    SetRestitution: function (restitution) {
        restitution = restitution || 0;
        this.m_restitution = restitution;
    },

    /**
     * GetAABB
     *
     * @param
     *
     */
    GetAABB: function () {
        return this.m_aabb;
    },

    /**
     * Create
     *
     * @param body
     * @param xf
     * @param def
     *
     */
    Create: function (body, xf, def) {
        this.m_userData = def.userData;
        this.m_friction = def.friction;
        this.m_restitution = def.restitution;
        this.m_body = body;
        this.m_next = null;
        this.m_filter = def.filter.Copy();
        this.m_isSensor = def.isSensor;
        this.m_shape = def.shape.Copy();
        this.m_density = def.density;
    },

    /**
     * Destroy
     *
     * @param
     *
     */
    Destroy: function () {
        this.m_shape = null;
    },

    /**
     * CreateProxy
     *
     * @param broadPhase
     * @param xf
     *
     */
    CreateProxy: function (broadPhase, xf) {
        this.m_shape.ComputeAABB(this.m_aabb, xf);
        this.m_proxy = broadPhase.CreateProxy(this.m_aabb, this);
    },

    /**
     * DestroyProxy
     *
     * @param broadPhase
     *
     */
    DestroyProxy: function (broadPhase) {
        if (this.m_proxy == null) {
            return;
        }
        broadPhase.DestroyProxy(this.m_proxy);
        this.m_proxy = null;
    },

    /**
     * Synchronize
     *
     * @param broadPhase
     * @param transform1
     * @param transform2
     *
     */
    Synchronize: function (broadPhase, transform1, transform2) {
        if (!this.m_proxy) return;
        var aabb1 = new b2AABB(),
            aabb2 = new b2AABB();
        this.m_shape.ComputeAABB(aabb1, transform1);
        this.m_shape.ComputeAABB(aabb2, transform2);
        this.m_aabb.Combine(aabb1, aabb2);
        var displacement = b2Math.SubtractVV(transform2.position, transform1.position);
        broadPhase.MoveProxy(this.m_proxy, this.m_aabb, displacement);
    }
}