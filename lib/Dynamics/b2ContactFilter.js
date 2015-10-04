
/**
*  Class b2ContactFilter
*
* @param
*
*/
b2ContactFilter = Box2D.Dynamics.b2ContactFilter = function b2ContactFilter() {};
b2ContactFilter.constructor = b2ContactFilter;
b2ContactFilter.prototype = {

    /**
     * ShouldCollide
     *
     * @param fixtureA
     * @param fixtureB
     *
     */
    ShouldCollide: function (fixtureA, fixtureB) {
        var filter1 = fixtureA.GetFilterData(),
            filter2 = fixtureB.GetFilterData();
        if (filter1.groupIndex === filter2.groupIndex && filter1.groupIndex !== 0) {
            return filter1.groupIndex > 0;
        }
        return (filter1.maskBits & filter2.categoryBits) !== 0 && (filter1.categoryBits & filter2.maskBits) !== 0;
    },

    /**
     * RayCollide
     *
     * @param userData
     * @param fixture
     *
     */
    RayCollide: function (userData, fixture) {
        if (!userData) return true;
        return this.ShouldCollide((userData instanceof b2Fixture ? userData : null), fixture);
    }
};

b2ContactFilter.b2_defaultFilter = new b2ContactFilter();
