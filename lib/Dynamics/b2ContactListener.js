/**
*  Class b2ContactListener
*
* @param
*
*/
b2ContactListener = Box2D.Dynamics.b2ContactListener = function b2ContactListener() {};
b2ContactListener.constructor = b2ContactListener;
b2ContactListener.prototype = {

    /**
     * BeginContact
     *
     * @param contact
     *
     */
    BeginContact: function (contact) {
    },

    /**
     * EndContact
     *
     * @param contact
     *
     */
    EndContact: function (contact) {
    },

    /**
     * PreSolve
     *
     * @param contact
     * @param oldManifold
     *
     */
    PreSolve: function (contact, oldManifold) {
    },

    /**
     * PostSolve
     *
     * @param contact
     * @param impulse
     *
     */
    PostSolve: function (contact, impulse) {
    }
};
b2ContactListener.b2_defaultListener = new b2ContactListener();