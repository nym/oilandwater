/**
*  Class b2ContactImpulse
*
* @param
*
*/
b2ContactImpulse = Box2D.Dynamics.b2ContactImpulse = function b2ContactImpulse() {
    this.normalImpulses = [];
    this.tangentImpulses = [];
    for (var i=0; i<b2Settings.b2_maxManifoldPoints; i++) {
        this.normalImpulses.push(0);
        this.tangentImpulses.push(0);
    }

};
b2ContactImpulse.constructor = b2ContactImpulse;
b2ContactImpulse.prototype = {
   normalImpulses: null,
   tangentImpulses: null
};

