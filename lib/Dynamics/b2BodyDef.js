/**
*  Class b2BodyDef
*
* @param
*
*/
b2BodyDef = Box2D.Dynamics.b2BodyDef = function b2BodyDef() {

    this.position = new b2Vec2(0, 0);
    this.linearVelocity = new b2Vec2(0, 0);
    this.position.Set(0.0, 0.0);
    this.linearVelocity.Set(0, 0);
};
/**
* prototype properties
*
*/
b2BodyDef.prototype = {
    position            : null,
    userData            : null,
    angle               : 0.0,
    linearVelocity      : null,
    angularVelocity     : 0.0,
    linearDamping       : 0.0,
    angularDamping      : 0.0,
    allowSleep          : true,
    awake               : true,
    fixedRotation       : false,
    bullet              : false,
    type                : b2Body.b2_staticBody,
    active              : true,
    inertiaScale        : 1.0
};
