
   /**
    *  Class b2FixtureDef
    *
    * @param
    *
    */
   b2FixtureDef = Box2D.Dynamics.b2FixtureDef = function b2FixtureDef() {
     this.filter = new b2FilterData();
   };
   b2FixtureDef.prototype = {
       shape         : null,
       userData      : null,
       friction      : 0.2,
       restitution   : 0.0,
       density       : 0.0,
       isSensor      : false,
       filter        : null
   };
