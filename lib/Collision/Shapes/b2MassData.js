
   /**
    *  Class b2MassData
    *
    * @param 
    *
    */
   b2MassData = Box2D.Collision.Shapes.b2MassData = function b2MassData() {
      this.mass = 0.0;
      this.center = new b2Vec2(0, 0);
      this.I = 0.0;

   };
   b2MassData.constructor = b2MassData;