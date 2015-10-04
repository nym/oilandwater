
   /**
    *  Class b2CircleShape
    *
    * @param radius
    *
    */
   b2CircleShape = Box2D.Collision.Shapes.b2CircleShape = function b2CircleShape(radius) {

       this.m_p = new b2Vec2(0, 0);
       radius = radius || 0;
       //b2Shape.call(this);
       this.m_type = b2Shape.e_circleShape;
       this.m_radius = radius;
   };

   b2CircleShape.prototype                  = Object.create(b2Shape.prototype );
   b2CircleShape.prototype.m_type           = b2Shape.e_circleShape;
   b2CircleShape.prototype.m_p              = null;
