
   /**
    *  Class ClipVertex
    *
    * @param 
    *
    */
   ClipVertex = Box2D.Collision.ClipVertex = function ClipVertex() {
      this.v = new b2Vec2(0, 0);
      this.id = new b2ContactID();

   };
   ClipVertex.constructor = ClipVertex;

   ClipVertex.prototype.v = null;
   ClipVertex.prototype.id = null;

   /**
    * Set
    *
    * @param other
    *
    */
   ClipVertex.prototype.Set = function (other) {
      this.v.SetV(other.v);
      this.id.Set(other.id);
   };