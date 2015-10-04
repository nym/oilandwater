
   /**
    *  Class b2SimplexVertex
    *
    * @param 
    *
    */
   b2SimplexVertex = Box2D.Collision.b2SimplexVertex = function b2SimplexVertex() {};
   b2SimplexVertex.constructor = b2SimplexVertex;

   b2SimplexVertex.prototype.wA = null;
   b2SimplexVertex.prototype.wB = null;
   b2SimplexVertex.prototype.w = null;
   b2SimplexVertex.prototype.a = null;
   b2SimplexVertex.prototype.indexA = null;
   b2SimplexVertex.prototype.indexB = null;
   /**
    * Set
    *
    * @param other
    *
    */
   b2SimplexVertex.prototype.Set = function (other) {
      this.wA.SetV(other.wA);
      this.wB.SetV(other.wB);
      this.w.SetV(other.w);
      this.a = other.a;
      this.indexA = other.indexA;
      this.indexB = other.indexB;
   };