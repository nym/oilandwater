
   /**
    *  Class b2BoundValues
    *
    * @param 
    *
    */
   b2BoundValues = Box2D.Collision.b2BoundValues = function b2BoundValues() {

      this.lowerValues = [];
      this.lowerValues[0] = 0.0;
      this.lowerValues[1] = 0.0;
      this.upperValues = [];
      this.upperValues[0] = 0.0;
      this.upperValues[1] = 0.0;
   };
   b2BoundValues.constructor = b2BoundValues;
   b2BoundValues.prototype.lowerValues = null;
   b2BoundValues.prototype.upperValues = null;
