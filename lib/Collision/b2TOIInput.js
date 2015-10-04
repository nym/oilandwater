
   /**
    *  Class b2TOIInput
    *
    * @param 
    *
    */
   b2TOIInput = Box2D.Collision.b2TOIInput = function b2TOIInput() {
      this.proxyA = new b2DistanceProxy();
      this.proxyB = new b2DistanceProxy();
      this.sweepA = new b2Sweep();
      this.sweepB = new b2Sweep();

   };
   b2TOIInput.constructor = b2TOIInput;
   b2TOIInput.prototype.proxyA = null;
   b2TOIInput.prototype.proxyB = null;
   b2TOIInput.prototype.sweepA = null;
   b2TOIInput.prototype.sweepB = null;

