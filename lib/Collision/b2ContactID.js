
   /**
    *  Class b2ContactID
    *
    * @param 
    *
    */
   b2ContactID = Box2D.Collision.b2ContactID = function b2ContactID() {
      this.features = new Features();
      this.features._m_id = this;
   };
   b2ContactID.constructor = b2ContactID;
   b2ContactID.prototype._key = 0;
   b2ContactID.prototype.features = null;


   /**
    * Set
    *
    * @param id
    *
    */
   b2ContactID.prototype.Set = function (id) {
      this.key = id._key;
   };

   /**
    * Copy
    *
    * @param 
    *
    */
   b2ContactID.prototype.Copy = function () {
      var id = new b2ContactID();
      id.key = this.key;
      return id;
   };

   Object.defineProperties(b2ContactID.prototype, {
       key: {
           enumerable: false,
           configurable: true,
           get: function () { return this._key;},
           set: function (value) {
               this._key = value;
               this.features._referenceEdge = this._key & 0x000000ff;
               this.features._incidentEdge = ((this._key & 0x0000ff00) >> 8) & 0x000000ff;
               this.features._incidentVertex = ((this._key & 0x00ff0000) >> 16) & 0x000000ff;
               this.features._flip = ((this._key & 0xff000000) >> 24) & 0x000000ff;
           }
       }
   });

