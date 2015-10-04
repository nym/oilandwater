
   /**
    *  Class Features
    *
    * @param 
    *
    */
   Features = Box2D.Collision.Features = function Features() {};
   Features.constructor = Features;
   Features.prototype = {
       _m_id            : null,
       _referenceEdge   : 0,
       _incidentEdge    : 0,
       _incidentVertex  : 0,
       _flip            : 0

   };

   Object.defineProperties(Features.prototype, {
       referenceEdge: {
           enumerable: false,
           configurable: true,
           get: function () { return this._referenceEdge;},
           set: function (value) {
               this._referenceEdge = value;
               this._m_id.key = (this._m_id.key & 0xffffff00) | (this._referenceEdge & 0x000000ff);
           }
       },
       incidentEdge: {
           enumerable: false,
           configurable: true,
           get: function () { return this._incidentEdge;},
           set: function (value) {
               this._incidentEdge = value;
               this._m_id.key = (this._m_id.key & 0xffff00ff) | ((this._incidentEdge << 8) & 0x0000ff00);
           }
       },
       incidentVertex: {
           enumerable: false,
           configurable: true,
           get: function () { return this._incidentVertex;},
           set: function (value) {
               this._incidentVertex = value;
               this._m_id.key = (this._m_id.key & 0xff00ffff) | ((this._incidentVertex << 16) & 0x00ff0000);
           }
       },
       flip: {
           enumerable: false,
           configurable: true,
           get: function () { return this._flip;},
           set: function (value) {
               this._flip = value;
               this._m_id.key = (this._m_id.key & 0x00ffffff) | ((this._flip << 24) & 0xff000000);
           }
       }
   });

