
   /**
    *  Class b2DynamicTreeBroadPhase
    *
    * @param 
    *
    */
   b2DynamicTreeBroadPhase = Box2D.Collision.b2DynamicTreeBroadPhase = function b2DynamicTreeBroadPhase() {
      this.m_tree = new b2DynamicTree();
      this.m_moveBuffer = [];
      this.m_pairBuffer = [];
      this.m_pairCount = 0;

   };
   b2DynamicTreeBroadPhase.constructor = b2DynamicTreeBroadPhase;
   b2DynamicTreeBroadPhase.prototype.m_tree = null;
   b2DynamicTreeBroadPhase.prototype.m_moveBuffer = null;
   b2DynamicTreeBroadPhase.prototype.m_pairBuffer = null;
   b2DynamicTreeBroadPhase.prototype.m_pairCount = null;

   /**
    * CreateProxy
    *
    * @param aabb
    * @param userData
    *
    */
   b2DynamicTreeBroadPhase.prototype.CreateProxy = function (aabb, userData) {
      var proxy = this.m_tree.CreateProxy(aabb, userData);
      ++this.m_proxyCount;
      this.BufferMove(proxy);
      return proxy;
   };

   /**
    * DestroyProxy
    *
    * @param proxy
    *
    */
   b2DynamicTreeBroadPhase.prototype.DestroyProxy = function (proxy) {
      this.UnBufferMove(proxy);
      --this.m_proxyCount;
      this.m_tree.DestroyProxy(proxy);
   };

   /**
    * MoveProxy
    *
    * @param proxy
    * @param aabb
    * @param displacement
    *
    */
   b2DynamicTreeBroadPhase.prototype.MoveProxy = function (proxy, aabb, displacement) {
      var buffer = this.m_tree.MoveProxy(proxy, aabb, displacement);
      if (buffer) {
         this.BufferMove(proxy);
      }
   };

   /**
    * TestOverlap
    *
    * @param proxyA
    * @param proxyB
    *
    */
   b2DynamicTreeBroadPhase.prototype.TestOverlap = function (proxyA, proxyB) {
      var aabbA = this.m_tree.GetFatAABB(proxyA),
          aabbB = this.m_tree.GetFatAABB(proxyB);
      return aabbA.TestOverlap(aabbB);
   };

   /**
    * GetUserData
    *
    * @param proxy
    *
    */
   b2DynamicTreeBroadPhase.prototype.GetUserData = function (proxy) {
      return this.m_tree.GetUserData(proxy);
   };

   /**
    * GetFatAABB
    *
    * @param proxy
    *
    */
   b2DynamicTreeBroadPhase.prototype.GetFatAABB = function (proxy) {
      return this.m_tree.GetFatAABB(proxy);
   };

   /**
    * GetProxyCount
    *
    * @param 
    *
    */
   b2DynamicTreeBroadPhase.prototype.GetProxyCount = function () {
      return this.m_proxyCount;
   };

   /**
    * UpdatePairs
    *
    * @param callback
    *
    */
   b2DynamicTreeBroadPhase.prototype.UpdatePairs = function (callback) {
      var __this = this;
      __this.m_pairCount = 0;
      var i = 0,
         queryProxy;

       function QueryCallback(proxy) {
           if (proxy === queryProxy) return true;
           if (__this.m_pairCount === __this.m_pairBuffer.length) {
               __this.m_pairBuffer[__this.m_pairCount] = new b2DynamicTreePair();
           }
           var pair = __this.m_pairBuffer[__this.m_pairCount];
           pair.proxyA = proxy < queryProxy ? proxy : queryProxy;
           pair.proxyB = proxy >= queryProxy ? proxy : queryProxy;++__this.m_pairCount;
           return true;
       }

      for (i = 0;
      i < __this.m_moveBuffer.length; ++i) {
         queryProxy = __this.m_moveBuffer[i];

         //function QueryCallback(proxy) {
         //   if (proxy == queryProxy) return true;
         //   if (__this.m_pairCount == __this.m_pairBuffer.length) {
         //      __this.m_pairBuffer[__this.m_pairCount] = new b2DynamicTreePair();
         //   }
         //   var pair = __this.m_pairBuffer[__this.m_pairCount];
         //   pair.proxyA = proxy < queryProxy ? proxy : queryProxy;
         //   pair.proxyB = proxy >= queryProxy ? proxy : queryProxy;++__this.m_pairCount;
         //   return true;
         //};
         var fatAABB = __this.m_tree.GetFatAABB(queryProxy);
         __this.m_tree.Query(QueryCallback, fatAABB);
      }
      __this.m_moveBuffer.length = 0;
      for (var i = 0; i < __this.m_pairCount;) {
         var primaryPair = __this.m_pairBuffer[i],
          userDataA = __this.m_tree.GetUserData(primaryPair.proxyA),
          userDataB = __this.m_tree.GetUserData(primaryPair.proxyB);
         callback(userDataA, userDataB);
         ++i;
         while (i < __this.m_pairCount) {
            var pair = __this.m_pairBuffer[i];
            if (pair.proxyA !== primaryPair.proxyA || pair.proxyB !== primaryPair.proxyB) {
               break;
            }++i;
         }
      }
   };

   /**
    * Query
    *
    * @param callback
    * @param aabb
    *
    */
   b2DynamicTreeBroadPhase.prototype.Query = function (callback, aabb) {
      this.m_tree.Query(callback, aabb);
   };

   /**
    * RayCast
    *
    * @param callback
    * @param input
    *
    */
   b2DynamicTreeBroadPhase.prototype.RayCast = function (callback, input) {
      this.m_tree.RayCast(callback, input);
   };

   /**
    * Validate
    *
    * @param 
    *
    */
   b2DynamicTreeBroadPhase.prototype.Validate = function () {};

   /**
    * Rebalance
    *
    * @param iterations
    *
    */
   b2DynamicTreeBroadPhase.prototype.Rebalance = function (iterations) {
      iterations = iterations || 0;
      this.m_tree.Rebalance(iterations);
   };

   /**
    * BufferMove
    *
    * @param proxy
    *
    */
   b2DynamicTreeBroadPhase.prototype.BufferMove = function (proxy) {
      this.m_moveBuffer[this.m_moveBuffer.length] = proxy;
   };

   /**
    * UnBufferMove
    *
    * @param proxy
    *
    */
   b2DynamicTreeBroadPhase.prototype.UnBufferMove = function (proxy) {
      var i = this.m_moveBuffer.indexOf(proxy);
      this.m_moveBuffer.splice(i, 1);
   };

   /**
    * ComparePairs
    *
    * @param pair1
    * @param pair2
    *
    */
   b2DynamicTreeBroadPhase.prototype.ComparePairs = function (pair1, pair2) {
      return 0;
   };