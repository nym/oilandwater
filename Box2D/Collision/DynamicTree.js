/*
 * Copyright (c) 2009 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
var b2_nullNode = -1;

/// A node in the dynamic tree. The client does not interact with this directly.
function b2DynamicTreeNode() {

}
b2DynamicTreeNode.constructor = b2DynamicTreeNode;
b2DynamicTreeNode.prototype = {

    IsLeaf: function() {
        return this.child1 === b2_nullNode;
    },
    aabb: null,
    userData: null,
    next: null,
    child1: null,
    child2: null
};

/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the client
/// object to move by small amounts without triggering a tree update.
///
/// Nodes are pooled and relocatable, so we use node indices rather than pointers.
function b2DynamicTree() {

}
b2DynamicTree.constructor = b2DynamicTree;
b2DynamicTree.prototype = {

    GetUserData: function(proxyId) {
        b2Assert(0 <= proxyId && proxyId < this.m_nodeCapacity);
        return this.m_nodes[proxyId].userData;
    },

    GetFatAABB: function(proxyId) {
        b2Assert(0 <= proxyId && proxyId < this.m_nodeCapacity);
        return this.m_nodes[proxyId].aabb;
    },

    Query: function(callback, aabb) {
        var stack = []; //[k_stackSize];

        stack.push(this.m_root);

        while (stack.length) {
            var nodeId = stack.pop();
            if (nodeId === b2_nullNode) {
                continue;
            }

            var node = this.m_nodes[nodeId];

            if (b2TestOverlap(node.aabb, aabb)) {
                if (node.IsLeaf()) {
                    var proceed = callback.QueryCallback(nodeId);
                    if (proceed === false) {
                        return;
                    }
                } else {
                    stack.push(node.child1);
                    stack.push(node.child2);
                }
            }
        }

    },
    m_root: 0,
    m_nodes: null,
    m_nodeCount: 0,
    m_nodeCapacity: 0,
    m_freeList: null,
    /// This is used incrementally traverse the tree for re-balancing.
    m_path: null,
    m_insertionCount: 0
};