/**
*  Class b2FilterData
*
* @param
*
*/
b2FilterData = Box2D.Dynamics.b2FilterData = function b2FilterData() {};
b2FilterData.constructor = b2FilterData;
b2FilterData.prototype = {
    categoryBits: 0x0001,
    maskBits: 0xffff,
    groupIndex: 0,

    /**
     * Copy
     *
     * @param
     *
     */
    Copy: function () {
        var copy = new b2FilterData();
        copy.categoryBits = this.categoryBits;
        copy.maskBits = this.maskBits;
        copy.groupIndex = this.groupIndex;
        return copy;
    }
};