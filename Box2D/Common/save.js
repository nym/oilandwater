b2Dot.b2Vec2 = function(a, b) {
    return a.x * b.x + a.y * b.y;
};

b2Dot.b2Vec3 = function(a, b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
};

b2Cross.b2Vec2 = function(a, b) {
    return a.x * b.y - a.y * b.x;
};
/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
b2Cross.b2Vec2_float = function(a, s) {
    return new b2Vec2(s * a.y, -s * a.x);
};
/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
b2Cross.float_b2Vec2 = function(s, a) {
    return new b2Vec2(-s * a.y, s * a.x);
};
/// Perform the cross product on two vectors.
b2Cross.b2Vec3 = function(a, b) {
    return new b2Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
};



b2Mul.float_b2Vec2 = function(s, a) {
    return new b2Vec2(s * a.x, s * a.y);
};

b2Mul.float_b2Vec3 = function(s, a) {
    return new b2Vec3(s * a.x, s * a.y, s * a.z);
};

b2Mul.b2Mat22_b2Vec2 = function(A, y) {
    return new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
};

b2Mul.b2Mat22_b2Mat22 = function(A, B) {
    return new b2Mat22(a.col1.Add(b.col1), a.col2.Add(b.col2))
};

/// Multiply a matrix times a vector.
b2Mul.b2Mat33_b2Vec3 = function(A, v) {
    return A.col1.Mul(v.x).Add(A.col2.Mul(v.y)).Add(A.col3.Mul(v.z));
};
b2Mul.b2Transform_b2Vec2 = function(T, v) {
    var x = T.position.x + T.R.col1.x * v.x + T.R.col2.x * v.y;
    var y = T.position.y + T.R.col1.y * v.x + T.R.col2.y * v.y;

    return new b2Vec2(x, y);
};


b2MulT.b2Mat22 = function(A, B) {
    var c1 = new b2Vec2(b2Dot.b2Vec2(A.col1, B.col1), b2Dot.b2Vec2(A.col2, B.col1));
    var c2 = new b2Vec2(b2Dot.b2Vec2(A.col1, B.col2), b2Dot.b2Vec2(A.col2, B.col2));
    return new b2Mat22(c1, c2);
};
b2MulT.b2Mat22_b2Vec2 = function(A, v) {
    return new b2Vec2(b2Dot.b2Vec2(v, A.col1), b2Dot.b2Vec2(v, A.col2));
};
b2MulT.b2Transform_b2Vec2 = function(T, v) {
    return b2MulT.b2Mat22_b2Vec2(T.R, b2Sub.b2Vec2(v, T.position));
};


b2Add.b2Vec2 = function(a, b) {
    return new b2Vec2(a.x + b.x, a.y + b.y);
};
b2Add.b2Vec3 = function(a, b) {
    return new b2Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
};
b2Add.b2Mat22 = function(A, B) {
    return new b2Mat22(A.col1 + B.col1, A.col2 + B.col2);
};

b2Sub.b2Vec2 = function(a, b) {
    return new b2Vec2(a.x - b.x, a.y - b.y);
};
b2Sub.b2Vec3 = function(a, b) {
    return new b2Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
};

b2Abs.float = Math.abs;

b2Abs.b2Vec2 = function(a) {
    return new b2Vec2(b2Abs.float(a.x), b2Abs.float(a.y));
};
b2Abs.b2Mat22 = function(a) {
    return new b2Mat22(b2Abs.b2Vec2(a.col1), b2Abs.b2Vec2(a.col2));
};


b2Min.float = Math.min;

b2Min.b2Vec2 = function(a, b) {
    return new b2Vec2(b2Min.float(a.x, b.x), b2Min.float(a.y, b.y));

};
b2Max.float = Math.max;
b2Max.b2Vec2 = function(a, b) {
    return new b2Vec2(b2Max.float(a.x, b.x), b2Max.float(a.y, b.y));
};

b2Clamp.float = function(a, low, high) {
    return b2Max.float(low, b2Min.float(a, high));
};
b2Clamp.b2Vec2 = function(a, low, high) {
    return b2Max.b2Vec2(low, b2Min.b2Vec2(a, high));
};


/// Perform the dot product on two vectors.
function b2Dot(a, b) {

    if (a instanceof  b2Vec2)
        return a.x * b.x + a.y * b.y;

    if (a instanceof  b2Vec3)
        return a.x * b.x + a.y * b.y + a.z * b.z;

    throw 'Invalid parameter type in b2Dot';
}


/// Perform the cross product on two vectors. In 2D this produces a scalar.
function b2Cross(a, b) {
    /*
     * (number, bVec2)
     */
    if ('number' === typeof(a))
        return new b2Vec2(-a * b.y, a * b.x);

    /*
     * (bVec2, number)
     */
    if ('number' === typeof(b))
        return new b2Vec2(b * a.y, -b * a.x);

    /*
     * (bVec2, bVec2)
     */
    if (a instanceof  b2Vec2)
        return a.x * b.y - a.y * b.x;

    /*
     * (bVec3, bVec3)
     */
    if (a instanceof  b2Vec3)
        return new b2Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);

    throw 'Invalid parameter type in b2Cross';

}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
function b2Mul(a, b) {

    if ('number' === typeof(a)) {
        /*
         * (number, bVec2)
         */
        if (b instanceof b2Vec2)
            return new b2Vec2(a * b.x, a * b.y);
        /*
         * (number, bVec3)
         */
        if (b instanceof b2Vec3)
            return new b2Vec3(a * b.x, a * b.y, a * b.z);
    }
    if (a instanceof b2Mat22) {
        /*
         * (b2Mat22, bVec2)
         */
        if (b instanceof b2Vec2)
            return  b2Mul.b2Mat22_b2Mat22(a, b);
        /*
         * (b2Mat22, b2Mat22)
         */
        if (b instanceof b2Mat22)
            return b2Mul.b2Mat22_b2Mat22(a, b);
    }
    /*
     * (b2Mat33, b2Vec3)
     */
    if (a instanceof b2Mat33)
        return a.col1.Mul(b.x).Add((a.col2.Mul(b.y))).Add((a.col3.Mul(b.z)));

    /*
     * (b2Transform, b2Vec2) T, v
     */
    if (a instanceof b2Transform) {
        var x = a.position.x + a.R.col1.x * b.x + a.R.col2.x * b.y;
        var y = a.position.y + a.R.col1.y * b.x + a.R.col2.y * b.y;
        return new b2Vec2(x, y);
    }


    throw 'Invalid parameter type in b2Mul';

}



/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
function b2MulT(a, b) {

    if (b instanceof b2Vec2) {
        /*
         * (b2Mat22, b2Vec2)
         */
        if (a instanceof b2Mat22) {
            return new b2Vec2(b2Dot.b2Vec2(b, a.col1), b2Dot.b2Vec2(b, a.col2));
        }
        /*
         * (b2Transform, b2Vec2)
         */
        if (a instanceof b2Transform) {
            return b2MulT.b2Mat22_b2Vec2(a.R, b2Sub.b2Vec2(b, a.position));
        }

    }
    /*
     * (b2Mat22, b2Mat22)
     */
    if (a instanceof b2Mat22)  {
        var c1 = new b2Vec2(b2Dot.b2Vec2(a.col1, b.col1), b2Dot.b2Vec2(a.col2, b.col1));
        var c2 = new b2Vec2(b2Dot.b2Vec2(a.col1, b.col2), b2Dot.b2Vec2(a.col2, b.col2));
        return new b2Mat22(c1, c2);
    }
    throw 'Invalid parameter type in b2MulT';


}

/// Add two vectors component-wise.
function b2Add(a, b) {
    /*
     * (b2Vec2, b2Vec2)
     */
    if (a instanceof b2Vec2)
        return new b2Vec2(a.x + b.x, a.y + b.y);
    /*
     * (b2Vec3, b2Vec3)
     */
    if (a instanceof b2Vec3)
        return new b2Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
    /*
     * (b2Mat22, b2Mat22)
     */
    if (a instanceof b2Mat22)
        return new b2Mat22(a.col1 + b.col1, a.col2 + b.col2);

    throw 'Invalid parameter type in b2Add';
}


/// Subtract two vectors component-wise.
function b2Sub(a, b) {
    /*
     * (b2Vec2, b2Vec2)
     */
    if (a instanceof b2Vec2)
        return new b2Vec2(a.x - b.x, a.y - b.y);
    /*
     * (b2Vec3, b2Vec3)
     */
    if (a instanceof b2Vec3)
        return new b2Vec3(a.x - b.x, a.y - b.y, a.z - b.z);

    throw 'Invalid parameter type in b2Sub';

}

function b2Equal(a, b) {
    return a.x === b.x && a.y === b.y;
}



function b2Distance(a, b) {
    var c = b2Sub.b2Vec2(a, b);
    return c.Length();
}



function b2DistanceSquared(a, b) {
    var c = b2Sub.b2Vec2(a - b);
    return b2Dot.b2Vec2(c, c);
}



function b2Abs(a) {
    if ('number' === typeof(a))
        return Math.abs(a);
    if (a instanceof b2Vec2)
        return new b2Vec2(b2Abs.float(a.x), b2Abs.float(a.y));
    if (a instanceof b2Mat22)
        return new b2Mat22(b2Abs.b2Vec2(a.col1), b2Abs.b2Vec2(a.col2));

    throw 'Invalid parameter type in b2Abs';

}


function b2Min(a, b) {
    if ('number' === typeof(a))
        return Math.min(a, b);
    if (a instanceof b2Vec2)
        return new b2Vec2(Math.min(a.x, b.x), Math.min(a.y, b.y));

    throw 'Invalid parameter type in b2Min';

}


function b2Max(a, b) {
    if ('number' === typeof(a))
        return Math.max(a, b);
    if (a instanceof b2Vec2)
        return new b2Vec2(Math.max(a.x, b.x), Math.max(a.y, b.y));

    throw 'Invalid parameter type in b2Max';

}

function b2Clamp(a, low, high) {
    if ('number' === typeof(a))
        return Math.max(low, Math.min(a, high));

    if (a instanceof b2Vec2)
        return b2Max.b2Vec2(low, b2Min.b2Vec2(a, high));

    throw 'Invalid parameter type in b2Clamp';

}
function b2Swap(a, b) {
    var tmp = a;
    a = b;
    b = tmp;
}

