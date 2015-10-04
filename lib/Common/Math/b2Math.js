/**
*  Class b2Math
*
* @param
*
*/
b2Math = Box2D.Common.Math.b2Math = function b2Math() {};
b2Math.constructor = b2Math;

b2Math.b2Vec2_zero = new b2Vec2(0.0, 0.0);
b2Math.b2Mat22_identity = b2Mat22.FromVV(new b2Vec2(1.0, 0.0), new b2Vec2(0.0, 1.0));

/**
* Static IsValid
*
* @param x
*
*/
b2Math.IsValid = function (x) {
  return isFinite(x);
};

/**
* Static Dot
*
* @param a
* @param b
*
*/
b2Math.Dot = function (a, b) {
  return a.x * b.x + a.y * b.y;
};

/**
* Static CrossVV
*
* @param a
* @param b
*
*/
b2Math.CrossVV = function (a, b) {
  return a.x * b.y - a.y * b.x;
};

/**
* Static CrossVF
*
* @param a
* @param s
*
*/
b2Math.CrossVF = function (a, s) {
   return new b2Vec2(s * a.y, (-s * a.x));
};

/**
* Static CrossFV
*
* @param s
* @param a
*
*/
b2Math.CrossFV = function (s, a) {
   return new b2Vec2((-s * a.y), s * a.x);
};

/**
* Static MulMV
*
* @param A
* @param v
*
*/
b2Math.MulMV = function (A, v) {
   return new b2Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
};

/**
* Static MulTMV
*
* @param A
* @param v
*
*/
b2Math.MulTMV = function (A, v) {
   return new b2Vec2(b2Math.Dot(v, A.col1), b2Math.Dot(v, A.col2));
};

/**
* Static MulX
*
* @param T
* @param v
*
*/
b2Math.MulX = function (T, v) {
  var a = b2Math.MulMV(T.R, v);
  a.x += T.position.x;
  a.y += T.position.y;
  return a;
};

/**
* Static MulXT
*
* @param T
* @param v
*
*/
b2Math.MulXT = function (T, v) {
  var a = b2Math.SubtractVV(v, T.position),
      tX = (a.x * T.R.col1.x + a.y * T.R.col1.y);
  a.y = (a.x * T.R.col2.x + a.y * T.R.col2.y);
  a.x = tX;
  return a;
};

/**
* Static AddVV
*
* @param a
* @param b
*
*/
b2Math.AddVV = function (a, b) {
  return new b2Vec2(a.x + b.x, a.y + b.y);
};

/**
* Static SubtractVV
*
* @param a
* @param b
*
*/
b2Math.SubtractVV = function (a, b) {
   return new b2Vec2(a.x - b.x, a.y - b.y);
};

/**
* Static Distance
*
* @param a
* @param b
*
*/
b2Math.Distance = function (a, b) {
  var cX = a.x - b.x,
      cY = a.y - b.y;
  return Math.sqrt(cX * cX + cY * cY);
};

/**
* Static DistanceSquared
*
* @param a
* @param b
*
*/
b2Math.DistanceSquared = function (a, b) {
  var cX = a.x - b.x,
      cY = a.y - b.y;
  return (cX * cX + cY * cY);
};

/**
* Static MulFV
*
* @param s
* @param a
*
*/
b2Math.MulFV = function (s, a) {
  return new b2Vec2(s * a.x, s * a.y);
};

/**
* Static AddMM
*
* @param A
* @param B
*
*/
b2Math.AddMM = function (A, B) {
  return b2Mat22.FromVV(b2Math.AddVV(A.col1, B.col1), b2Math.AddVV(A.col2, B.col2));
};

/**
* Static MulMM
*
* @param A
* @param B
*
*/
b2Math.MulMM = function (A, B) {
  return b2Mat22.FromVV(b2Math.MulMV(A, B.col1), b2Math.MulMV(A, B.col2));
};

/**
* Static MulTMM
*
* @param A
* @param B
*
*/
b2Math.MulTMM = function (A, B) {
  var c1 = new b2Vec2(b2Math.Dot(A.col1, B.col1), b2Math.Dot(A.col2, B.col1)),
      c2 = new b2Vec2(b2Math.Dot(A.col1, B.col2), b2Math.Dot(A.col2, B.col2));
  return b2Mat22.FromVV(c1, c2);
};

/**
* Static Abs
*
* @param a
*
*/
b2Math.Abs = function (a) {
  return a > 0.0 ? a : (-a);
};

/**
* Static AbsV
*
* @param a
*
*/
b2Math.AbsV = function (a) {
  return new b2Vec2(b2Math.Abs(a.x), b2Math.Abs(a.y));
};

/**
* Static AbsM
*
* @param A
*
*/
b2Math.AbsM = function (A) {
  return b2Mat22.FromVV(b2Math.AbsV(A.col1), b2Math.AbsV(A.col2));
};

/**
* Static Min
*
* @param a
* @param b
*
*/
b2Math.Min = function (a, b) {
  return a < b ? a : b;
};

/**
* Static MinV
*
* @param a
* @param b
*
*/
b2Math.MinV = function (a, b) {
  return new b2Vec2(b2Math.Min(a.x, b.x), b2Math.Min(a.y, b.y));
};

/**
* Static Max
*
* @param a
* @param b
*
*/
b2Math.Max = function (a, b) {
  return a > b ? a : b;
};

/**
* Static MaxV
*
* @param a
* @param b
*
*/
b2Math.MaxV = function (a, b) {
  return new b2Vec2(b2Math.Max(a.x, b.x), b2Math.Max(a.y, b.y));
};

/**
* Static Clamp
*
* @param a
* @param low
* @param high
*
*/
b2Math.Clamp = function (a, low, high) {
  return a < low ? low : a > high ? high : a;
};

/**
* Static ClampV
*
* @param a
* @param low
* @param high
*
*/
b2Math.ClampV = function (a, low, high) {
  return b2Math.MaxV(low, b2Math.MinV(a, high));
};

/**
* Static Swap
*
* @param a
* @param b
*
*/
b2Math.Swap = function (a, b) {
  var tmp = a[0];
  a[0] = b[0];
  b[0] = tmp;
};

/**
* Static Random
*
* @param
*
*/
b2Math.Random = function () {
  return Math.random() * 2 - 1;
};

/**
* Static RandomRange
*
* @param lo
* @param hi
*
*/
b2Math.RandomRange = function (lo, hi) {
  return (hi - lo) * Math.random() + lo;
};

/**
* Static NextPowerOfTwo
*
* @param x
*
*/
b2Math.NextPowerOfTwo = function (x) {
  x |= (x >> 1) & 0x7FFFFFFF;
  x |= (x >> 2) & 0x3FFFFFFF;
  x |= (x >> 4) & 0x0FFFFFFF;
  x |= (x >> 8) & 0x00FFFFFF;
  x |= (x >> 16) & 0x0000FFFF;
  return x + 1;
};

/**
* Static IsPowerOfTwo
*
* @param x
*
*/
b2Math.IsPowerOfTwo = function (x) {
  return x > 0 && (x & (x - 1)) === 0;
};
b2Math.b2Transform_identity = new b2Transform(b2Math.b2Vec2_zero, b2Math.b2Mat22_identity);