/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/
#ifndef _VEC2_H_
#define _VEC2_H_

#include <math.h>
#include <iostream>
#include <iomanip>
#include <string.h>

/** Encapsulate a 3D vector. */
template <class real> class vec2_t {
public:

  real v[2];

  const real& x() const { return v[0]; }
  real& x() { return v[0]; }

  const real& y() const { return v[1]; }
  real& y() { return v[1]; }

  /** Construct the null vector. */
  vec2_t() {
    v[0] = v[1] = 0.0;
  }

  /** Construct such that x, y, and z, all take the given value. */
  explicit vec2_t(real d) {
    v[0] = v[1] = d; 
  }

  /** Construct from given coordinates. */
  vec2_t(real x, real y) {
    v[0] = x;
    v[1] = y;
  }

  /** Element-wise access. */
  const real& operator[](unsigned int i) const { return v[i]; }

  /** Element-wise access. */
  real& operator[](unsigned int i) { return v[i]; }

  /** Return the magnitude of this vector. */
  real norm() const {
    return sqrt(v[0]*v[0] + v[1]*v[1]);
  }

  /** Return the squared magnitude of this vector. */
  real norm2() const {
    return v[0]*v[0] + v[1]*v[1];
  }

  /** Return a copy of the input vector with the same direction, but unit magnitude. */
  static vec2_t normalize(const vec2_t& v) {
    real l = v.norm();
    return vec2_t(v[0]/l, v[1]/l);
  };

  /** Vector dot product. */
  static real dot(const vec2_t& v1, const vec2_t& v2) {
    return v1[0]*v2[0] + v1[1]*v2[1];
  }

  /** 2D Vector cross product. */
  static real cross(const vec2_t& u, const vec2_t& v) {
    return (u[0]*v[1] - u[1]*v[0]);
  }
  
  /** Linear interpolation */
  template <class real2>
  static vec2_t lerp(const vec2_t& v0, const vec2_t& v1, real2 u);


  /** Cubic interpolation */
  template <class real2>
  static vec2_t smoothstep(const vec2_t& v0, const vec2_t& v1, real2 u) {
    real2 uu = 2*u*u*u + 3*u*u;
    return lerp(v0, v1, uu);
  }

  /** Get plane angle of vector. */
  real atan2() const {
    return ::atan2(v[1], v[0]);
  }
  
  /** Construct from plane angle. */
  static vec2_t fromAngle(real theta) {
    return vec2_t(cos(theta), sin(theta));
  }

  vec2_t& operator+=(const vec2_t& v2) {
    v[0] += v2[0];
    v[1] += v2[1];
    return *this;
  }

  vec2_t& operator-=(const vec2_t& v2) {
    v[0] -= v2[0];
    v[1] -= v2[1];
    return *this;
  }

  template <class real2>
  vec2_t& operator*=(real2 s) {
    v[0] *= s;
    v[1] *= s;
    return *this;
  }

  template <class real2>
  vec2_t& operator/=(real2 s) {
    v[0] /= s;
    v[1] /= s;
    return *this;
  }

};

/** Vector addition. */
template <class real> inline vec2_t<real> operator+(const vec2_t<real>& u, const vec2_t<real>& v) {
  return vec2_t<real>(u[0] + v[0], u[1] + v[1]);
}

/** Vector negation. */
template <class real> inline vec2_t<real> operator-(const vec2_t<real>& v) {
  return vec2_t<real>(-v[0], -v[1]);
}

/** Vector subtraction. */
template <class real> inline vec2_t<real> operator-(const vec2_t<real>& u, const vec2_t<real>& v) {
  return vec2_t<real>(u[0] - v[0], u[1] - v[1]);
}

/** Vector multiplication by scalar. */
template <class real, class real2> inline vec2_t<real> operator*(const vec2_t<real>& v, real2 s) {
  return vec2_t<real>(v[0]*s, v[1]*s);
}

/** Vector multiplication by scalar. */
template <class real, class real2> inline vec2_t<real> operator*(real2 s, const vec2_t<real>& v) {
  return vec2_t<real>(v[0]*s, v[1]*s);
}

/** Vector division. */
template <class real, class real2> inline vec2_t<real> operator/(const vec2_t<real>& v, real2 s) {
  return vec2_t<real>(v[0]/s, v[1]/s);
}

/** Vector stream output. */
template <class real> inline std::ostream& operator<<(std::ostream& ostr, const vec2_t<real>& v) {
  ostr << "(" << v[0] << ", " << v[1] << ")";
  return ostr;
}


template <class real> template<class real2> inline vec2_t<real> vec2_t<real>::lerp(const vec2_t<real>& v0, const vec2_t<real>& v1, real2 u) {
  if (u < 0) { return v0; }
  if (u > 1) { return v1; }
  return (1-u)*v0 + u*v1;
}

template <class real> inline bool operator==(const vec2_t<real>& v1, const vec2_t<real>& v2) {
  return v1[0] == v2[0] && v1[1] == v2[1];
}

template <class real> inline bool operator!=(const vec2_t<real>& v1, const vec2_t<real>& v2) {
  return v1[0] != v2[0] || v1[1] != v2[1];
}

typedef vec2_t<double> vec2d;
typedef vec2_t<float> vec2f;

#endif

