/*********************************************************************
 * Copyright (c) 2012 Matt Zucker. All rights reserved.
 *
 * Permission to use and modify this software for educational,
 * research and non-profit purposes, without fee, and without a
 * written agreement is hereby granted to all students currently
 * studying at Swarthmore College or Drexel University, and all
 * faculty and staff currently employed at Swarthmore College or
 * Drexel University.
 *
 * Author:
 *
 *   Matt Zucker - mzucker1@swarthmore.edu
 *   http://www.swarthmore.edu/NatSci/mzucker1/
 *
 ********************************************************************/

#ifndef _BBOX3_H_
#define _BBOX3_H_

#include "vec3.h"

template <class real> class Box3_t {
public:

  typedef vec3_t<real> vec3;

  vec3 p0;
  vec3 p1;

  Box3_t(): p0(1), p1(-1) {}

  Box3_t(const vec3f& pmin, const vec3f& pmax): p0(pmin), p1(pmax) {}

  Box3_t(real x0, real y0, real z0, 
            real x1, real y1, real z1):
    p0(x0, y0, z0), p1(x1, y1, z1) {}

  bool empty() const {
    return 
      (p0.x() > p1.x()) || 
      (p0.y() > p1.y()) || 
      (p0.z() > p1.z());
  }
  
  vec3 center() const {
    return 0.5*(p0+p1);
  }

  bool contains(const vec3& v) const {
    return 
      (v.x() >= p0.x() && v.x() <= p1.x()) &&
      (v.y() >= p0.y() && v.y() <= p1.y()) &&
      (v.z() >= p0.z() && v.z() <= p1.z());
  }

  void addPoint(const vec3& v) {
    if (empty()) {
      p0 = p1 = v;
    } else {
      for (int i=0; i<3; ++i) {
	if (v[i] < p0[i]) { p0[i] = v[i]; }
	if (v[i] > p1[i]) { p1[i] = v[i]; }
      }
    }
  }

  void dilate(double d) {
    p0 -= vec3(d);
    p1 += vec3(d);
  }

  void clear() {
    p0 = vec3(1);
    p1 = vec3(-1);
  }

  static bool intersects(const Box3_t& b1, const Box3_t& b2) {
    if (b1.empty() || b2.empty()) {
      return false;
    }
    for (int i=0; i<3; ++i) {
      if (b1.p0[i] > b2.p1[i] || b1.p1[i] < b2.p0[i]) { return false; }
    }
    return true;
  }
  
  static Box3_t unite(const Box3_t& b1, const Box3_t& b2) {
    if (b1.empty()) {
      return b2;
    } else if (b2.empty()) {
      return b1;
    } else {
      Box3_t rval;
      for (int i=0; i<3; ++i) {
	rval.p0[i] = std::min(b1.p0[i], b2.p0[i]);
	rval.p1[i] = std::max(b1.p1[i], b2.p1[i]);
      }
      return rval;
    }
  }

  static Box3_t intersect(const Box3_t& b1, const Box3_t& b2) {
    if (b1.empty()) {
      return b1;
    } else if (b2.empty()) {
      return b2;
    } else {
      Box3_t rval;
      for (int i=0; i<3; ++i) {
	rval.p0[i] = std::max(b1.p0[i], b2.p0[i]);
	rval.p1[i] = std::min(b1.p1[i], b2.p1[i]);
      }
      return rval;
    }
  }

  vec3 closest(const vec3& v) const {
    vec3 rval;
    for (int i=0; i<3; ++i) {
      rval[i] = std::max(v[i], p0[i]);
      rval[i] = std::min(rval[i], p1[i]);
    }
    return rval;
  }

  bool clipLine(const vec3& v0, const vec3& v1, real& u0, real& u1) const {
    vec3 delta = v1-v0;
    return _clipLine(v0, v1, delta, u0, u1);
  }

  bool clipLine(vec3& v0, vec3& v1) const {
    vec3 delta = v1-v0;
    real u0, u1;
    if (!clipLine(v0, v1, u0, u1)) { return false; }
    v1 = v0 + delta*u1;
    v0 = v0 + delta*u0;
    return true;
  }

private:

  static bool _clipTest(real p, real q, real& u0, real& u1) {
    if (p == 0 && q < 0) { 
      return false;
    } else if (p < 0) {
      real u = q / p;
      if (u > u1) {
        return false;
      } else if (u > u0) {
        u0 = u;
      }
    } else if (p > 0) {
      real u = q / p;
      if (u < u0) {
        return false;
      } else if (u < u1) {
        u1 = u;
      }
    }
    return true;
  }

  bool _clipLine(const vec3& v0, const vec3& v1, 
                 const vec3& delta,
                 real& u0, real& u1) const {

    u0 = 0;
    u1 = 1;

    for (int i=0; i<3; ++i) {
      if (!_clipTest(-delta[i], v0[i]-p0[i], u0, u1) ||
          !_clipTest( delta[i], p1[i]-v0[i], u0, u1)) {
        return false;
      }
    }

    return true;
    
  }


};

typedef Box3_t<double> Box3d;
typedef Box3_t<float>  Box3f;

#endif
