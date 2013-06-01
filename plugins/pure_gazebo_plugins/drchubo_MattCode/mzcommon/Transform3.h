/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/
#ifndef _MZTRANSFORM3_H_
#define _MZTRANSFORM3_H_

#include "quat.h"

template <class real> class Transform3_t {
public:

  typedef quat_t<real> quat;
  typedef vec3_t<real> vec3;
  typedef mat3_t<real> mat3;
  typedef mat4_t<real> mat4;
  
  Transform3_t(): _rotFwd(false), _rotInv(false), _dirty(true) {
    setTransform(quat(0,0,0,1), vec3(0,0,0));
  }

  Transform3_t(const quat& rot, const vec3& tx): 
    _rotation(rot), _translation(tx), _rotFwd(false), _rotInv(false), _dirty(true) {}

  explicit Transform3_t(const quat& rot):
    _rotation(rot), _translation(0,0,0), _dirty(true) {}

  explicit Transform3_t(const vec3& t):
    _rotation(0,0,0,1), _translation(t), _dirty(true) {}

  explicit Transform3_t(const mat4& m):
    _rotFwd(false), _rotInv(false), _dirty(true)
  {
    setFromMatrix(m);
  }

  static Transform3_t rx(float theta, const vec3& tx) {
    return Transform3_t(quat::fromAxisAngle(vec3(1,0,0), theta), tx);
  }

  static Transform3_t ry(float theta, const vec3& tx) {
    return Transform3_t(quat::fromAxisAngle(vec3(0,1,0), theta), tx);
  }

  static Transform3_t rz(float theta, const vec3& tx) {
    return Transform3_t(quat::fromAxisAngle(vec3(0,0,1), theta), tx);
  }

  const quat& rotation() const { return _rotation; }

  const vec3& translation() const { return _translation; }

  void setTranslation(const vec3& tx) { _translation = tx; _dirty = true; }
  void setRotation(const quat& rot) { _rotation = rot; _dirty = true; }
  void setRotFromEuler(const vec3& e) { _rotation = quat::fromEuler(e); _dirty = true; }
  void setRotFromMatrix(const mat4& m) { _rotation = quat::fromMatrix(m); _dirty = true; }
  void setRotFromMat3(const mat3& m) { _rotation = quat::fromMat3(m); _dirty = true; }

  void setTransform(const quat& rot, const vec3& tx) {
    _rotation = rot;
    _translation = tx;
    _dirty = true;
  }

  void setFromMatrix(const mat4& m) {
    _rotation = quat::fromMatrix(m);
    _translation = vec3(m(0,3), m(1,3), m(2,3));
    _dirty = true;
  }

  vec3 transformFwd(const vec3& p) const {
    return rotFwd() * p + _translation;
  }

  vec3 transformInv(const vec3& p) const {
    return rotInv() * (p - _translation);
  }
  
  Transform3_t inverse() const {
    return Transform3_t( _rotation.inverse(), transformInv(vec3(0,0,0)) );
  }

  const mat3& rotFwd() const {
    if (_dirty) { _update(); }
    return _rotFwd;
  }

  const mat3& rotInv() const {
    if (_dirty) { _update(); }
    return _rotInv;
  }

  mat4 matrix() const {
    if (_dirty) { _update(); }
    mat4 rval;
    for (int i=0; i<3; ++i) {
      for (int j=0; j<3; ++j) {
        rval(i,j) = _rotFwd(i,j);
      }
    }
    rval(0,3) = _translation[0];
    rval(1,3) = _translation[1];
    rval(2,3) = _translation[2];
    return rval;
  }

private:

  void _update() const {
    _rotation.toMat3(_rotFwd);
    _rotFwd.transpose(_rotInv);
    _dirty = false;
  }

  quat _rotation;
  vec3 _translation;

  mutable mat3 _rotFwd;
  mutable mat3 _rotInv;

  mutable bool _dirty;

};

template <class real> inline std::ostream&
operator<<(std::ostream& ostr, const Transform3_t<real>& t) {
  return ostr << "[" << t.rotation() << "," << t.translation() << "]";
}

template <class real> inline vec3_t<real>
operator*(const Transform3_t<real>& tx, const vec3_t<real>& v) {
  return tx.transformFwd(v);
}

template <class real> inline quat_t<real>
operator*(const Transform3_t<real>& tx, const quat_t<real>& q) {
  return tx.rotation() * q;
}

template <class real> inline Transform3_t<real>
operator*(const Transform3_t<real>& t1, const Transform3_t<real>& t2) {
  return Transform3_t<real>(t1.rotation() * t2.rotation(),
                            t1.transformFwd(t2.translation()));
}

typedef Transform3_t<double> Transform3d;
typedef Transform3_t<float>  Transform3f;

/*
//////////////////////////////////////////////////////////////////////

inline mat4 transform_rx(double theta) {
  double ct = cos(theta);
  double st = sin(theta);
  return mat4(vec4(1, 0,  0,   0),
	      vec4(0, ct, -st, 0),
	      vec4(0, st, ct,  0),
	      vec4(0, 0,  0,   1));
}

inline mat4 transform_ry(double theta) {
  double ct = cos(theta);
  double st = sin(theta);
  return mat4(vec4(ct,  0, st, 0),
	      vec4(0,   1, 0,  0),
	      vec4(-st, 0, ct, 0),
	      vec4(0,   0, 0,  1));
}

inline mat4 transform_rz(double theta) {
  double ct = cos(theta);
  double st = sin(theta);
  return mat4(vec4(ct, -st, 0, 0),
	      vec4(st, ct,  0, 0),
	      vec4(0,  0,   1, 0),
	      vec4(0,  0,   0, 1));
}

//////////////////////////////////////////////////////////////////////

inline mat4 transform_translate(const vec3& tx) {
  return mat4(vec4(1, 0, 0, tx.n[0]),
	      vec4(0, 1, 0, tx.n[1]),
	      vec4(0, 0, 1, tx.n[2]),
	      vec4(0, 0, 0, 1));
}

//////////////////////////////////////////////////////////////////////

inline mat4 transform_rx_fwd(double theta, const vec3& tx) {
  return transform_translate(tx) * transform_rx(theta);
}

inline mat4 transform_ry_fwd(double theta, const vec3& tx) {
  return transform_translate(tx) * transform_ry(theta);
}

inline mat4 transform_rz_fwd(double theta, const vec3& tx) {
  return transform_translate(tx) * transform_rz(theta);
}

//////////////////////////////////////////////////////////////////////

inline mat4 transform_euler_fwd(const vec3& angles, const vec3& tx) {

  return transform_translate(tx) *
    transform_rx(angles.n[0]) * 
    transform_ry(angles.n[1]) * 
    transform_rz(angles.n[2]);

}

//////////////////////////////////////////////////////////////////////


inline mat4 transform_rx_inv(double theta, const vec3& tx) {
  return  transform_rx(theta) * transform_translate(-tx);
}

inline mat4 transform_ry_inv(double theta, const vec3& tx) {
  return transform_ry(-theta) * transform_translate(-tx);
}

inline mat4 transform_rz_inv(double theta, const vec3& tx) {
  return transform_rz(-theta) * transform_translate(-tx);
}

//////////////////////////////////////////////////////////////////////

inline mat4 transform_euler_inv(const vec3& angles, const vec3& tx) {

  return transform_rz(-angles.n[2]) *
    transform_ry(-angles.n[1]) *
    transform_rx(-angles.n[0]) *
    transform_translate(-tx);

}
*/


#endif

