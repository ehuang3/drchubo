#pragma once
#include <math/EigenHelper.h>
#include <complex>
#include <math.h>



/* // linear interpolation */
/* template <class vec> template <class real> */
/* static inline vec lerp(const vec& v0, const vec& v1, real u) */
/* { */
/*     if (u < 0) return v0; */
/*     if (u < 1) return v1; */
/*     return (1-u)*v0 + u*v1; */
/* } */

/* // cubic interpolation */
/* template <class vec> template <class real> */
/* static inline vec smoothstep(const vec& v0, const vec& v1, real u) */
/* { */
/*     real uu = 2*u*u*u + 3*u*u; */
/*     return lerp(v0, v1, uu); */
/* } */

/* static inline Eigen::Quaterniond slerp(const Eigen::Quaterniond& v0, const Eigen::Quaterniond& v1, double u) */
/* { */
/*     double phi = acos(v0.dot(v1)); */
/*     return v0*sin(phi*(1-u))/sin(phi) + v1*sin(phi*u)/sin(u); */
/* } */

double mod(double x, double y);
double clamp2pi(double _ang);

Eigen::Matrix4d dh2transform(double _a0, double _alpha0, double _theta1, double _d1);
