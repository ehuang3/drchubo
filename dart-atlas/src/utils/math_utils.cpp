#include "math_utils.h"

using namespace Eigen;

double mod(double _x, double _y) {
    if (0 == _y) return _x;
    return _x - _y * floor(_x/_y);
}

double clamp2pi(double _ang) {
   return mod(_ang + M_PI, 2*M_PI) - M_PI;
}

Matrix4d dh2transform(double _r0, double _a0, double _t1, double _d1) {
	Matrix4d T;
	T << cos(_t1), 		   -sin(_t1), 			0, 		  _r0,
		 sin(_t1)*cos(_a0), cos(_t1)*cos(_a0), -sin(_a0), -sin(_a0)*_d1,
		 sin(_t1)*sin(_a0), cos(_t1)*sin(_a0),  cos(_a0),  cos(_a0)*_d1,
		 0,					0,					0,		   1;
	return T;
}
