#pragma once
#include <math/EigenHelper.h>
#include <complex>
#include <math.h>

namespace atlas {

double mod(double x, double y);
double clamp2pi(double _ang);

Eigen::Matrix4d dh2transform(double _a0, double _alpha0, double _theta1, double _d1);

}
