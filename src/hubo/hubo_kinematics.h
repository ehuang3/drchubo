#pragma once
#include <robot/robot_kinematics.h>

namespace Eigen { typedef Matrix<double, 6, 1> Vector6d; }
namespace kinematics { class Skeleton; }
namespace hubo
{

class hubo_kinematics_t : public robot::robot_kinematics_t {
public:
	hubo_kinematics_t();
	virtual ~hubo_kinematics_t();

	virtual void init(kinematics::Skeleton *_hubo);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
