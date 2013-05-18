#pragma once
#include <robot/robot_kinematics.h>

namespace Eigen { typedef Matrix<double, 6, 1> Vector6d; }
namespace kinematics { class Skeleton; }
namespace atlas
{

class atlas_kinematics_t : public robot::robot_kinematics_t {
public:
	atlas_kinematics_t();
	virtual ~atlas_kinematics_t();

	virtual void init(kinematics::Skeleton *_atlas);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
