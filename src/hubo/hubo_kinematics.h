#pragma once
#include <robot/robot_kinematics.h>
#include <robot/robot_state.h>

namespace Eigen { typedef Matrix<double, 6, 1> Vector6d; }
namespace kinematics { class Skeleton; }
namespace hubo
{

class hubo_kinematics_t : public robot::robot_kinematics_t {
public:
	hubo_kinematics_t();
	virtual ~hubo_kinematics_t();

	virtual void init(kinematics::Skeleton *_hubo);

    virtual void xform_w_dsy(Eigen::Isometry3d& B, bool left, robot::robot_state_t& state);
    virtual void xform_dh_wrist(Eigen::Isometry3d& R, bool left);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
