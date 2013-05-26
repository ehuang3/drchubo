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
    
    virtual void xform_dh_wrist(Eigen::Isometry3d& R, bool left);
    
    virtual void xform_w_dsy(Eigen::Isometry3d& B, bool left, robot::robot_state_t& state);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
