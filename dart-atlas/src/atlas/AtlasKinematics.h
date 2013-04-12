#pragma once
#include <utils/EigenHelper.h>

namespace Eigen { typedef Matrix<double, 6, 1> Vector6d; }
namespace kinematics { class Skeleton; }

namespace atlas
{

enum ManipIndex {
	MANIP_L_FOOT,
	MANIP_R_FOOT,
	MANIP_L_HAND,
	MANIP_R_HAND,
	NUM_MANIPULATORS,
};

enum IK_Mode {
	IK_MODE_FREE,    // you can do whatever you want to these joint angles
	IK_MODE_FIXED,   // joint angles already specified, do not mess with them
	IK_MODE_BODY,    // manipulator specified relative to body
	IK_MODE_WORLD,   // manipulator specified relative to world
	IK_MODE_SUPPORT, // manipulator specfied relative to world, and holding up robot
};

class AtlasKinematics {
public:
	AtlasKinematics();
	virtual ~AtlasKinematics();

	void init(kinematics::Skeleton *_atlas);

	Eigen::Matrix4d legT(int _frame, double _u);
	Eigen::Matrix4d legFK(const Eigen::Vector6d& _u, bool _left);

	/* @function: bool comIK(kinematics::Skeleton *_atlas,
	 * 						 const Eigen::Vector3d& _dcom,
	 * 						 Eigen::Matrix4d& _Twb,
	 * 						 IK_Mode _mode[NUM_MANIPULATORS],
	 * 						 const Eigen::Matrix4d _Twm[NUM_MANIPULATORS],
	 * 						 Eigen::VectorXd& _dofs)
	 * @brief: solves IK to put com at _dcom
	 * @parameters:
	 *	   _atlas:
	 *		_dcom: desired com in world frame
	 *		 _Twb: xform of body frame in world frame
	 *		_mode:
	 *		 _Twm: xform of manipulator frames in world frame
	 * 		_dofs: ouput joint angles & descent initial conditions
	 * @return:
	 * 		 bool: true if success
	 * @preconditions:
	 * 		- manipulator frames are oriented in my DH convention
	 * 		  (call legFK(0, true) to see axis conventions)
	 * 		- dofs are represented in DART-ordering
	 */
	bool comIK(kinematics::Skeleton *_atlas,
			   const Eigen::Vector3d& _dcom,
			   Eigen::Matrix4d& _Twb,
			   IK_Mode _mode[NUM_MANIPULATORS],
			   const Eigen::Matrix4d _Twm[NUM_MANIPULATORS],
			   Eigen::VectorXd& _dofs);

	/* @function: stanceIK(const Eigen::Matrix4d& _Twb,
	 * 					   const Eigen::Matrix4d& _Twl,
	 * 					   const Eigen::Matrix4d& _Twr,
	 * 					   const Eigen::VectorXd& _p)
	 * @brief: solves IK for both legs
	 * @parameters:
	 * 		 _Twb: transfrom from world (frame w) to body (frame b)
	 * 		 _Twl: transform from world (frame w) to left foot (frame l)
	 * 		 _Twr: transform from world (frame w) to right foot (frame r)
	 * 		   _p: joint angles for nearest-based selection
	 * @return:
	 *			u: 12x1 joint angle solution nearest to _p
	 * @pre:
	 * @post:
	 */
	bool stanceIK(const Eigen::Matrix4d& _Twb,
				  const Eigen::Matrix4d& _Twl,
				  const Eigen::Matrix4d& _Twr,
				  const Eigen::VectorXd& _p,
				  Eigen::VectorXd& _u);

	/* @function: bool legIK(const Eigen::Matrix4d& _Tbf,
	 * 						 bool _left,
	 * 						 const Eigen::Vector6d& _p,
	 * 						 Eigen::MatrixXd& _u)
	 * @brief: solves for joint angles given the foot transform
	 * @input:
	 * 		_Tbf: transfrom from body (frame b) to foot (frame f/6)
	 * 	   _left: true if left foot
	 * 		  _p: joint angles for nearest-based selection
	 * @output:
	 * 		 &_u: the valid solution nearest _p or _p
	 * @return:
	 * 		bool: false if no valid solutions exists
	 */
	bool legIK(const Eigen::Matrix4d& _Tbf, bool _left, const Eigen::Vector6d& _p, Eigen::Vector6d& _u);

	/* @function: legIK(const Eigen::Matrix4d& _Tbf
	 * 					bool _left,
	 * 					Eigen::MatrixXd& _u)
	 * @brief: solves for joint angles given the foot transform
	 * @input:
	 * 		_Tbf: transfrom from body (frame b) to foot (frame f/6)
	 * 	   _left: true if left foot
	 * @output:
	 * 	      _u: 6x? matrix of valid joint angle solutions
	 * @return:
	 *		bool: false if no solutions exist
	 */
	bool legIK(const Eigen::Matrix4d& _Tbf, bool _left, Eigen::MatrixXd& _u);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	kinematics::Skeleton *atlas;  //warning: shared and modified across classes
	double l0;  // pelvis to hip
	double l1;  // hip yaw to hip pitch z
	double l2;  // hip yaw to hip pitch x
	double l3;  // hip pitch to knee pitch
	double l4;  // knee pitch to ankle pitch
	double u_off[6];  // offsets to joint zeros in context of DART/urdf
	double u_lim[6][2];  // joint limits min max
	double lr[7], la[7], lt[7], ld[7];  // leg dh parameters

	Eigen::Matrix4d _legT(int _frame, double _u);
	Eigen::Matrix4d _legFK(const Eigen::Vector6d& _u, bool _left);

	/* @function: _legIK(Eigen::Matrix4d _Tf,
	 * 					 bool _left,
	 * 					 const Eigen::Vector6d& _p,
	 * 					 bool _nearest,
	 * 					 Eigen::Vector6d& _u,
	 * 					 Eigen::MatrixXd& _U)
	 */
	bool _legIK(Eigen::Matrix4d _Tf,
				bool _left,
			    const Eigen::Vector6d& _p,
			    bool _nearest,
				Eigen::Vector6d& _u,
				Eigen::MatrixXd& _U);
};

}
