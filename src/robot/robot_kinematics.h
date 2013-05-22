#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace kinematics { class Skeleton; }
namespace Eigen {
	typedef Matrix<double, 6, 1> Vector6d;
	typedef Matrix< double, 6, 2 > Matrix62d;
}
namespace robot
{

class robot_kinematics_t {
public:
	enum ManipIndex {
		MANIP_L_FOOT,
		MANIP_R_FOOT,
		MANIP_L_HAND,
		MANIP_R_HAND,
		NUM_MANIPULATORS,
	};
	//TODO: Support these modes
	enum IK_Mode {
		IK_MODE_FREE,    // you can do whatever you want to these joint angles
		IK_MODE_FIXED,   // joint angles already specified, do not mess with them
		IK_MODE_BODY,    // manipulator specified relative to body
		IK_MODE_WORLD,   // manipulator specified relative to world
		IK_MODE_SUPPORT, // manipulator specfied relative to world, and holding up robot
	};

	robot_kinematics_t();
	virtual ~robot_kinematics_t();

	virtual void init(kinematics::Skeleton *_robot) = 0;

	//TODO: Convert to global rotation frames for joints
	Eigen::Matrix4d dart_armT(int _link, double _u);
	void dart_armFK(Eigen::Isometry3d &B, const Eigen::Vector6d& _u, int _left);
	bool armIK();

	Eigen::Matrix4d legT(int _frame, double _u);
	Eigen::Matrix4d legFK(const Eigen::Vector6d& _u, bool _left);

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

	/* @function: bool comIK(kinematics::Skeleton *_Robot,
	 * 						 const Eigen::Vector3d& _dcom,
	 * 						 Eigen::Matrix4d& _Twb,
	 * 						 IK_Mode _mode[NUM_MANIPULATORS],
	 * 						 const Eigen::Matrix4d _Twm[NUM_MANIPULATORS],
	 * 						 Eigen::VectorXd& _dofs)
	 * @brief: solves IK to put com at _dcom
	 * @parameters:
	 *	   _Robot: for com calculations
	 *		_dcom: desired com in world frame
	 *		 _Twb: xform of body frame in world frame
	 *		_mode: (temporarily unused)
	 *		 _Twm: xform of manipulator frames in world frame
	 * 		_dofs: ouput joint angles & descent initial conditions
	 * @return:
	 * 		 bool: true on success
	 * @preconditions:
	 * 		- manipulator frames are oriented in my DH convention
	 * 		  (call legFK(0, true) to see axis conventions)
	 * 		- dofs are represented in DART-ordering
	 */
	bool comIK(kinematics::Skeleton *_Robot,
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
	 * @postcondition:
	 * 			- u top 6 is left leg, bottom 6 is right leg
	 */
	bool stanceIK(const Eigen::Matrix4d& _Twb, const Eigen::Matrix4d& _Twl,
				  const Eigen::Matrix4d& _Twr, const Eigen::VectorXd& _p,
				  Eigen::VectorXd& _u);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
	struct dh_t {
		double r, a, t, d;
	};

	kinematics::Skeleton *robot;

	//FIXME: double arm_u_off[6]; //< arm angle offsets to 0 position
	//FIXME: double arm_u_lim[6][2]; //< arm joint limits

	double leg_u_off[6]; //< leg angle offsets to 0 position
	double leg_u_lim[6][2]; //< leg joint limits

	//FIXME: Eigen::Vector3d arm_link_disp[7]; //< arm link to link displacement
	Eigen::Vector3d leg_link_disp[7]; //< leg link to link displacement

	//FIXME: dh_t arm_dh[7]; //< arm DH parameters
	dh_t leg_dh[7]; //< leg DH parameters

	int dart_dof_ind[NUM_MANIPULATORS][6]; //< index of joint angles in DART

	Eigen::Matrix4d _legT(int _frame, double _u);
	Eigen::Matrix4d _legFK(const Eigen::Vector6d& _u, bool _left);
	bool _legIK(Eigen::Matrix4d _Tf, bool _left, const Eigen::Vector6d& _p,
			    bool _nearest, Eigen::Vector6d& _u, Eigen::MatrixXd& _U);


	/**************************************************************************
	 * HuboKin code - Adopted from Rowland's HUBO armIK solver. 		 	  *
	 **************************************************************************/
	typedef std::vector<int> IntArray;

	enum {
		SIDE_RIGHT = 0,
		SIDE_LEFT = 1
	};

	struct robot_constants_t {
		double arm_nsy, arm_ssz, arm_sez, arm_ewz, arm_whz;
		//double arm_l1,          arm_l2, arm_l3, arm_l4;

		//FIXME: double leg_nwz, leg_why, leg_whz, leg_hhx, leg_hhz, leg_hkz, leg_kaz;
		//double leg_l1, leg_l2, leg_l3, leg_l4, leg_l5, leg_l6;

		Eigen::Matrix62d arm_limits;
		//FIXME: Eigen::Matrix62d leg_limits;

		Eigen::Vector6d arm_offset;
		//FIXME: Eigen::Vector6d leg_offset;

		IntArray arm_mirror;
		//FIXME: IntArray leg_mirror;

		robot_constants_t();

		Eigen::Matrix62d getArmLimits(int side) const;
		//FIXME: Eigen::Matrix62d getLegLimits(int side) const;
		Eigen::Vector6d  getArmOffset(int side) const;
		//FIXME: Eigen::Vector6d  getLegOffset(int side) const;
	};

	robot_constants_t kc;

    static Eigen::Matrix62d mirrorLimits(const Eigen::Matrix62d& orig, const IntArray& mirror);
    static Eigen::Vector6d  mirrorAngles(const Eigen::Vector6d& orig, const IntArray& mirror);

	static void _DH2HG(Eigen::Isometry3d &B, double t, double f, double r, double d);

    void _armFK(Eigen::Isometry3d &B, const Eigen::Vector6d &q, int side) const;

    void _armFK(Eigen::Isometry3d &B, const Eigen::Vector6d &q, int side,
                const Eigen::Isometry3d &endEffector) const;

    void _armIK(Eigen::Vector6d &q, const Eigen::Isometry3d& B,
                const Eigen::Vector6d& qPrev, int side) const;

    void _armIK(Eigen::Vector6d &q, const Eigen::Isometry3d& B,
                const Eigen::Vector6d& qPrev, int side,
                const Eigen::Isometry3d &endEffector) const;
};

}
