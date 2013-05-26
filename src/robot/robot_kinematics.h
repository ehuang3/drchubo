#pragma once
#include "robot.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include "robot_arm_kinematics.h"

namespace kinematics { class Skeleton; }
namespace Eigen {
	typedef Matrix<double, 6, 1> Vector6d;
	typedef Matrix< double, 6, 2 > Matrix62d;
}

namespace robot
{

    class robot_state_t;

    class robot_arm_constants_t; //< arm constants
    class robot_arm_kinematics_t;

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

        ////////////////////////////////////////////////////////////////////////
        /// GENERAL IK
        ////////////////////////////////////////////////////////////////////////
        void com_ik(const Eigen::Vector3d& desired_com, 
                    const Eigen::Isometry3d end_effectors[NUM_MANIPULATORS],
                    robot::IK_Mode ik_mode[NUM_MANIPULATORS], robot_state_t& state);

        void stance_ik(const Eigen::Isometry3d end_effectors[NUM_MANIPULATORS],
                       robot::IK_Mode ik_mode[NUM_MANIPULATORS], robot_state_t& state);

        void manip_ik(const Eigen::Isometry3d end_effectors[NUM_MANIPULATORS], 
                      robot::IK_Mode mode[NUM_MANIPULATORS], robot_state_t& state);

        ////////////////////////////////////////////////////////////////////////
        /// LEG IK (EXACT)
        ////////////////////////////////////////////////////////////////////////
        void leg_ik(const Eigen::Isometry3d& B, bool left, robot_state_t& state);

        Eigen::Matrix4d leg_world_to_dh(const Eigen::Matrix4d& B);
        
        ////////////////////////////////////////////////////////////////////////
        /// ARM IK (EXPECT ~6cm ERROR)
        ////////////////////////////////////////////////////////////////////////
        // With option for inaccurate FK through HUBO FK .. 
        void arm_fk(Eigen::Isometry3d& B, bool left, robot_state_t& state, bool use_hubo_fk = false);
        // Inexact, expect ~8cm pos error, exact orientation; Implemented using HUBO FK/IK solver
        void arm_ik(const Eigen::Isometry3d& B, bool left, robot_state_t& state);

        // calculates transform from world to dsy (DH shoulder origin)
        // used to convert from world to HUBO's DH convention
        virtual void xform_w_dsy(Eigen::Isometry3d& B, bool left, robot_state_t& state) = 0;
        // used to convert HUBO DH resting wrist orientation to dart orientation
        // Tw_dsy * B * R = Tw_eef ==> B = Tw_dsy.inv * Tw_eef * R.inv
        virtual void xform_dh_wrist(Eigen::Isometry3d& R) = 0;
        
        // constants used by HUBO FK/IK solver
        robot_arm_kinematics_t arm_kin() { return rak; }
        robot_arm_constants_t arm_constants() { return rak.get_constants(); }
        void arm_constants(const robot_arm_constants_t& _rac) { rak.set_constants(_rac); }

        ////////////////////////////////////////////////////////////////////////
        /// DEPRECIATED FUNCTIONS
        ////////////////////////////////////////////////////////////////////////
        Eigen::Matrix4d legT(int _frame, double _u);
        Eigen::Matrix4d legFK(const Eigen::Vector6d& _u, bool _left);
        bool legIK(const Eigen::Matrix4d& _Tbf, bool _left, const Eigen::Vector6d& _p, Eigen::Vector6d& _u);
        bool comIK(kinematics::Skeleton *_Robot,
                   const Eigen::Vector3d& _dcom,
                   Eigen::Matrix4d& _Twb,
                   IK_Mode _mode[NUM_MANIPULATORS],
                   const Eigen::Matrix4d _Twm[NUM_MANIPULATORS],
                   Eigen::VectorXd& _dofs);
        bool stanceIK(const Eigen::Matrix4d& _Twb, const Eigen::Matrix4d& _Twl,
                      const Eigen::Matrix4d& _Twr, const Eigen::VectorXd& _p,
                      Eigen::VectorXd& _u);
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        struct dh_t {
            double r, a, t, d;
        };
        
        kinematics::Skeleton *robot;
        
        ////////////////////////////////////////////////////////////////////////
        /// LEG KINEMATICS CONSTANTS
        ////////////////////////////////////////////////////////////////////////
        double leg_u_off[6]; //< leg angle offsets to 0 position
        double leg_u_lim[6][2]; //< leg joint limits
        
        Eigen::Vector3d leg_link_disp[7]; //< leg link to link displacement
            
        dh_t leg_dh[7]; //< leg DH parameters
            
        int dart_dof_ind[NUM_MANIPULATORS][6]; //< index of joint angles in DART
        
        Eigen::Matrix4d _legT(int _frame, double _u);
        Eigen::Matrix4d _legFK(const Eigen::Vector6d& _u, bool _left);
        bool _legIK(Eigen::Matrix4d _Tf, bool _left, const Eigen::Vector6d& _p,
                    bool _nearest, Eigen::Vector6d& _u, Eigen::MatrixXd& _U);
        
        ////////////////////////////////////////////////////////////////////////
        /// ARM KINEMATICS SOLVER
        ////////////////////////////////////////////////////////////////////////
        robot_arm_kinematics_t rak;
    };
    
}
