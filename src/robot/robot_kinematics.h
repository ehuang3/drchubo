#pragma once
#include "robot.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>


namespace kinematics { class Skeleton; }

namespace Eigen 
{
	typedef Matrix<double, 6, 1> Vector6d;
	typedef Matrix< double, 6, 2 > Matrix62d;
}

namespace robot
{
    class robot_state_t;

    enum {
        SIDE_RIGHT = 0,
        SIDE_LEFT = 1
    };

    ////////////////////////////////////////////////////////////////////////////
    /// KINEMATIC CONSTANTS FOR HUBO ARM FK/IK SOLVER
    ////////////////////////////////////////////////////////////////////////////
    struct arm_constants_t {
        double ssz; //< shoulder pitch to roll
        double sez; //< shoulder to elbow
        double ewz; //< elbow to wrist
        double whz; //< wrist to hand

        // Arm limits and offsets
        Eigen::Matrix62d left_limits;
        Eigen::Matrix62d right_limits;

        Eigen::Vector6d left_offset;
        Eigen::Vector6d right_offset;

        // How to flip angle positivity
        Eigen::Vector6d left_mirror;
        Eigen::Vector6d right_mirror;

        arm_constants_t();

        Eigen::Matrix62d getLimits(int side) const;
        Eigen::Vector6d getOffset(int side) const;
        Eigen::Vector6d getMirror(int side) const;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    ////////////////////////////////////////////////////////////////////////////
    /// KINEMATICS SOLVER CLASS: Solves for (pry)pyp-arms & yr(ppp)r-legs
    ////////////////////////////////////////////////////////////////////////////
    class robot_kinematics_t {
    public:
        ////////////////////////////////////////////////////////////////////////
        /// LIFECYLE
        ////////////////////////////////////////////////////////////////////////
        robot_kinematics_t();
        virtual ~robot_kinematics_t();
        virtual void init(kinematics::Skeleton *_robot) = 0;

        ////////////////////////////////////////////////////////////////////////
        /// SPECIAL IK METHODS
        ////////////////////////////////////////////////////////////////////////
        bool com_ik(const Eigen::Vector3d& desired_com, 
                    const Eigen::Isometry3d end_effectors[NUM_MANIPULATORS],
                    robot::IK_Mode ik_mode[NUM_MANIPULATORS], 
                    robot_state_t& state);

        bool stance_ik(const Eigen::Isometry3d end_effectors[NUM_MANIPULATORS],
                       robot::IK_Mode ik_mode[NUM_MANIPULATORS], 
                       robot_state_t& state);

        bool manip_ik(const Eigen::Isometry3d end_effectors[NUM_MANIPULATORS], 
                      robot::IK_Mode mode[NUM_MANIPULATORS], 
                      robot_state_t& state);

        ////////////////////////////////////////////////////////////////////////
        /// LEG IK
        ////////////////////////////////////////////////////////////////////////
        // Global-frame FK/IK
        void leg_fk(Eigen::Isometry3d& B, bool left, robot_state_t& state);
        bool leg_ik(const Eigen::Isometry3d& B, bool left, robot_state_t& state);
        
        // Helper function for transforming leg from world to DH frame
        Eigen::Matrix4d leg_world_to_dh(const Eigen::Matrix4d& B);

        // Body-frame FK/IK
        Eigen::Matrix4d legT(int _frame, double _u);
        Eigen::Matrix4d legFK(const Eigen::Vector6d& _u, bool _left);
        bool legIK(const Eigen::Matrix4d& _Tbf, bool _left, const Eigen::Vector6d& _p, Eigen::Vector6d& _u);
        
        ////////////////////////////////////////////////////////////////////////
        /// ARM IK
        ////////////////////////////////////////////////////////////////////////
        // Global-frame FK/IK
        void arm_fk(Eigen::Isometry3d& B, bool left, robot_state_t& state, bool use_hubo_fk = false);
        // Inexact, expect ~8cm pos error, exact orientation; Implemented using HUBO FK/IK solver
        bool arm_ik(const Eigen::Isometry3d& B, bool left, robot_state_t& state);
        
        // Analytic arm IK with additional jacobian ik to resolve inexact errors
        bool arm_jac_ik(const Eigen::Isometry3d& B, bool left, robot_state_t& state);

        // calculates transform from world to dsy (DH shoulder origin)
        // used to convert from world to HUBO's DH convention
        virtual void xform_w_dsy(Eigen::Isometry3d& B, bool left, robot_state_t& state) = 0;
        // used to convert HUBO DH resting wrist orientation to dart orientation
        // Tw_dsy * B * R = Tw_eef ==> B = Tw_dsy.inv * Tw_eef * R.inv
        virtual void xform_dh_wrist(Eigen::Isometry3d& R, bool left) = 0;
        
        // Body-frame FK/IK, Note: HUBO arm FK/IK solver impl. ==> inexact solutions
        static void DH2HG(Eigen::Isometry3d &B, double t, double f, double r, double d);
        void armFK(Eigen::Isometry3d &B, const Eigen::Vector6d &q, int side) const;
        void armFK(Eigen::Isometry3d &B, const Eigen::Vector6d &q, int side,
                   const Eigen::Isometry3d &endEffector) const;
        bool armIK(Eigen::Vector6d &q, const Eigen::Isometry3d& B,
                   const Eigen::Vector6d& qPrev, int side) const;
        bool armIK(Eigen::Vector6d &q, const Eigen::Isometry3d& B,
                   const Eigen::Vector6d& qPrev, int side,
                   const Eigen::Isometry3d &endEffector) const;

        ////////////////////////////////////////////////////////////////////////
        /// DEPRECIATED FUNCTIONS
        ////////////////////////////////////////////////////////////////////////
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
        bool comIK(kinematics::Skeleton *_Robot,
                   const Eigen::Vector3d& _dcom,
                   Eigen::Matrix4d& _Twb,
                   IK_Mode _mode[NUM_MANIPULATORS],
                   const Eigen::Matrix4d _Twm[NUM_MANIPULATORS],
                   Eigen::VectorXd& _dofs);
        bool stanceIK(const Eigen::Matrix4d& _Twb, const Eigen::Matrix4d& _Twl,
                      const Eigen::Matrix4d& _Twr, const Eigen::VectorXd& _p,
                      Eigen::VectorXd& _u);
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	public:
        struct dh_t {
            double r, a, t, d;
        };
        
        kinematics::Skeleton *robot;
        
        ////////////////////////////////////////////////////////////////////////
        /// LEG KINEMATICS CONSTANTS
        ////////////////////////////////////////////////////////////////////////
        // TODO: leg_constants_t leg;
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
        /// ARM KINEMATICS CONSTANTS
        ////////////////////////////////////////////////////////////////////////
        arm_constants_t arm;

    };
    
}
