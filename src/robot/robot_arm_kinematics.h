#pragma once
#include "robot.h"
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
    class robot_state_t;

    enum {
        SIDE_RIGHT = 0,
        SIDE_LEFT = 1
    };

    typedef std::vector<int> IntArray;

    struct robot_arm_constants_t {
        //double arm_l1,          arm_l2, arm_l3, arm_l4;
        double arm_nsy, arm_ssz, arm_sez, arm_ewz, arm_whz;

        // Arm limits and offsets
        Eigen::Matrix62d left_limits;
        Eigen::Matrix62d right_limits;

        Eigen::Vector6d left_offset;
        Eigen::Vector6d right_offset;
        
        // How to flip angle positivity
        Eigen::Vector6d left_mirror;
        Eigen::Vector6d right_mirror;

        robot_arm_constants_t();
        
        Eigen::Matrix62d getArmLimits(int side) const;
        Eigen::Vector6d  getArmOffset(int side) const;
        IntArray getArmMirror(int side) const;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
            
    class robot_arm_kinematics_t {
    public:
        robot_arm_kinematics_t();
        virtual ~robot_arm_kinematics_t();
        
        /* virtual void init(kinematics::Skeleton *_robot) = 0; */
        

    protected:
        kinematics::Skeleton *robot;
        
        /**************************************************************************
         * HuboKin code - Adopted from Rowland's HUBO armIK solver. 		 	  *
         **************************************************************************/
        robot_arm_constants_t kc;

    public:

        static Eigen::Matrix62d mirrorLimits(const Eigen::Matrix62d& orig, const IntArray& mirror);
        static Eigen::Vector6d  mirrorAngles(const Eigen::Vector6d& orig, const IntArray& mirror);

        static void DH2HG(Eigen::Isometry3d &B, double t, double f, double r, double d);

        robot_arm_constants_t get_constants() { return kc; }
        void set_constants(const robot_arm_constants_t& rc) { kc = rc; }
        
        void armFK(Eigen::Isometry3d &B, const Eigen::Vector6d &q, int side) const;
        
        void armFK(Eigen::Isometry3d &B, const Eigen::Vector6d &q, int side,
                   const Eigen::Isometry3d &endEffector) const;
        
        void armIK(Eigen::Vector6d &q, const Eigen::Isometry3d& B,
                   const Eigen::Vector6d& qPrev, int side) const;

        void armIK(Eigen::Vector6d &q, const Eigen::Isometry3d& B,
                   const Eigen::Vector6d& qPrev, int side,
                   const Eigen::Isometry3d &endEffector) const;
    };
    
}
