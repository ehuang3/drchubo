#pragma once
#include "robot.h"
#include <vector>
#include <map>
#include <string>

#include <kinematics/Skeleton.h>

namespace kinematics { class Skeleton; class BodyNode; }

namespace robot {

    class robot_state_t;
    typedef robot_state_t robot_gains_t;

    class robot_state_t {
    public:
        virtual void init(kinematics::Skeleton *_robot) = 0;

        // Sets floating coordinates of DART dofs
        // warning: assumes dof(0...6) are floating coordinates
        void set_body(const Eigen::Matrix4d& Twb);
        void get_body(Eigen::Matrix4d& Twb) const;
        void set_body(const Eigen::Isometry3d& Twb);
        void get_body(Eigen::Isometry3d& Twb) const;

        // accepts both ManipIndex and LimbIndex
        // returns the limb in g_d_limb (defined in init)
        void set_manip(const Eigen::VectorXd& q, int mi);
        void get_manip(Eigen::VectorXd& q, int mi) const;

        void get_dofs(Eigen::VectorXd& q, const std::vector<int>& indexes) const;
        void set_dofs(const Eigen::VectorXd& q, const std::vector<int>& indexes);
        double& dofs(int i) { return _dofs(i); }
        double dofs(int i) const { return _dofs(i); }
        Eigen::VectorXd& dofs() { return _dofs; }

        //TODO: rename to _dart_, _ros_
        void set_dart_pose(const Eigen::VectorXd& q) { _dofs = q; }
        void get_dart_pose(Eigen::VectorXd& q) const { q = _dofs; }
        void set_ros_pose(const Eigen::VectorXd& q);
        void get_ros_pose(Eigen::VectorXd& q) const;
        const Eigen::VectorXd& dart_pose() const { return _dofs; }
        const Eigen::VectorXd ros_pose() const;

        // WARNING: Using this may cause hard to trace bugs
        // because this Skeleton* is globally shared and writen into.
        // If you do grab this ptr, make sure to do 
        // 
        //       robotSkel->setPose( state.dart_pose() );
        //
        // to avoid inconsistencies.
        kinematics::Skeleton* robot() const { return _robot; }
        
        // returns dart index of joint
        int get_index(const std::string& joint) const;
        // returns the joints initialized in g_d_limb
        // note: accepts both ManipIndex and LimbIndex
        void get_manip_indexes(std::vector<int>& indexes, int mi) const;
        // returns chain from end effector to root (ala dart)
        // warning: only returns joints mappable to ros (see init() of subclass)
/***** FIXME: ROOT ON ATLAS MEANS PELVIS, KIND OF USELESS, BUILD CHAINS MANUALLY INSTEAD PLZ *****/
        void get_branch_indexes(std::vector<int>& indexes, kinematics::BodyNode* end_effector) const;
        // returns joints in chain from base to end effector
        // warning: only returns joints mappable to ros (see init() of subclass)
/***** FIXME: DOES NOT WORK AS EXPECTED B/C I'M NOT HANDLING NON-JOINT LINKS IN SKELETON *****/
/***** PIECE TOGETHER CHAIN INDEXES MANUALLY *****/
        void get_chain_indexes(std::vector<int>& indexes, kinematics::BodyNode* base,
                               kinematics::BodyNode* end_effector) const;
        // returns all joints w/ mapping to ros (see init() of subclass)
        void get_full_indexes(std::vector<int>& indexes) const;

        // wrappers to stl
        std::vector<int> set_intersect(const std::vector<int>& a, const std::vector<int>& b) const;
        std::vector<int> set_union(const std::vector<int>& a, const std::vector<int>& b) const;
        
        // true: if no joint exceed limits
        bool check_limits(const std::vector<int>& indexes, double zero_tol = 1e-9) const;
        bool check_limits(double zero_tol = 1e-9) const;
        Eigen::Vector2d get_limits(int i) const;

        // clamps joints to limits
        // true: if no joints exceed limits
        bool clamp_all(bool err_msg = true, double zero_tol = 1e-9); //< only clamps joints declared in init()
        bool clamp_manip(int mi, bool err_msg = true, double zero_tol = 1e-9);
        bool clamp_indexes(const std::vector<int>& indexes, bool err_msg = true, double zero_tol = 1e-9);
        bool clamp_dof(int i, bool err_msy = true, double zero_tol = 1e-9);

        std::map<int,int> get_d2r() const { return g_d2r; }
        std::map<int,int> get_r2d() const { return g_r2d; }

        void print_nodes(const std::vector<int>& indexes) const; //< nodes <--> links
        void print_joints(const std::vector<int>& indexes) const; //< joints <--> dofs
        void print_limits(const std::vector<int>& indexes) const;
        void print_children(const std::vector<int>& indexes) const; //< child joints of links
        void print_backchain(int i) const; //< index of dof
        void print_dependent_dofs(int i) const; //< index of dof
        static void print_mappings(); //< prints out all mappings

    protected:
        Eigen::VectorXd _dofs; //< DART format
        kinematics::Skeleton *_robot;

        static void static_init(kinematics::Skeleton *robot, std::map<int, std::string> ros2s,
                                std::vector<std::string> l2s[NUM_LIMBS]);

        static void limb_init(std::vector<int>& limb, const std::string& prefix,
                              const std::vector<std::string>& names, std::map<std::string, int>& mmap);

        static int find_d_name(std::string name, kinematics::Skeleton *robot);

        // Assume 1 robot type
        // globals to avoid copying
        static bool g_init;
        static std::string g_ros_prefix;
        
        static std::map<int, std::string> g_r2s;
        static std::map<int, std::string> g_d2s;
        static std::map<std::string, int> g_s2r;
        static std::map<std::string, int> g_s2d;
        static std::map<int, int> g_r2d;
        static std::map<int, int> g_d2r;

        static std::vector<int> g_d_limb[NUM_LIMBS];
        static std::vector<int> g_r_limb[NUM_LIMBS];

    };

}
