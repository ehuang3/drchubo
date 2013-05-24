#pragma once
#include "robot.h"
#include <vector>
#include <map>
#include <string>

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
        void get_body(Eigen::Matrix4d& Twb);
        void set_body(const Eigen::Isometry3d& Twb);
        void get_body(Eigen::Isometry3d& Twb);

        // accepts both ManipIndex and LimbIndex
        // returns the limb in g_d_limb (defined in init)
        void set_manip(const Eigen::VectorXd& q, int mi);
        void get_manip(Eigen::VectorXd& q, int mi);

        void get_dofs(Eigen::VectorXd& q, const std::vector<int>& indexes);
        void set_dofs(const Eigen::VectorXd& q, const std::vector<int>& indexes);
        double& dofs(int i) { return _dofs(i); }
        Eigen::VectorXd& dofs() { return _dofs; }

        //TODO: rename to _dart_, _ros_
        void set_dart_pose(const Eigen::VectorXd& q) { _dofs = q; }
        void get_dart_pose(Eigen::VectorXd& q) { q = _dofs; }
        void set_ros_pose(const Eigen::VectorXd& q);
        void get_ros_pose(Eigen::VectorXd& q);
        Eigen::VectorXd& dart_pose() { return _dofs; }

        // copies dofs into internal dart skeleton
        void copy_robot_pose();
        kinematics::Skeleton* robot() { return _robot; }
        
        // returns dart index of joint
        int get_index(const std::string& joint);
        // returns the joints initialized in g_d_limb
        // note: accepts both ManipIndex and LimbIndex
        void get_manip_indexes(std::vector<int>& indexes, int mi);
        // returns chain from end effector to root (ala dart)
        // warning: only returns joints mappable to ros (see init() of subclass)
/***** FIXME: ROOT ON ATLAS MEANS PELVIS, KIND OF USELESS, BUILD CHAINS MANUALLY INSTEAD PLZ *****/
        void get_branch_indexes(std::vector<int>& indexes, kinematics::BodyNode* end_effector);
        // returns joints in chain from base to end effector
        // warning: only returns joints mappable to ros (see init() of subclass)
/***** FIXME: DOES NOT WORK AS EXPECTED B/C I'M NOT HANDLING NON-JOINT LINKS IN SKELETON *****/
/***** PIECE TOGETHER CHAIN INDEXES MANUALLY *****/
        void get_chain_indexes(std::vector<int>& indexes, kinematics::BodyNode* base,
                               kinematics::BodyNode* end_effector);
        // returns all joints w/ mapping to ros (see init() of subclass)
        void get_full_indexes(std::vector<int>& indexes);

        // wrappers to stl
        std::vector<int> set_intersect(const std::vector<int>& a, const std::vector<int>& b);
        std::vector<int> set_union(const std::vector<int>& a, const std::vector<int>& b);
        
        // true: if no joint exceed limits
        bool check_limits(const std::vector<int>& indexes, double zero_tol = 1e-9);

        // clamps joints to limits
        // true: if no joints exceed limits
        bool clamp_all(bool err_msg = true, double zero_tol = 1e-9); //< only clamps joints declared in init()
        bool clamp_manip(int mi, bool err_msg = true, double zero_tol = 1e-9);
        bool clamp_indexes(const std::vector<int>& indexes, bool err_msg = true, double zero_tol = 1e-9);
        inline bool clamp_dof(int i, bool err_msy = true, double zero_tol = 1e-9);

        std::map<int,int> get_d2r() { return g_d2r; }
        std::map<int,int> get_r2d() { return g_r2d; }

        void print_nodes(const std::vector<int>& indexes); //< nodes <--> links
        void print_joints(const std::vector<int>& indexes); //< joints <--> dofs
        void print_limits(const std::vector<int>& indexes);
        void print_children(const std::vector<int>& indexes); //< child joints of links
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
