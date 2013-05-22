#pragma once
#include "robot.h"
#include <vector>
#include <map>
#include <string>

namespace kinematics { class Skeleton; }

namespace robot {
    
    class robot_state_t;
    typedef robot_state_t robot_gains_t;

    class robot_state_t {
    public:
        virtual void init(kinematics::Skeleton *_robot) = 0;

        void set_d_body(const Eigen::Isometry3d& Twb);
        void get_d_body(Eigen::Isometry3d& Twb);

        int num_links_head() { return g_d_limb[LIMB_HEAD].size(); }
        int num_links_torso() { return g_d_limb[LIMB_TORSO].size(); }
        int num_links_arm() { return g_d_limb[LIMB_L_ARM].size(); }
        int num_links_leg() { return g_d_limb[LIMB_L_LEG].size(); }
        
        // accepts both ManipIndex and LimbIndex
        void set_manip(const Eigen::VectorXd& q, int mi);
        void get_manip(Eigen::VectorXd& q, int mi);

        Eigen::VectorXd& d_pose() { return _dofs; }

        void set_d_pose(const Eigen::VectorXd& q) { _dofs = q; }
        void get_d_pose(Eigen::VectorXd& q) { q = _dofs; }

        void set_r_pose(const Eigen::VectorXd& q);
        void get_r_pose(Eigen::VectorXd& q);
        
        // accepts both ManipIndex and LimbIndex
        void get_manip_indexes(std::vector<int>& indexes, int mi);

        void get_dofs(Eigen::VectorXd& q, const std::vector<int>& indexes);
        void set_dofs(const Eigen::VectorXd& q, const std::vector<int>& indexes);

        void print_dofs(const std::vector<int>& indexes);

        double& dofs(int i) { return _dofs(i); }
        Eigen::VectorXd& dofs() { return _dofs; }

        kinematics::Skeleton* robot() { return _robot; }
        

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
