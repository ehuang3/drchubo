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

        void set_floating(const Eigen::Vector6d& q);
        void get_floating(Eigen::Vector6d& q);

        void set_body(const Eigen::Isometry3d& Twb);
        void get_body(Eigen::Isometry3d& Twb);
        
        void set_d_head(const Eigen::VectorXd& q);
        void get_d_head(Eigen::VectorXd& q);
        void set_r_head(const Eigen::VectorXd& q);
        void get_r_head(Eigen::VectorXd& q);
        int num_links_head();

        void set_d_torso(const Eigen::VectorXd& q);
        void get_d_torso(Eigen::VectorXd& q);
        void set_r_torso(const Eigen::VectorXd& q);
        void get_r_torso(Eigen::VectorXd& q);
        int num_links_torso();
        
        void set_d_arm(const Eigen::VectorXd& q);
        void get_d_arm(Eigen::VectorXd& q);
        int num_links_arm();

        void set_d_leg(const Eigen::Vector6d& q);
        void get_d_leg(Eigen::Vector6d& q);
        int num_links_leg();

        // std::vector<int> get_limb_index();

        Eigen::VectorXd& dart_pose();
        Eigen::VectorXd& ros_pose();

        void dart_pose(const Eigen::VectorXd& q);
        void ros_pose(const Eigen::VectorXd& q);

    protected:
        Eigen::VectorXd dofs; //< dofs in DART << perhaps no state.
        kinematics::Skeleton *robot;
        
        static void static_init(kinematics::Skeleton *robot, std::map<int, std::string> ros2s,
                                std::vector<std::string> l2s[NUM_LIMBS]);

        static void limb_init(std::vector<int>& limb, const std::string& prefix,
                              const std::vector<std::string>& names, std::map<std::string, int>& mmap);

        static int find_d_name(std::string name, kinematics::Skeleton *robot);

        // Assume 1 robot type
        // globals
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
