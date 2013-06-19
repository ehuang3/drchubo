#pragma once
// DART includes
#include <cstdio>
#include <stdarg.h>
#include "yui/Win3D.h"
#include "simulation/SimWindow.h"
// VRC includes
#include <robot/robot.h>
#include <robot/robot_state.h>

namespace dynamics{
	class SkeletonDynamics;
	class ContactDynamics;
}

namespace gui {

    struct params_t {
        // Teleoperation Mode
        robot::TeleopMode teleop_mode;
        // Color joint limits
        bool draw_limits;
        // COM stuff always true
        bool com_projection;

        // Target transforms of end effectors
        Eigen::Isometry3d end_effectors[robot::NUM_MANIPULATORS];

        // Goal isometry
        Eigen::Isometry3d* goal;

        // Global transforms of end effectors
        Eigen::Isometry3d *Tf_global_manips;

        // Target index of single joint manipulation
        int target_joint; //< in dart convention
        // Robot states
        robot::robot_state_t *current;
        robot::robot_state_t *target;
        // Centering function, should place robot in world frame
        void (*orientation_func)(robot::robot_state_t& robot);
    };

    class teleop_gui_t : public simulation::SimWindow {
    public:
        /// The constructor - set the position of the skeleton
    	teleop_gui_t(): SimWindow(), key(-1) {
            mTrans[1] = 0.f;
            mZoom = 0.3;
        }
        virtual ~teleop_gui_t() {}
        
        // pthread hook
        static void* start_routine(void *arg);
        
        //############################################################
        //### Render functions
        //############################################################
        virtual void drawSkels(); //< Main hook from SimWindow/GL
        // Custom effects
        virtual void render_skel(kinematics::Skeleton* robot,
                                 const robot::robot_state_t& state, //< will not copy into skel
                                 Eigen::Vector4d color = Eigen::Vector4d(0.5,0.5,0.5,1),
                                 bool use_default_color = true,
                                 bool draw_limits = true,
                                 int target_joint = -1);
        virtual void render_link(kinematics::Skeleton* robot,
                                 kinematics::BodyNode *link,
                                 const robot::robot_state_t& state, //< will not copy into skel
                                 Eigen::Vector4d color = Eigen::Vector4d(0.5,0.5,0.5,1),
                                 bool use_default_color = true,
                                 bool draw_limits = true,
                                 int target_joint = -1);
        virtual void render_limits(kinematics::Skeleton *robot,
                                   kinematics::BodyNode *link,
                                   const robot::robot_state_t& state, //< will not copy into skel
                                   Eigen::Vector4d color_ok,
                                   Eigen::Vector4d color_limit);
        virtual void render_dof_range(kinematics::BodyNode *link,
                                      const robot::robot_state_t& state) {}
        virtual void render_target_joint(kinematics::Joint *, const robot::robot_state_t& state) {}

        virtual void render_com_disk(kinematics::Skeleton *robot,
                                     double radius);
        virtual void render_com_dot(kinematics::Skeleton *robot,
                                    double radius);
        virtual void render_com_links(kinematics::Skeleton *robot,
                                      Eigen::Vector4d color = Eigen::Vector4d(1,0,0,1)) {}
        virtual void render_com_links(kinematics::BodyNode *link,
                                      Eigen::Vector4d color = Eigen::Vector4d(1,0,0,1)) {}
        virtual void render_link_wires(kinematics::Skeleton* robot,
                                       Eigen::Vector4d color = Eigen::Vector4d(1,0,0,1)) {}
        virtual void render_link_wires(kinematics::BodyNode *link,
                                       Eigen::Vector4d color = Eigen::Vector4d(1,0,0,1)) {}
        virtual void render_xform_arrows(Eigen::Isometry3d xform, double alpha = 0.9);
        
                
        params_t* gui_params;

        int key;
        virtual void keyboard(unsigned char _key, int x, int y) {
            key = _key;
        }
    };


}
