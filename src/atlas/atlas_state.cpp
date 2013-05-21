#include "atlas_state.h"
//#define NDEBUG
#include <assert.h>
#define DEBUG
#define MODULE_NAME "atlas-state"
#include "utils/debug_utils.h"

#include <kinematics/Skeleton.h>

using namespace Eigen;
using namespace kinematics;
using namespace std;

namespace atlas {
    
    void atlas_state_t::init(Skeleton *_robot) {
        robot = _robot;

        dofs = robot->getPose();
        dofs.setZero();

        if(robot_state_t::g_init)
            return;

        map<int, string> ros2s;
        ros2s[0] = "atlas::back_lbz";
        ros2s[1] = "atlas::back_mby";
        ros2s[2] = "atlas::back_ubx";
        ros2s[3] = "atlas::neck_ay";
        ros2s[4] = "atlas::l_leg_uhz";
        ros2s[5] = "atlas::l_leg_mhx";
        ros2s[6] = "atlas::l_leg_lhy";
        ros2s[7] = "atlas::l_leg_kny";
        ros2s[8] = "atlas::l_leg_uay";
        ros2s[9] = "atlas::l_leg_lax";
        ros2s[10] = "atlas::r_leg_uhz";
        ros2s[11] = "atlas::r_leg_mhx";
        ros2s[12] = "atlas::r_leg_lhy";
        ros2s[13] = "atlas::r_leg_kny";
        ros2s[14] = "atlas::r_leg_uay";
        ros2s[15] = "atlas::r_leg_lax";
        ros2s[16] = "atlas::l_arm_usy";
        ros2s[17] = "atlas::l_arm_shx";
        ros2s[18] = "atlas::l_arm_ely";
        ros2s[19] = "atlas::l_arm_elx";
        ros2s[20] = "atlas::l_arm_uwy";
        ros2s[21] = "atlas::l_arm_mwx";
        ros2s[22] = "atlas::r_arm_usy";
        ros2s[23] = "atlas::r_arm_shx";
        ros2s[24] = "atlas::r_arm_ely";
        ros2s[25] = "atlas::r_arm_elx";
        ros2s[26] = "atlas::r_arm_uwy";
        ros2s[27] = "atlas::r_arm_mwx";

        vector<string> limb2s[robot::NUM_LIMBS];
        limb2s[robot::LIMB_L_LEG] = {"l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax"};
        limb2s[robot::LIMB_R_LEG] = {"r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax"};
        limb2s[robot::LIMB_L_ARM] = {"l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx"};
        limb2s[robot::LIMB_R_ARM] = {"r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"};
        limb2s[robot::LIMB_HEAD] = {"neck_ay"};
        limb2s[robot::LIMB_TORSO] = {"back_lbz", "back_mby", "back_ubx"};

        static_init(_robot, ros2s, limb2s);
    }


}
