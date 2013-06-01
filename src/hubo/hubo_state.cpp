#include "hubo_state.h"
//#define NDEBUG
#include <assert.h>
#define DEBUG
#define MODULE_NAME "hubo-state"
#include "utils/debug_utils.h"

#include <kinematics/Skeleton.h>

using namespace Eigen;
using namespace kinematics;
using namespace std;

namespace hubo {

    void hubo_state_t::init(Skeleton *_hubo)
    {
        _robot = _hubo;

        _dofs = _robot->getPose();
        _dofs.setZero();

        if(robot_state_t::g_init)
            return;

        map<int, string> ros2s;
        ros2s[0]= "hubo::LHY";
        ros2s[1] = "hubo::RHY";
        ros2s[2] = "hubo::TSY";
        ros2s[3] = "hubo::LHR";
        ros2s[4] = "hubo::RHR";
        ros2s[5] = "hubo::LSP";
        ros2s[6] = "hubo::NKY";
        ros2s[7] = "hubo::RSP";
        ros2s[8] = "hubo::LHP";
        ros2s[9]  = "hubo::RHP";
        ros2s[10] = "hubo::LSR";
        ros2s[11] = "hubo::NKP";
        ros2s[12] = "hubo::RSR";
        ros2s[13] = "hubo::LKP";
        ros2s[14] = "hubo::RKP";
        ros2s[15] = "hubo::LSY";
        ros2s[16] = "hubo::RSY";
        ros2s[17] = "hubo::LAP";
        ros2s[18] = "hubo::RAP";
        ros2s[19] = "hubo::LEP";
        ros2s[20] = "hubo::REP";
        ros2s[21] = "hubo::LAR";
        ros2s[22] = "hubo::RAR";
        ros2s[23] = "hubo::LWY";
        ros2s[24] = "hubo::RWY";
        ros2s[25] = "hubo::LWP";
        ros2s[26] = "hubo::RWP";
        ros2s[27] = "hubo::LWR";
        ros2s[28] = "hubo::RWR";
        ros2s[29] = "hubo::LF1";
        ros2s[30] = "hubo::LF2";
        ros2s[31] = "hubo::LF3";
        ros2s[32] = "hubo::RF1";
        ros2s[33] = "hubo::RF2";
        ros2s[34] = "hubo::RF3";

        vector<string> limb2s[robot::NUM_LIMBS];

        limb2s[robot::LIMB_L_LEG] = {"LHY", "LHR", "LHP", "LKP", "LAP", "LAR"};
        limb2s[robot::LIMB_R_LEG] = {"RHY", "RHR", "RHP", "RKP", "RAP", "RAR"};
        limb2s[robot::LIMB_L_ARM] = {"LSP", "LSR", "LSY", "LEP", "LWY", "LWP", "LWR"};
        limb2s[robot::LIMB_R_ARM] = {"RSP", "RSR", "RSY", "REP", "RWY", "RWP", "RWR"};
        limb2s[robot::LIMB_HEAD] = {"NKY", "NKP"};
        limb2s[robot::LIMB_TORSO] = {"TSY"};

        static_init(_robot, ros2s, limb2s);
    }

}
