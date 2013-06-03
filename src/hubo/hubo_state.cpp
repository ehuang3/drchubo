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
        ros2s[0]= "drchubo::LHY";
        ros2s[1] = "drchubo::RHY";
        ros2s[2] = "drchubo::TSY";
        ros2s[3] = "drchubo::LHR";
        ros2s[4] = "drchubo::RHR";
        ros2s[5] = "drchubo::LSP";
        ros2s[6] = "drchubo::NKY";
        ros2s[7] = "drchubo::RSP";
        ros2s[8] = "drchubo::LHP";
        ros2s[9]  = "drchubo::RHP";
        ros2s[10] = "drchubo::LSR";
        ros2s[11] = "drchubo::NKP";
        ros2s[12] = "drchubo::RSR";
        ros2s[13] = "drchubo::LKP";
        ros2s[14] = "drchubo::RKP";
        ros2s[15] = "drchubo::LSY";
        ros2s[16] = "drchubo::RSY";
        ros2s[17] = "drchubo::LAP";
        ros2s[18] = "drchubo::RAP";
        ros2s[19] = "drchubo::LEP";
        ros2s[20] = "drchubo::REP";
        ros2s[21] = "drchubo::LAR";
        ros2s[22] = "drchubo::RAR";
        ros2s[23] = "drchubo::LWY";
        ros2s[24] = "drchubo::RWY";
        ros2s[25] = "drchubo::LWP";
        ros2s[26] = "drchubo::RWP";
        ros2s[27] = "drchubo::LWR";
        ros2s[28] = "drchubo::RWR";
        ros2s[29] = "drchubo::LF1";
        ros2s[30] = "drchubo::LF2";
        ros2s[31] = "drchubo::LF3";
        ros2s[32] = "drchubo::RF1";
        ros2s[33] = "drchubo::RF2";
        ros2s[34] = "drchubo::RF3";

        vector<string> limb2s[robot::NUM_LIMBS];

        limb2s[robot::LIMB_L_LEG] = {"LHY", "LHR", "LHP", "LKP", "LAP", "LAR"};
        limb2s[robot::LIMB_R_LEG] = {"RHY", "RHR", "RHP", "RKP", "RAP", "RAR"};
        limb2s[robot::LIMB_L_ARM] = {"LSP", "LSR", "LSY", "LEP", "LWY", "LWP"};//, "LWR"};
        limb2s[robot::LIMB_R_ARM] = {"RSP", "RSR", "RSY", "REP", "RWY", "RWP"};//, "RWR"};
        limb2s[robot::LIMB_HEAD] = {"NKY", "NKP"};
        limb2s[robot::LIMB_TORSO] = {"TSY"};
        limb2s[robot::LIMB_L_HAND] = {"LF1", "LF2", "LF3"};
        limb2s[robot::LIMB_R_HAND] = {"RF1", "RF2", "RF3"};

        static_init(_robot, ros2s, limb2s);
    }

}
