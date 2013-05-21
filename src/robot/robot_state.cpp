#include "robot_state.h"
//#define NDEBUG
#include <assert.h>
#define DEBUG
#define MODULE_NAME "robot-state"
#include "utils/debug_utils.h"

#include <boost/algorithm/string.hpp>

#include <utils/data_paths.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>


using namespace Eigen;
using namespace kinematics;
using namespace std;

namespace robot {
    
    string robot_state_t::g_ros_prefix;
    bool robot_state_t::g_init = false;
    
    map<int, std::string> robot_state_t::g_r2s;
    map<int, std::string> robot_state_t::g_d2s;
    map<std::string, int> robot_state_t::g_s2r;
    map<std::string, int> robot_state_t::g_s2d;
    map<int, int> robot_state_t::g_r2d;
    map<int, int> robot_state_t::g_d2r;

    vector<int> robot_state_t::g_d_limb[NUM_LIMBS];
    vector<int> robot_state_t::g_r_limb[NUM_LIMBS];

    void robot_state_t::static_init(Skeleton *robot, map<int, string> r2s, vector<string> l2s[NUM_LIMBS]) 
    {

        for(int i_r=0; i_r < r2s.size(); i_r++) {
            
            // Find mapping from dart name to index
            vector<string> tokens; // remove ROS headers
            boost::split(tokens, r2s[i_r], boost::is_any_of(":"));
            string name = tokens.back();
            int i_d = find_d_name(name, robot);
            assert(i_d != -1);

            // fill maps
            g_r2s[i_r] = r2s[i_r];
            g_d2s[i_d] = name;
            g_d2r[i_d] = i_r;
            g_r2d[i_r] = i_d;
            g_s2r[g_r2s[i_r]] = i_r;
            g_s2d[g_d2s[i_d]] = i_d;
            
        }

        // ROS prefix
        vector<string> tokens;
        boost::split(tokens, r2s[0], boost::is_any_of(":"));
        g_ros_prefix = "";
        string name = tokens.back();
        int found = r2s[0].find(name);
        g_ros_prefix = r2s[0].substr(0,found);

        DEBUG_PRINT("g_ros_prefix %s\n", g_ros_prefix.c_str());
        
        // Map limbs to indexes
        for(int i=0; i < NUM_LIMBS; i++) {
            limb_init(g_d_limb[i], "", l2s[i], g_s2d);
            limb_init(g_r_limb[i], g_ros_prefix, l2s[i], g_s2r);
        }

        // Verify mapping
        // ROS map
        for(auto iter = g_s2r.begin(); iter != g_s2r.end(); ++iter) {
            DEBUG_PRINT("s2r %17s --> %d\n", iter->first.c_str(), iter->second);
        }
        for(auto iter = g_r2s.begin(); iter != g_r2s.end(); ++iter) {
            DEBUG_PRINT("r2s %17d --> %s\n", iter->first, iter->second.c_str());
        }
        // DART map
        for(auto iter = g_s2d.begin(); iter != g_s2d.end(); ++iter) {
            DEBUG_PRINT("s2d %17s --> %d\n", iter->first.c_str(), iter->second);
        }
        for(auto iter = g_d2s.begin(); iter != g_d2s.end(); ++iter) {
            DEBUG_PRINT("d2s %17d --> %s\n", iter->first, iter->second.c_str());
        }
        // LIMB maps
        for(int i=0; i < NUM_LIMBS; ++i) {
            DEBUG_PRINT("r limb %d\n", i);
            for(int j=0; j < g_r_limb[i].size(); ++j) {
                DEBUG_PRINT("%2d <--> %s\n", g_r_limb[i][j], g_r2s[g_r_limb[i][j]].c_str());
            }
        }
        for(int i=0; i < NUM_LIMBS; ++i) {
            DEBUG_PRINT("d limb %d\n", i);
            for(int j=0; j < g_d_limb[i].size(); ++j) {
                DEBUG_PRINT("%2d <--> %s\n", g_d_limb[i][j], g_d2s[g_d_limb[i][j]].c_str());
            }
        }
        
        g_init = true;
    }
    
    void robot_state_t::limb_init(vector<int>& limb, const string& prefix, 
                                  const vector<string>& names, map<string, int>& mmap) 
    {
        for(int i=0; i < names.size(); i++) {
            limb.push_back(mmap[prefix + names[i]]);
        }
    }

    int robot_state_t::find_d_name(string name, Skeleton *robot)
    {
        for(int i=0; i < robot->getNumJoints(); i++)
            if(name == robot->getJoint(i)->getName())
                return robot->getJoint(i)->getFirstDofIndex();
        return -1;
    }

}
