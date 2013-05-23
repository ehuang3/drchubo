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
    
    // statics for tracking joint mappings
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

        // Map limbs to indexes
        for(int i=0; i < NUM_LIMBS; i++) {
            limb_init(g_d_limb[i], "", l2s[i], g_s2d);
            limb_init(g_r_limb[i], g_ros_prefix, l2s[i], g_s2r);
        }

        g_init = true;
    }

    void robot_state_t::print_mappings()
    {
        DEBUG_PRINT("g_ros_prefix %s\n", g_ros_prefix.c_str());
        // ROS map
        int i = 0;
        for(auto iter = g_s2r.begin(); iter != g_s2r.end(); ++iter) {
            DEBUG_PRINT("s2r %2d: %17s --> %d\n", i++, iter->first.c_str(), iter->second);
        }
        i = 0;
        for(auto iter = g_r2s.begin(); iter != g_r2s.end(); ++iter) {
            DEBUG_PRINT("r2s %2d: %17d --> %s\n", i++, iter->first, iter->second.c_str());
        }
        // DART map
        i = 0;
        for(auto iter = g_s2d.begin(); iter != g_s2d.end(); ++iter) {
            DEBUG_PRINT("s2d %2d: %17s --> %d\n", i++, iter->first.c_str(), iter->second);
        }
        i = 0;
        for(auto iter = g_d2s.begin(); iter != g_d2s.end(); ++iter) {
            DEBUG_PRINT("d2s %2d: %17d --> %s\n", i++, iter->first, iter->second.c_str());
        }
        // ROS-DART map
        i = 0;
        for(auto iter = g_r2d.begin(); iter != g_r2d.end(); ++iter) {
            DEBUG_PRINT("r2d %2d: %17s --> %s\n", i++, g_r2s[iter->first].c_str(), g_d2s[iter->second].c_str());
        }
        i = 0;
        for(auto iter = g_d2r.begin(); iter != g_d2r.end(); ++iter) {
            DEBUG_PRINT("d2r %2d: %17s --> %s\n", i++, g_d2s[iter->first].c_str(), g_r2s[iter->second].c_str());
        }
        // LIMB maps
        for(int i=0; i < NUM_LIMBS; ++i) {
            DEBUG_PRINT("r limb %d\n", i);
            for(int j=0; j < g_r_limb[i].size(); ++j) {
                DEBUG_PRINT("%d: %2d <--> %s\n", j, g_r_limb[i][j], g_r2s[g_r_limb[i][j]].c_str());
            }
        }
        for(int i=0; i < NUM_LIMBS; ++i) {
            DEBUG_PRINT("d limb %d\n", i);
            for(int j=0; j < g_d_limb[i].size(); ++j) {
                DEBUG_PRINT("%d: %2d <--> %s\n", j, g_d_limb[i][j], g_d2s[g_d_limb[i][j]].c_str());
            }
        }
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

    void robot_state_t::set_body(const Matrix4d& Twb) {
        // you spin me right round, Talt-Bryan, Z1Y2X3
        double u1, u2, u3;
        double ZERO_TOL = 1e-9;
        u2 = atan2( -Twb(2,0), sqrt( Twb(0,0)*Twb(0,0) + Twb(1,0)*Twb(1,0) ) );
        if( fabs( fabs( u2 ) - M_PI/2 ) < ZERO_TOL ) {
            u1 = -atan2( Twb(0,1), Twb(0,2) );
            u3 = 0;
        } else {
            u1 = atan2( Twb(1,0)/cos(u2), Twb(0,0)/cos(u2) );
            u3 = atan2( Twb(2,1)/cos(u2), Twb(2,2)/cos(u2) );
        }
        _dofs.block<3,1>(0,0) = Twb.block<3,1>(0,3);
        _dofs(3) = u1; // yaw
        _dofs(4) = u2; // pitch
        _dofs(5) = u3; // roll
    }

    void robot_state_t::get_body(Matrix4d& Twb) {
        Isometry3d _Twb;
        get_body(_Twb);
        Twb = _Twb.matrix();
    }

    void robot_state_t::set_body(const Isometry3d& Twb) {
        set_body(Twb.matrix());
    }

    void robot_state_t::get_body(Isometry3d& Twb) {
        Twb = Matrix4d::Identity();
        Twb.translate( _dofs.block<3,1>(0,0) );
        Twb.rotate( AngleAxisd( _dofs(3), Vector3d::UnitZ()) );
        Twb.rotate( AngleAxisd( _dofs(4), Vector3d::UnitY()) );
        Twb.rotate( AngleAxisd( _dofs(5), Vector3d::UnitX()) );
    }

    void robot_state_t::set_manip(const VectorXd& q, int mi) 
    {
        const vector<int>& mmap = g_d_limb[mi];
        for(int i=0; i < mmap.size(); i++) {
            _dofs(mmap[i]) = q(i);
        }
    }

    void robot_state_t::get_manip(VectorXd& q, int mi)
    {
        const vector<int>& mmap = g_d_limb[mi];
        for(int i=0; i < mmap.size(); ++i) {
            q(i) = _dofs(mmap[i]);
        }
    }

    void robot_state_t::set_r_pose(const Eigen::VectorXd& q)
    {
        for(auto iter = g_r2d.begin(); iter != g_r2d.end(); ++iter) {
            _dofs(iter->second) = q(iter->first);
        }
    }

    void robot_state_t::get_r_pose(VectorXd& q)
    {
        for(auto iter = g_r2d.begin(); iter != g_d2r.end(); ++iter) {
            q(iter->first) = _dofs(iter->second);
        }
    }

    void robot_state_t::get_manip_indexes(vector<int>& indexes, int mi)
    {
        indexes = g_d_limb[mi];
    }

    void robot_state_t::get_dofs(VectorXd& q, const vector<int>& indexes)
    {
        for(int i=0; i < indexes.size(); ++i) {
            q(i) = _dofs(indexes[i]);
        }
    }

    void robot_state_t::set_dofs(const VectorXd& q, const vector<int>& indexes)
    {
        for(int i=0; i < indexes.size(); ++i) {
            _dofs(indexes[i]) = q(i);
        }
    }
    
    void robot_state_t::print_dofs(const vector<int>& indexes)
    {
        for(int i=0; i < indexes.size(); ++i) {
            DEBUG_PRINT("%d %s\n", i, g_d2s[indexes[i]].c_str());
        }
    }
}
