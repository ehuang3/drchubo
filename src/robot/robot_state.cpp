#include "robot_state.h"
//#define NDEBUG
#include <assert.h>
#define DEBUG
#define MODULE_NAME "robot-state"
#include "utils/debug_utils.h"

#include <algorithm>
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
        if(g_init)
            return;

        for(auto iter = r2s.begin(); iter != r2s.end(); ++iter) {
            int i_r = iter->first;
            
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
        string ros_name = r2s.begin()->second;
        vector<string> tokens;
        boost::split(tokens, ros_name, boost::is_any_of(":"));
        g_ros_prefix = "";
        string name = tokens.back();
        int found = ros_name.find(name);
        g_ros_prefix = ros_name.substr(0,found);

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
        q.resize(mmap.size(), 1);
        for(int i=0; i < mmap.size(); ++i) {
            q(i) = _dofs(mmap[i]);
        }
    }

    void robot_state_t::set_ros_pose(const Eigen::VectorXd& q)
    {
        for(auto iter = g_r2d.begin(); iter != g_r2d.end(); ++iter) {
            _dofs(iter->second) = q(iter->first);
        }
    }

    void robot_state_t::get_ros_pose(VectorXd& q)
    {
        q.resize(g_r2d.size());
        for(auto iter = g_r2d.begin(); iter != g_r2d.end(); ++iter) {
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
    
    void robot_state_t::print_nodes(const vector<int>& indexes)
    {
        DEBUG_PRINT("Links associated with indexes\n");
        for(int i=0; i < indexes.size(); ++i) {
            DEBUG_PRINT("%2d %7s (child of %s)\n", i,
                        _robot->getDof(indexes[i])->getJoint()->getChildNode()->getName(),
                        g_d2s[indexes[i]].c_str());
        }
    }
    
    void robot_state_t::print_joints(const vector<int>& indexes)
    {
        DEBUG_PRINT("Joints associated with indexes\n");
        for(int i=0; i < indexes.size(); ++i) {
            DEBUG_PRINT("%d %s\n", i, g_d2s[indexes[i]].c_str());
        }
    }

    void robot_state_t::print_limits(const vector<int>& indexes)
    {
        DEBUG_PRINT("Limits associated with indexes\n");
        for(int i=0; i < indexes.size(); ++i) {
            DEBUG_PRINT("%d %15s min: %.4f max: %.4f\n",
                        i, g_d2s[indexes[i]].c_str(),
                        _robot->getDof(indexes[i])->getMin(),
                        _robot->getDof(indexes[i])->getMax());
        }
    }

    void robot_state_t::print_children(const vector<int>& indexes)
    {
        DEBUG_PRINT("Children associated with indexes\n");
        for(int i=0; i < indexes.size(); ++i) {
            BodyNode *node = _robot->getDof(indexes[i])->getJoint()->getChildNode();
            DEBUG_PRINT("%2d %s\n", i, node->getName());
            for(int j=0; j < node->getNumChildJoints(); ++j) {
                DEBUG_PRINT("\t\tjoint %9s --> link %s\n",
                            node->getChildJoint(j)->getName(),
                            node->getChildJoint(j)->getChildNode()->getName());
            }
        }
    }

    void robot_state_t::print_backchain(int i)
    {
        int index = i;
        Joint *joint = robot()->getDof(index)->getJoint();
        DEBUG_PRINT("Backchain of %s\n\t", joint->getName());
        while(joint) {
            BodyNode *node = joint->getParentNode();
            if(node) {
                printf("%s>(%s)>", joint->getName(), node->getName());
                joint = node->getParentJoint();
            } else {
                printf("end of chain");
                joint = 0;
            }
        }
        printf("\n");
    }

    void robot_state_t::print_dependent_dofs(int i)
    {
        int index = i;
        Joint *joint = robot()->getDof(index)->getJoint();
        BodyNode *node = joint->getChildNode();
        DEBUG_PRINT("Dependent dofs of (%s)>%s\n\t", node->getName(), joint->getName());
        for(int i=node->getNumDependentDofs()-1; i > 0; i--) {
            if(g_d2s.count(node->getDependentDof(i)))
                printf("%s>", g_d2s[node->getDependentDof(i)].c_str());
            else {
                printf("??%s", robot()->getDof(node->getDependentDof(i))->getJoint()->getName());
            }
        }
        printf("\n");
    }

    bool robot_state_t::check_limits(const vector<int>& indexes, double zero_tol)
    {
        bool any_exceed = false;
        bool exceed = false;
        for(int i=0; i < indexes.size(); ++i) {
            Dof* dof = _robot->getDof(indexes[i]);
            double val = _dofs(indexes[i]);
            if(val < dof->getMin() || val > dof->getMax()) {
                // Ignore epsilon errors caused by computations
                exceed = (val < dof->getMin() - zero_tol) || (val > dof->getMax() + zero_tol);
                // Do clamp
                double clamp = val < dof->getMin() ? dof->getMin() : dof->getMax();
                // Error message
                if(exceed) {
                    ERROR_PRINT("Joint %s exceeded limit\n", g_d2s[indexes[i]].c_str());
                    ERROR_PRINT("\tvalue: %f\tmin: % .3f max: % .3f\n",
                                val, dof->getMin(), dof->getMax());
                }
                any_exceed = true;
            }
        }
        return !any_exceed;
    }

    bool robot_state_t::clamp_all(bool err_msg, double zero_tol)
    {
        bool no_exceed = true;
        for(auto iter = g_r2d.begin(); iter != g_r2d.end(); ++iter) {
            no_exceed &= clamp_dof(iter->second, err_msg, zero_tol);
        }
        return no_exceed;
    }

    bool robot_state_t::clamp_manip(int mi, bool err_msg, double zero_tol)
    {
        return clamp_indexes(g_d_limb[mi], err_msg, zero_tol);
    }

    bool robot_state_t::clamp_indexes(const vector<int>& indexes, bool err_msg, double zero_tol)
    {
        bool no_exceed = true;
        for(int i=0; i < indexes.size(); ++i) {
            no_exceed &= clamp_dof(indexes[i], err_msg, zero_tol);
        }
        return no_exceed;
    }

    //FIXME: How do I handle joints wrapping around 2PI?    
    bool robot_state_t::clamp_dof(int i, bool err_msg, double zero_tol)
    {
        Dof* dof = _robot->getDof(i);
        double val = _dofs(i);
        bool exceed = false;
        if(val < dof->getMin() || val > dof->getMax()) {
            // Ignore epsilon errors caused by computations
            exceed = (val < dof->getMin() - zero_tol) || (val > dof->getMax() + zero_tol);
            // Do clamp
            double clamp = val < dof->getMin() ? dof->getMin() : dof->getMax();
            _dofs(i) = clamp;
            // Error message
            if(exceed && err_msg) {
                ERROR_PRINT("Joint %s exceeded limit\n", g_d2s[i].c_str());
                ERROR_PRINT("\tvalue: %f\tmin: % .3f max: % .3f\n",
                            val, dof->getMin(), dof->getMax());
                ERROR_PRINT("\tClamping to %.3f\n", clamp);
            }
        }
        return !exceed;
    }

    vector<int> robot_state_t::set_intersect(const vector<int> &a, const vector<int> &b)
    {
        vector<int> A, B;
        A = a;
        B = b;
        std::sort(A.begin(), A.end());
        std::sort(B.begin(), B.end());
        vector<int> ret(A.size() + B.size());
        auto it = std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), ret.begin());
        ret.resize(it-ret.begin());
        // better printing
        std::sort(ret.begin(), ret.end());
        
        return ret;
    }
    
    vector<int> robot_state_t::set_union(const vector<int>&a, const vector<int>&b)
    {
        vector<int> A, B;
        A = a;
        B = b;
        std::sort(A.begin(), A.end());
        std::sort(B.begin(), B.end());
        vector<int> ret(A.size() + B.size());
        auto it = std::set_union(a.begin(), a.end(), b.begin(), b.end(), ret.begin());
        ret.resize(it-ret.begin());
        // better printing
        std::sort(ret.begin(), ret.end());        
        
        return ret;        
    }

    int robot_state_t::get_index(const string& joint)
    {
        return g_s2d[joint];
    }
    
    void robot_state_t::get_branch_indexes(vector<int>& indexes, BodyNode *end_effector)
    {
        indexes.clear();
        for(int i=0; i < end_effector->getNumDependentDofs(); ++i) {
            int dd = end_effector->getDependentDof(i);
            if(g_d2r.count(dd))
                indexes.push_back(end_effector->getDependentDof(i));
        }
    }

    void robot_state_t::get_chain_indexes(vector<int>& indexes, BodyNode* base, BodyNode *end_effector) 
    {
        // Translate to DART indexes
        int b_index = g_s2d[base->getParentJoint()->getName()];
        // 
        bool shared_branch = end_effector->dependsOn(b_index);
        //
        indexes.clear();
        if(shared_branch) {
            // walk backwards b/c order is root->end effector
            int i = end_effector->getNumDependentDofs() - 1;
            int dd;
            for(dd = -1; i > 0; --i) {
                dd = end_effector->getDependentDof(i);
                indexes.push_back(dd);
                if(dd == b_index)
                    break;
            }
        } else {
            vector<int> a, b;
            get_branch_indexes(a, base);
            get_branch_indexes(b, end_effector);
            indexes = set_union(a,b);
        }
    }

    void robot_state_t::get_full_indexes(vector<int>& indexes)
    {
        indexes.clear();
        for(auto iter = g_r2d.begin(); iter != g_r2d.end(); ++iter) {
            indexes.push_back(iter->second);
        }
    }
        
}
