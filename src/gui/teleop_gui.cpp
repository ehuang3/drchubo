// DART includes
#include <simulation/World.h>
#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <kinematics/FileInfoSkel.hpp>
#include <yui/GLFuncs.h>
#include <renderer/OpenGLRenderInterface.h>

// VRC includes
#include "teleop_gui.h"
#include <utils/data_paths.h>
#include <robot/robot_graphics.h>
#include <utils/robot_configs.h>
#include <robot/robot.h>

// STL includes
#include <string>
#include <iostream>

// PTHREAD includes
#include <pthread.h>


using namespace Eigen;
using namespace kinematics;
using namespace dynamics;
using namespace std;
//############################################################
//### Globals
//############################################################
double foot2ground; //< distance the ground should spawn below the ankle
dynamics::SkeletonDynamics *ground;
dynamics::SkeletonDynamics *robotSkel;

double radius = 0.05; //< com radius

static GLUquadricObj *quadObj;
#define QUAD_OBJ_INIT { if(!quadObj) initQuadObj(); }
static void initQuadObj(void)
{
	quadObj = gluNewQuadric();
	if(!quadObj)
		cerr << "OpenGL: Fatal Error: out of memory." << endl;
}

namespace gui {
//############################################################
//### PTHREAD START ROUTINE
//############################################################
    void* teleop_gui_t::start_routine(void *args)
    {
        teleop_gui_t *window = ((teleop_gui_t *) args);

        // 1. Load a local copy of Atlas
        DartLoader dart_loader;
        simulation::World *mWorld = dart_loader.parseWorld(VRC_DATA_PATH ROBOT_URDF);
        robotSkel = mWorld->getSkeleton(ROBOT_NAME);
        VectorXd dofs = robotSkel->getPose();
        robotSkel->setPose(dofs);

        // 2. Load the ground
        FileInfoSkel<SkeletonDynamics> model;
        model.loadFile(VRC_DATA_PATH"/models/skel/ground1.skel", SKEL);
        ground = dynamic_cast<SkeletonDynamics *>(model.getSkel());
        ground->setName("ground");
        mWorld->addSkeleton(ground);
        dofs = ground->getPose();
        ground->setPose(dofs);
    
        // 3. Zero atlas's feet on ground
        kinematics::BodyNode* left_foot = robotSkel->getNode(ROBOT_LEFT_FOOT);
        kinematics::Shape* left_shoe = left_foot->getCollisionShape();
        // height from ankle to base of shoe
        foot2ground = left_shoe->getDim()(2)/2 - left_shoe->getTransform().matrix()(2,3);
        // height from ground center to surface
        double ground_z = ground->getNode("ground1_root")->getCollisionShape()->getDim()(2)/2;
        // invert to get height below left ankle
        foot2ground *= -1;
        // set into ground
        dofs(2) = foot2ground - ground_z;
        ground->setPose(dofs);

        // 4. Initialize GL stuff
        QUAD_OBJ_INIT;

        // 4. Finish init and 
        window->setWorld(mWorld);
        // fake argc, argv
        char *argv[1];
        int argc = 1;
        argv[0] = "robot-viz";
        // gl stuff
        glutInit(&argc, argv);
        window->initWindow(640, 480, "Robot Viz");
        glutMainLoop();

        return args;
    }

    void teleop_gui_t::drawSkels() {
        //############################################################
        //### GL initialization
        //############################################################
        // 1. Pretend to understand GL stuff
        glMatrixMode(GL_MODELVIEW);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_NORMALIZE);
        //glEnable(GL_COLOR_MATERIAL);
        //glDisable(GL_CULL_FACE);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);

        // 
        glShadeModel(GL_SMOOTH);
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        glEnable(GL_MULTISAMPLE_ARB);

        glEnable(GL_TEXTURE_2D);
        glEnable(GL_BLEND);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glHint(GL_FOG_HINT,GL_NICEST);
        glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);
        //glFrontFace(GL_CCW);
        glEnable(GL_COLOR_MATERIAL);

        glColor4d(1.0,1.0,1.0,0);

        static float front_mat_shininess[] = {60.0};
        static float front_mat_specular[]  = {0.2, 0.2,  0.2,  1.0};
        static float front_mat_diffuse[]   = {0.5, 0.28, 0.38, 1.0};

        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  front_mat_specular);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   front_mat_diffuse);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);

        glDisable(GL_FOG);

        glDisable(GL_POLYGON_SMOOTH); //< this one sucks

        //############################################################
        //### Initialize robots and parameters
        //############################################################
        // 1. Set up parameters
        robot::robot_state_t* current_state = gui_params->current;
        robot::robot_state_t* target_state = gui_params->target;
        bool draw_limits = gui_params->draw_limits;
        Eigen::Isometry3d goal = *gui_params->goal;
        // 2. Set up state
        robotSkel->setPose( current_state->dart_pose() );
    
        //############################################################
        //### Gl rendering
        //############################################################
        // 1. Render ground
        VectorXd dofs = ground->getPose();
        dofs.block<2,1>(0,0) = robotSkel->getPose().block<2,1>(0,0); // centers ground at feet
        ground->setPose(dofs, true, false);
        ground->draw(mRI);

        // 2. Render robots
        glClear(GL_DEPTH_BUFFER_BIT);
        // current state
        robotSkel->setPose( current_state->dart_pose() );
        robotSkel->draw(mRI, Vector4d(0.5, 0.5, 0.5, 0.5), false); // he's the grey one
        // target state
        glClear(GL_DEPTH_BUFFER_BIT);
        robotSkel->setPose( target_state->dart_pose() );
        // robotSkel->draw(mRI, Vector4d(0, 1, 0, 0.7), false); // he's green and transparents
        render_skel(robotSkel, *target_state, Vector4d(0,1,0,1), false, true);

        // 5. Render COM dots and disks
        Vector3d com;
        double com_radius = 0.05;
        glClear(GL_DEPTH_BUFFER_BIT);
        // target state
        glColor4d(1,0,1,0.8); //< purple 
        render_com_dot(robotSkel, com_radius);
        render_com_disk(robotSkel, com_radius);
        // current state
        glColor4d(1,0,0,0.8);
        glClear(GL_DEPTH_BUFFER_BIT);
        robotSkel->setPose( current_state->dart_pose() );
        render_com_dot(robotSkel, com_radius);
        render_com_disk(robotSkel, com_radius);

        // 6. Render goal
        render_xform_arrows(goal);

        // 99. Post up redisplay and go home
        glutPostRedisplay();
    }

//############################################################
//### Render functions
//############################################################
    void teleop_gui_t::render_skel(kinematics::Skeleton* robot,
                                   const robot::robot_state_t& state,
                                   Eigen::Vector4d color,
                                   bool use_default_color,
                                   bool draw_limits,
                                   int target_joint)
    {
        kinematics::BodyNode *root = robot->getRoot();
        glPushMatrix();
        // Do self transform
        kinematics::Joint* joint = root->getParentJoint();
        for(int i=0; i < joint->getNumTransforms(); ++i) {
            joint->getTransform(i)->applyGLTransform(mRI);
        }
        for(int i=0; i < root->getNumChildJoints(); ++i)
            render_link(robot, root->getChildJoint(i)->getChildNode(), state, color, 
                        use_default_color, draw_limits, target_joint);
        glPopMatrix();
    }

    void teleop_gui_t::render_link(kinematics::Skeleton* robot,
                                   kinematics::BodyNode *link,
                                   const robot::robot_state_t& state,
                                   Eigen::Vector4d color,
                                   bool use_default_color,
                                   bool draw_limits,
                                   int target_joint)
    {
        if(!link)
            return;
    
        glPushMatrix();
        // Do self transform
        kinematics::Joint* joint = link->getParentJoint();
        for(int i=0; i < joint->getNumTransforms(); ++i) {
            joint->getTransform(i)->applyGLTransform(mRI);
        }
        kinematics::Shape* shape = link->getVisualizationShape();
        if(shape && !draw_limits) {
            glPushMatrix();
            shape->draw(mRI, color, use_default_color);
            glPopMatrix();
        }
        if(shape && draw_limits) {
            render_limits(robot, link, state, color, Vector4d(1,0,0,color[3]));
        }
        // Render subtree
        for(int i=0; i < link->getNumChildJoints(); ++i) {
            render_link(robot, link->getChildJoint(i)->getChildNode(), state, color, 
                        use_default_color, draw_limits, target_joint);
        }
        glPopMatrix();
    }

    Eigen::Vector4d lerp(Eigen::Vector4d& v0, Eigen::Vector4d& v1, double t)
    {
        if (t < 0) return v0;
        if (t > 1) return v1;
        return (1-t)*v0 + t*v1;
    }

    Eigen::Vector4d smoothstep(Eigen::Vector4d& v0, Eigen::Vector4d& v1, double t)
    {
        double tt = -2*t*t*t + 3*t*t;
        return lerp(v0, v1, t);
    }

    void teleop_gui_t::render_limits(kinematics::Skeleton *robot,
                                     kinematics::BodyNode *link,
                                     const robot::robot_state_t& state,
                                     Eigen::Vector4d color_ok,
                                     Eigen::Vector4d color_limit)
    {
        glPushMatrix();
        // Color to render at
        Eigen::Vector4d color = color_ok;
        // What limit are we at?
        kinematics::Joint *joint = link->getParentJoint();
        int i = state.get_index(joint->getName());
        if(i != -1) {
            double dof = state.dofs(i);
            Eigen::Vector2d limits = state.get_limits(i);
            if(dof < limits[0] || limits[1] < dof) {
                std::vector<int> jind;
                jind.push_back(i);
                // state.print_limits(jind);
                // std::cout << "\tvalue" << dof << std::endl;
            }

            double middle = (limits[1]+limits[0])/2;

            double t = fabs(dof - middle)/(fabs(middle-limits[0]));

            color = smoothstep(color_ok, color_limit, t);
        }
        // Render shape w/ color
        kinematics::Shape *shape = link->getVisualizationShape();
        if(shape) {
            shape->draw(mRI, color, false);
        }
        glPopMatrix();
    }

    void teleop_gui_t::render_com_disk(kinematics::Skeleton* robot,
                                       double radius)
    {
        Vector3d com = robot->getWorldCOM();
        glPushMatrix();
        glTranslated(com(0), com(1), foot2ground);
        gluSphere(quadObj, radius, 16, 16);
        glPopMatrix();    
    }

    void teleop_gui_t::render_com_dot(kinematics::Skeleton* robot,
                                      double radius)
    {
        Vector3d com = robot->getWorldCOM();
        glPushMatrix();
        glTranslated(com(0), com(1), com(2));
        gluSphere(quadObj, radius, 16, 16);
        glPopMatrix();    
    }

    void teleop_gui_t::render_xform_arrows(Eigen::Isometry3d xform)
    {
        glPushMatrix();
        glMultMatrixd(xform.data());
        Eigen::Vector3d base = Eigen::Vector3d::Zero();
        glColor4d(1,0,0,0.9);
        yui::drawArrow3D(base, Eigen::Vector3d::UnitX(), 0.2, 0.01, 0.02);
        glColor4d(0,1,0,0.9);
        yui::drawArrow3D(base, Eigen::Vector3d::UnitY(), 0.2, 0.01, 0.02);
        glColor4d(0,0,1,0.9);
        yui::drawArrow3D(base, Eigen::Vector3d::UnitZ(), 0.2, 0.01, 0.02);
        glPopMatrix();
    }

}

