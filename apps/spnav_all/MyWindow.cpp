#include <simulation/World.h>
#include <dynamics/SkeletonDynamics.h>
#include <robotics/parser/dart_parser/DartLoader.h>
#include <kinematics/FileInfoSkel.hpp>
#include <utils/data_paths.h>

#include "MyWindow.h"
#include <robot/robot_graphics.h>
#include <renderer/OpenGLRenderInterface.h>
#include <string>
#include <iostream>

#include <utils/robot_configs.h>

#include <pthread.h>

#include <yui/GLFuncs.h>

using namespace Eigen;
using namespace kinematics;
using namespace dynamics;
using namespace std;

//##############################################################################
//### Globals
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
		cerr << "OpenGL: Fatal Error in ATLAS: out of memory." << endl;
}

//##############################################################################
//### PTHREAD START ROUTINE
void* MyWindow::start_routine(void *args)
{
    MyWindow *window = ((MyWindow *) args);

	// 1. Load a local copy of Atlas
	DartLoader dart_loader;
    simulation::World *mWorld = dart_loader.parseWorld(VRC_DATA_PATH ROBOT_URDF);
    robotSkel = mWorld->getSkeleton("atlas");
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

void MyWindow::drawSkels() {
    // 1. Pretend to understand GL stuff
    glMatrixMode(GL_MODELVIEW);
	//glDisable (GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//glDisable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);

	glShadeModel(GL_SMOOTH);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glEnable(GL_MULTISAMPLE_ARB);

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glHint(GL_FOG_HINT,GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);
    glFrontFace(GL_CCW);
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

    // 1.5 Prepare Atlas
    robotSkel->setPose( current_state->dart_pose() );

    // 2. Render the ground
    // move ground underneath foot
    VectorXd dofs = ground->getPose();
    dofs.block<2,1>(0,0) = robotSkel->getPose().block<2,1>(0,0); // centers ground at feet
    ground->setPose(dofs, true, false);
    ground->draw(mRI);

    // 2.5 Get ready to render robots
    robot::robot_graphics_t special_effects;

    glClear(GL_DEPTH_BUFFER_BIT);
    // 3. Render the current state
    robotSkel->setPose( current_state->dart_pose() );
    robotSkel->draw(mRI, Vector4d(0.5, 0.5, 0.5, 0.5), false); // he's the grey one

    // 4. Render the target state
    robotSkel->setPose( target_state->dart_pose() );
    robotSkel->draw(mRI, Vector4d(0, 1, 0, 0.7), false); // he's green and transparents

    // 5. Render COM dots and disks
    Vector3d com;
    
    // 5.1 Purple target COMs
    glClear(GL_DEPTH_BUFFER_BIT);

    glColor4d(1,0,1,0.8);
    // The purple dot of com
    com = robotSkel->getWorldCOM();
    glPushMatrix();
    glTranslated(com(0), com(1), com(2));
    gluSphere(quadObj, radius, 16, 16);
    glPopMatrix();
    // The purple disk of com
    glPushMatrix();
    glTranslated(com(0), com(1), foot2ground);
    gluDisk(quadObj, 0, radius, 16, 16);
    glPopMatrix();

    // 5.2 Red current COMs
    glColor4d(1,0,0,0.8);
    glClear(GL_DEPTH_BUFFER_BIT);
    robotSkel->setPose( current_state->dart_pose() );
    // The red dot of com
    com = robotSkel->getWorldCOM();
    glPushMatrix();
    glTranslated(com(0), com(1), com(2));
    gluSphere(quadObj, radius, 16, 16);
    glPopMatrix();
    // The red disk of com
    glPushMatrix();
    glTranslated(com(0), com(1), foot2ground);
    gluDisk(quadObj, 0, radius, 16, 16);
    glPopMatrix();

    // 6. Render goal
    Eigen::Isometry3d goalTarget = *goal;
    glPushMatrix();
    glMultMatrixd(goalTarget.data());
    Eigen::Vector3d base = Eigen::Vector3d::Zero();
    glColor4d(1,0,0,0.5);
    yui::drawArrow3D(base, Eigen::Vector3d::UnitX(), 0.2, 0.01, 0.02);
    glColor4d(0,1,0,0.5);
    yui::drawArrow3D(base, Eigen::Vector3d::UnitY(), 0.2, 0.01, 0.02);
    glColor4d(0,0,1,0.5);
    yui::drawArrow3D(base, Eigen::Vector3d::UnitZ(), 0.2, 0.01, 0.02);
    glPopMatrix();

    // 99. Post up and go home
    glutPostRedisplay();
}

