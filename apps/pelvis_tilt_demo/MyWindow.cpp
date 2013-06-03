/**
 * @file MyWindow.h
 * @author Can Erdogan
 * @date Feb 02, 2013
 * @brief Simple example of a skeleton created from scratch.
 */

#include "MyWindow.h"
#include <robot/robot_graphics.h>
#include <renderer/OpenGLRenderInterface.h>
#include <string>

using namespace Eigen;
using namespace kinematics;
using namespace dynamics;
using namespace std;

void MyWindow::drawSkels() {
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

    glDisable(GL_POLYGON_SMOOTH);

    renderer::OpenGLRenderInterface *mGLRI = dynamic_cast<renderer::OpenGLRenderInterface *>(mRI);
	for(int i=0; i < mWorld->getNumSkeletons(); i++) {
        //mGLRI->draw(mWorld->getSkeleton(i), false, false); //FIXME: dart sucks at openGL settings
	    //mWorld->getSkeleton(i)->draw(mRI, Vector4d(0.5, 0.5, 0.5, 1), false);
        mWorld->getSkeleton(i)->draw(mRI);
	}

    glColor4d(0,0,0,1);

	robot::robot_graphics_t r_viz;
    string name = "atlas";
	for(int i=0; i < mWorld->getNumSkeletons(); i++) {
        if(mWorld->getSkeleton(i)->getName() == name) {
            glClear(GL_DEPTH_BUFFER_BIT);
            r_viz.renderCOM(mWorld->getSkeleton(i), mRI);
            glClear(GL_DEPTH_BUFFER_BIT);
            r_viz.renderJoints(mWorld->getSkeleton(i), mRI);
        }
	}

	glFlush();

    glutPostRedisplay();
    //glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
}

