/**
 * @file MyWindow.h
 * @author Can Erdogan
 * @date Feb 02, 2013
 * @brief Simple example of a skeleton created from scratch.
 */

#include "MyWindow.h"
#include <robot/robot_graphics.h>

using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

void MyWindow::drawSkels() {
	glDisable (GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//glDisable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);

	glShadeModel(GL_SMOOTH);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glEnable(GL_MULTISAMPLE_ARB);

	for(int i=0; i < mWorld->getNumSkeletons(); i++) {
		mWorld->getSkeleton(i)->draw(mRI);
	}

	robot::robot_graphics_t r_viz;
	for(int i=0; i < mWorld->getNumSkeletons(); i++) {
		glClear(GL_DEPTH_BUFFER_BIT);
		r_viz.renderCOM(mWorld->getSkeleton(i), mRI);
		glClear(GL_DEPTH_BUFFER_BIT);
		r_viz.renderJoints(mWorld->getSkeleton(i), mRI);
	}
}

