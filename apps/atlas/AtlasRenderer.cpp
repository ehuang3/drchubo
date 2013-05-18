#include "AtlasRenderer.h"
#include <renderer/LoadOpengl.h>
#include <Eigen/Dense>
#include <renderer/RenderInterface.h>
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Joint.h>
#include <iostream>
#include <kinematics/Transformation.h>

using namespace dynamics;
using namespace kinematics;
using namespace Eigen;
using namespace std;

namespace atlas {

void recursiveDraw(BodyNode * _node, renderer::RenderInterface* _ri) {

    if (!_ri) return;

    Joint *jointParent = _node->getParentJoint();

    glDisable(GL_CULL_FACE);

	_ri->pushMatrix();

	// render the self geometry
	for (int i = 0; i < jointParent->getNumTransforms(); i++) {


	    jointParent->getTransform(i)->applyGLTransform(_ri);

	    if(i == jointParent->getNumTransforms() - 1) {

			_ri->pushMatrix();

			_ri->drawCylinder(0.05, 0.1);

			_ri->popMatrix();

		}
	}

	_ri->pushMatrix();


	_ri->popMatrix();

	// render the subtree
	for (int i = 0; i < _node->getNumChildJoints(); i++){

		recursiveDraw(_node->getChildJoint(i)->getChildNode(), _ri);

	}

	_ri->popMatrix();
}

void AtlasRenderer::drawSkeleton(SkeletonDynamics *_atlas, renderer::RenderInterface* _ri) {
	glMatrixMode(GL_MODELVIEW);

	recursiveDraw(_atlas->getRoot(), _ri);

	// CoM spheres
	
	// for(int i=0; i < _atlas->getNumNodes(); ++i) {
	// 	BodyNode *node = _atlas->getNode(i);

 //    	Affine3d pose;
 //    	pose.matrix() = node->getWorldTransform();

 //    	Vector3d offset = node->getLocalCOM();


 //    	glPushMatrix();
 //    	///
 //    	cout << node->getName() << endl;
 //    	Joint *jointParent = node->getParentJoint();
 //    	for (int j = 0; j < jointParent->getNumTransforms(); j++) {
 //    		cout << "joint " << j << endl;
 //            cout << jointParent->getTransform(j)->getName() << endl;

 //            if(j < jointParent->getNumTransforms() - 1) {

 //            	jointParent->getTransform(j)->applyGLTransform(_ri);

 //            } else {



 //            	jointParent->getTransform(j)->applyGLTransform(_ri);

 //            	glPushMatrix();
 //            	_ri->drawCylinder(0.1, 0.1);
 //            	glPopMatrix();
 //            }
 //        }
 //        glPopMatrix();

 //    	glPushMatrix();
 //    	glMultMatrixd(pose.data());

	// 	glPushMatrix();
 //    	_ri->translate(offset);
	// 	glColor3ub(191,191,191);
 //    	_ri->drawEllipsoid(Vector3d(0.03, 0.03, 0.03));
 //    	glPopMatrix();

 //    	glPopMatrix();
	// }

}

}
