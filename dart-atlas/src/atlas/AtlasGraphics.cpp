#include "AtlasGraphics.h"
#include <Eigen/Dense>
#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Joint.h>
#include <kinematics/Transformation.h>
#include <renderer/RenderInterface.h>
#include <iostream>

using namespace kinematics;
using namespace renderer;
using namespace Eigen;
using namespace std;

static GLUquadricObj *quadObj;
#define QUAD_OBJ_INIT { if(!quadObj) initQuadObj(); }
static void initQuadObj(void)
{
	quadObj = gluNewQuadric();
	if(!quadObj)
		cerr << "OpenGL: Fatal Error in ATLAS: out of memory." << endl;
}

namespace atlas {

void AtlasGraphics::renderCOM(Skeleton *_atlas, RenderInterface *_ri) {
	glDisable(GL_CULL_FACE);
	renderCOM(_atlas->getRoot(), _ri);
}

void AtlasGraphics::renderJoints(Skeleton *_atlas, RenderInterface *_ri) {
	glDisable(GL_CULL_FACE);
	renderJoints(_atlas->getRoot(), _ri);
}

void AtlasGraphics::renderCOM(BodyNode *_node, RenderInterface *_ri) {
	if(!_node)
		return;

	_ri->pushMatrix();
	// render self geometry
	Joint *_jointParent = _node->getParentJoint();
	for(int i=0; i < _jointParent->getNumTransforms(); ++i) {
		_jointParent->getTransform(i)->applyGLTransform(_ri);
	}

	Vector3d com_off = _node->getLocalCOM();
	double mass = _node->getMass();

	mass = mass / 10 * 0.05;

	glColor3d(1.0, 0.0, 0.0);
	_ri->pushMatrix();
	_ri->translate(com_off);
	_ri->drawEllipsoid(Vector3d(1,1,1) * mass);
	_ri->popMatrix();


	// render subtree
	for(int i=0; i < _node->getNumChildJoints(); ++i) {
		renderCOM(_node->getChildJoint(i)->getChildNode(), _ri);
	}
	_ri->popMatrix();
}

void AtlasGraphics::renderJoints(BodyNode *_node, RenderInterface *_ri) {
	if(!_node)
		return;

	_ri->pushMatrix();
	// render self geometry
	Joint *_jointParent = _node->getParentJoint();
	int nt = _jointParent->getNumTransforms();
	for(int i=0; i < _jointParent->getNumTransforms(); ++i) {
		_jointParent->getTransform(i)->applyGLTransform(_ri);

//		if(i == _jointParent->getNumTransforms()-1) {
//			// dof transform
//			cout << "BodyNode: " << _node->getName() << endl;
//			cout << "Trfm: " << _jointParent->getTransform(i)->getName() << endl;
//			cout << "Tfrm Type: " << _jointParent->getTransform(i)->getType() << endl;
//			cout << "= \n" << _jointParent->getTransform(i)->getTransform() << endl;
//		}
	}



	_ri->pushMatrix();

	double radius = 0.02;
	double height = 0.05;

	Vector3d axis;
	switch (_jointParent->getTransform(nt-1)->getType()) {
	case Transformation::TransFormType::T_ROTATEX:
		axis << 0, 1, 0;
		break;
	case Transformation::TransFormType::T_ROTATEY:
		axis << 1, 0, 0;
		break;
	case Transformation::TransFormType::T_ROTATEZ:
		axis << 0, 0, 1;
		break;
	}

	_ri->rotate(axis, 90);

		_ri->pushMatrix();
		glTranslated(0.0,0.0,-height/2);
		glColor3d(0.0, 0.0, 1.0);
		QUAD_OBJ_INIT;
		gluCylinder(quadObj, radius, radius, height, 16, 16);
		gluDisk(quadObj, 0, radius, 16, 16);
		glTranslated(0.0,0.0,height);
		gluDisk(quadObj, 0, radius, 16, 16);
		_ri->popMatrix();

	//glColor3d(0.0, 1.0, 0.0);
	//_ri->drawCylinder(radius, height);
	_ri->popMatrix();


	// render subtree
	for(int i=0; i < _node->getNumChildJoints(); ++i) {
		renderJoints(_node->getChildJoint(i)->getChildNode(), _ri);
	}
	_ri->popMatrix();
}

}
