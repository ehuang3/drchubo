#include "robot_graphics.h"
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

namespace robot {

void robot_graphics_t::renderCOM(Skeleton *_atlas, RenderInterface *_ri) {
	glDisable(GL_CULL_FACE);
    
    Vector3d com = _atlas->getWorldCOM();
    double radius = 0.05;
    
    // COM disk
    _ri->pushMatrix();
    glTranslated(com(0), com(1), 0);
    glColor3d(0.0, 1.0, 0.3);
    QUAD_OBJ_INIT;
    gluDisk(quadObj, 0, radius, 8, 8);
    _ri->popMatrix();

	renderCOM(_atlas->getRoot(), _ri);
}

void robot_graphics_t::renderJoints(Skeleton *_atlas, RenderInterface *_ri) {
	glDisable(GL_CULL_FACE);
	renderJoints(_atlas->getRoot(), _ri, 0);
}

void robot_graphics_t::renderCOM(BodyNode *_node, RenderInterface *_ri) {
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

	glColor3d(1.0, 0.1, 0.2);
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

void robot_graphics_t::renderJoints(BodyNode *_node, RenderInterface *_ri, int _depth) {
	if(!_node)
		return;

	// render self geometry
    Joint *_jointParent = _node->getParentJoint();
    int nt = _jointParent->getNumTransforms();

    if(_depth > 0) {
        // lines?
        glColor3d(1,1,1);
        glLineWidth(2.0);
        glDisable(GL_LIGHTING);
        glBegin(GL_LINES);

        glVertex3d(0, 0, 0);

        Matrix4d locTrans = _jointParent->getLocalTransform();

        // Lines?
        glVertex3d(locTrans(0,3), locTrans(1,3), locTrans(2,3));
        glEnd();

        glEnable(GL_LIGHTING);
    }

	_ri->pushMatrix();
	for(int i=0; i < _jointParent->getNumTransforms(); ++i) {
		_jointParent->getTransform(i)->applyGLTransform(_ri);

//		if(i == _jointParent->getNumTransforms()-1) {
//			// dof transform
//			cout << "BodyNode: " << _node->getName() << endl;
//			cout << "Trfm: " << _jointParent->getTransform(i)->getName() << endl;
//			cout << "Tfrm Type: " << _jointParent->getTransform(i)->getType() << endl;
//			cout << "= \n" << _jointParent->getTransform(i)->getTransform() << endl;
//			cout << "ParentJoint: " << _jointParent->getName() << endl;
//			for(int i=0; i < 3; i++) {
//				cout << "axis " << i << "= \n"
//					 << _jointParent->getAxis(i) << endl;
//			}
//		}
	}

	// axis 0 is joint axis of rotation (i think)
	Vector3d ax = _jointParent->getAxis(0).normalized();
	Vector3d zx = Vector3d::UnitZ();
	Vector3d perp = zx.cross(ax);
	double y = perp.norm();
	double x = zx.dot(ax);
	double ang = atan2(y,x) * 180.0 / M_PI;

	_ri->pushMatrix();

	double radius = 0.02;
	double height = 0.05;

	_ri->rotate(perp, ang);

		_ri->pushMatrix();
		glTranslated(0.0,0.0,-height/2);
		glColor3d(0.0, 0.3, 1.0);
		QUAD_OBJ_INIT;
		gluCylinder(quadObj, radius, radius, height, 8, 8);
		gluDisk(quadObj, 0, radius, 8, 8);
		glTranslated(0.0,0.0,height);
		gluDisk(quadObj, 0, radius, 8, 8);
		_ri->popMatrix();

	//glColor3d(0.0, 1.0, 0.0);
	//_ri->drawCylinder(radius, height);
	_ri->popMatrix();


	// render subtree
	for(int i=0; i < _node->getNumChildJoints(); ++i) {
		BodyNode *child = _node->getChildJoint(i)->getChildNode();
		renderJoints(child, _ri, _depth+1);
	}
	_ri->popMatrix();
}

}
