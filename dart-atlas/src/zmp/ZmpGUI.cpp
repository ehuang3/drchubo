#include "ZmpGUI.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "kinematics/Dof.h"
#include "collision/CollisionSkeleton.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include <yui/GLFuncs.h>
#include <stdio.h>
#include <iostream>

#include <renderer/OpenGLRenderInterface.h>
#include <atlas/AtlasGraphics.h>

#include <kinematics/BodyNode.h>
#include <kinematics/Shape.h>
#include <kinematics/ShapeBox.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <robotics/World.h>
#include <utils/AtlasPaths.h>
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/FileInfoSkel.hpp>


using namespace dynamics;
using namespace utils;
using namespace std;

ZmpGUI::ZmpGUI(vector<SkeletonDynamics *> _skels) : Win3D() {

    mBackground[0] = 1.0;
    mBackground[1] = 1.0;
    mBackground[2] = 1.0;
    mBackground[3] = 1.0;

    mSim = false;
    mPlay = false;
    mSimFrame = 0;
    mPlayFrame = 0;
    mShowMarkers = false;

    mPersp = 45.f;
    mTrans[1] = 300.f;

    mGravity = Eigen::Vector3d(0.0, 0.0, -9.8);
    mTimeStep = 1.0/1000.0;
    mForce = Eigen::Vector3d::Zero();

    mSkels = _skels;

    int sumNDofs = 0;
    mIndices.push_back(sumNDofs);
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        int nDofs = mSkels[i]->getNumDofs();
        sumNDofs += nDofs;
        mIndices.push_back(sumNDofs);
    }
    initDyn();
}

void ZmpGUI::initDyn()
{
    mDofs.resize(mSkels.size());
    mDofVels.resize(mSkels.size());

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mDofs[i].resize(mSkels[i]->getNumDofs());
        mDofVels[i].resize(mSkels[i]->getNumDofs());
        mDofs[i].setZero();
        mDofVels[i].setZero();
    }

    // ground
    mDofs[0][0] = 0.00;
    mDofs[0][1] = 0.00;
    mDofs[0][2] = -0.927119 - 0.025;

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        mSkels[i]->initDynamics();
        mSkels[i]->setPose(mDofs[i], true, false);
    }
    // set the ground to be an immobile object; it will still participate in collision
    mSkels[0]->setImmobileState(true);
    // create a collision handler
    mCollisionHandle = new dynamics::ContactDynamics(mSkels, mTimeStep);

    // zero ground at atlas's feet
    double z = mSkels[0]->getNode("ground1_root")->getCollisionShape()->getDim()(2)/2;
    kinematics::BodyNode *LAR = mSkels[1]->getNode("l_foot");
    kinematics::Shape *feet = LAR->getCollisionShape();
    z += feet->getDim()(2)/2 - feet->getTransform().matrix()(2,3);
    z += -LAR->getWorldTransform()(2,3);
    // just a little bit up
    mDofs[0][2] = -z - 0.0001;

    mSkels[0]->setPose(mDofs[0], true, false);
}

VectorXd ZmpGUI::getState() {
    VectorXd state(mIndices.back() * 2);
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        state.segment(start, size) = mDofs[i];
        state.segment(start + size, size) = mDofVels[i];
    }
    return state;
}

VectorXd ZmpGUI::evalDeriv() {
    // compute dynamic equations
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        if (mSkels[i]->getImmobileState()) {
            // need to update node transformation for collision
            mSkels[i]->setPose(mDofs[i], true, false);
        } else {
            // need to update first derivatives for collision
            mSkels[i]->setPose(mDofs[i], false, true);
            mSkels[i]->computeDynamics(mGravity, mDofVels[i], true);
        }
    }
    // compute contact forces
    mCollisionHandle->applyContactForces();

    // compute derivatives for integration
    VectorXd deriv = VectorXd::Zero(mIndices.back() * 2);
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        // skip immobile objects in forward simulation
        if (mSkels[i]->getImmobileState())
            continue;
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        VectorXd qddot = mSkels[i]->getInvMassMatrix() * (-mSkels[i]->getCombinedVector() + mSkels[i]->getExternalForces() + mCollisionHandle->getConstraintForce(i));
        mSkels[i]->clampRotation(mDofs[i], mDofVels[i]);
        deriv.segment(start, size) = mDofVels[i] + (qddot * mTimeStep); // set velocities
        deriv.segment(start + size, size) = qddot; // set qddot (accelerations)
    }
    return deriv;
}

void ZmpGUI::setState(const VectorXd &newState) {
    for (unsigned int i = 0; i < mSkels.size(); i++) {
        int start = mIndices[i] * 2;
        int size = mDofs[i].size();
        mDofs[i] = newState.segment(start, size);
        mDofVels[i] = newState.segment(start + size, size);
    }
}

void ZmpGUI::shakeHips() {

}

void ZmpGUI::displayTimer(int _val)
{
    int numIter = mDisplayTimeout / (mTimeStep * 1000);
    if (mPlay) {
        mPlayFrame += 16;
        if (mPlayFrame >= mBakedStates.size())
            mPlayFrame = 0;
    }else if (mSim) {
        //        static Timer tSim("Simulation");
        for (int i = 0; i < numIter; i++) {
            //            tSim.startTimer();
            static_cast<BodyNodeDynamics*>(mSkels[1]->getNode(0))->addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);
            mIntegrator.integrate(this, mTimeStep);
            //            tSim.stopTimer();
            //            tSim.printScreen();
            shakeHips();
            bake();
            mSimFrame++;
        }
        mForce.setZero();

    }
    glutPostRedisplay();
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void ZmpGUI::draw()
{
    //glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDisable(GL_CULL_FACE);
    glEnable(GL_NORMALIZE);

    if (!mSim) {
        if (mPlayFrame < mBakedStates.size()) {
            for (unsigned int i = 0; i < mSkels.size(); i++) {
                int start = mIndices[i];
                int size = mDofs[i].size();
                mSkels[i]->setPose(mBakedStates[mPlayFrame].segment(start, size), true, false);
            }
            if (mShowMarkers) {
                int sumDofs = mIndices[mSkels.size()];
                int nContact = (mBakedStates[mPlayFrame].size() - sumDofs) / 6;
                for (int i = 0; i < nContact; i++) {
                    Vector3d v = mBakedStates[mPlayFrame].segment(sumDofs + i * 6, 3);
                    Vector3d f = mBakedStates[mPlayFrame].segment(sumDofs + i * 6 + 3, 3) / 10.0;
                    glBegin(GL_LINES);
                    glVertex3f(v[0], v[1], v[2]);
                    glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
                    glEnd();
                    mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
                    mRI->pushMatrix();
                    glTranslated(v[0], v[1], v[2]);
                    mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
                    mRI->popMatrix();
                }
            }
        }
    }else{
        if (mShowMarkers) {
            for (int k = 0; k < mCollisionHandle->getCollisionChecker()->getNumContact(); k++) {
                Vector3d  v = mCollisionHandle->getCollisionChecker()->getContact(k).point;
                Vector3d f = mCollisionHandle->getCollisionChecker()->getContact(k).force / 10.0;
                glBegin(GL_LINES);
                glVertex3f(v[0], v[1], v[2]);
                glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
                glEnd();
                mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
                mRI->pushMatrix();
                glTranslated(v[0], v[1], v[2]);
                mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
                mRI->popMatrix();
            }
        }
    }

    renderer::OpenGLRenderInterface *mGLRI = dynamic_cast<renderer::OpenGLRenderInterface *>(mRI);

    for (unsigned int i = 0; i < mSkels.size(); i++) {
        //mGLRI->draw(mSkels[i], false, true); //FIXME: dart sucks at openGL settings
        mSkels[i]->draw(mRI);
    }

    atlas::AtlasGraphics AG;
    glClear(GL_DEPTH_BUFFER_BIT);
    AG.renderJoints(mSkels[1], mRI);
    glClear(GL_DEPTH_BUFFER_BIT);
    AG.renderCOM(mSkels[1], mRI);

    // display the frame count in 2D text
    char buff[64];
    if (!mSim)
        sprintf(buff, "%d", mPlayFrame);
    else
        sprintf(buff, "%d", mSimFrame);
    string frame(buff);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
    glEnable(GL_LIGHTING);
}

void ZmpGUI::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        // mSim = !mSim; // not needed for this demo
        if (mSim) {
            mPlay = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '1': // upper right force
        mForce[0] = -40;
        cout << "push left" << endl;
        break;
    case '2': // upper right force
        mForce[0] = 40;
        cout << "push right" << endl;
        break;
    case 'p': // playBack
        mPlay = !mPlay;
        if (mPlay) {
            mSim = false;
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        }
        break;
    case '[': // step backward
        if (!mSim) {
            mPlayFrame--;
            if(mPlayFrame < 0)
                mPlayFrame = 0;
            glutPostRedisplay();
        }
        break;
    case ']': // step forwardward
        if (!mSim) {
            mPlayFrame++;
            if(mPlayFrame >= mBakedStates.size())
                mPlayFrame--;
            glutPostRedisplay();
        }
        break;
    case 'v': // show or hide markers
        mShowMarkers = !mShowMarkers;
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}

void ZmpGUI::bake()
{
    int nContact = mCollisionHandle->getCollisionChecker()->getNumContact();
    VectorXd state(mIndices.back() + 6 * nContact);
    for (unsigned int i = 0; i < mSkels.size(); i++)
        state.segment(mIndices[i], mDofs[i].size()) = mDofs[i];
    for (int i = 0; i < nContact; i++) {
        int begin = mIndices.back() + i * 6;
        state.segment(begin, 3) = mCollisionHandle->getCollisionChecker()->getContact(i).point;
        state.segment(begin + 3, 3) = mCollisionHandle->getCollisionChecker()->getContact(i).force;
    }
    mBakedStates.push_back(state);
}

void ZmpGUI::bake(const VectorXd &_dofs) {
	VectorXd state(mIndices.back());
	state.setZero();
	state.segment(mIndices[0], mDofs[0].size()) = mDofs[0];
	state.segment(mIndices[1], _dofs.size()) = _dofs;
	mBakedStates.push_back(state);
}

void ZmpGUI::bake(const std::vector<Eigen::VectorXd>& _Dofs)
{
	for(int i=0; i < _Dofs.size(); ++i) {
		bake(_Dofs[i]);
	}
}
