/**
 * @file Controller.cpp
 * @author T. Kunz
 */

#include "Controller.h"
#include <dynamics/SkeletonDynamics.h>
#include <planning/Trajectory.h>
#include <kinematics/Dof.h>
#include <iostream>
#include <dynamics/BodyNodeDynamics.h>
#include <math/UtilsMath.h>

using namespace std;
using namespace Eigen;

namespace planning {

Controller::Controller(dynamics::SkeletonDynamics* _skel, 
		       const vector<int> &_actuatedDofs,
                       const VectorXd &_kP, 
		       const VectorXd &_kD, 
		       const vector<int> &_ankleDofs, 
		       const VectorXd &_anklePGains, 
		       const VectorXd &_ankleDGains) :
    mSkel(_skel),
    mKp(_kP.asDiagonal()),
    mKd(_kD.asDiagonal()),
    mAnkleDofs(_ankleDofs),
    mAnklePGains(_anklePGains),
    mAnkleDGains(_ankleDGains),
    mTrajectory(NULL)
{
    const int nDof = mSkel->getNumDofs();

    mSelectionMatrix = MatrixXd::Zero(nDof, nDof);
    for (int i = 0; i < _actuatedDofs.size(); i++) {
        mSelectionMatrix(_actuatedDofs[i], _actuatedDofs[i]) = 1.0;
    }

    mDesiredDofs.resize(nDof);
    for (int i = 0; i < nDof; i++) {
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
    }

    Vector3d com = mSkel->getWorldCOM();
    double cop = 0.0;
    mPreOffset = com[0] - cop;
}


void Controller::setTrajectory(const Trajectory* _trajectory, double _startTime, const std::vector<int> &_dofs) {
    mTrajectoryDofs = _dofs;
    mTrajectory = _trajectory;
    mStartTime = _startTime;
}


VectorXd Controller::getTorques(const VectorXd& _dof, const VectorXd& _dofVel, double _time) {
    Eigen::VectorXd desiredDofVels = VectorXd::Zero(mSkel->getNumDofs());

    if(mTrajectory && _time - mStartTime >= 0.0 & _time - mStartTime <= mTrajectory->getDuration()) {
        for(unsigned int i = 0; i < mTrajectoryDofs.size(); i++) {
            mDesiredDofs[mTrajectoryDofs[i]] = mTrajectory->getPosition(_time - mStartTime)[i];
            desiredDofVels[mTrajectoryDofs[i]] = mTrajectory->getVelocity(_time - mStartTime)[i];
        }
    }
    
    VectorXd torques;
    const double mTimestep = 0.001;

    // SPD controller
    // J. Tan, K. Liu, G. Turk. Stable Proportional-Derivative Controllers. IEEE Computer Graphics and Applications, Vol. 31, No. 4, pp 34-44, 2011.
    MatrixXd M = mSkel->getMassMatrix() + mKd * mTimestep;
    VectorXd p = -mKp * (_dof - mDesiredDofs + _dofVel * mTimestep);
    VectorXd d = -mKd * (_dofVel - desiredDofVels);
    VectorXd qddot = M.ldlt().solve(-mSkel->getCombinedVector() + p + d);
    torques = p + d - mKd * qddot * mTimestep;

    // ankle strategy for sagital plane
    Vector3d com = mSkel->getWorldCOM();
    double cop = 0.0;
    double offset = com[0] - cop;

    for(unsigned int i = 0; i < mAnkleDofs.size(); i++) {
        torques[mAnkleDofs[i]] = - mAnklePGains[i] * offset - mAnkleDGains[i] * (offset - mPreOffset) / mTimestep;
    }

    mPreOffset = offset;

    return mSelectionMatrix * torques;
}

}
