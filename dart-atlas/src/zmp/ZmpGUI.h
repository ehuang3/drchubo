#pragma once

#include <stdarg.h>
#include "yui/Win3D.h"
#include "integration/EulerIntegrator.h"
#include "integration/RK4Integrator.h"
#include "collision/CollisionSkeleton.h"
#include "dynamics/SkeletonDynamics.h"
#include <iostream>

using namespace std;

namespace dynamics{
    class ContactDynamics;
}

class ZmpGUI : public yui::Win3D, public integration::IntegrableSystem {
public:
	ZmpGUI(std::vector<dynamics::SkeletonDynamics *> _skels);

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);

    // Needed for integration
    virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(const Eigen::VectorXd &state);

    // baking interface
    void bake(const std::vector<Eigen::VectorXd>& _Dofs); //< bake many
    void bake(const Eigen::VectorXd& _dofs); //< bake once
    std::vector<Eigen::VectorXd> dofs() { return mDofs; }

protected:
    int mSimFrame;
    bool mSim;
    int mPlayFrame;
    bool mPlay;
    bool mShowMarkers;
    integration::EulerIntegrator mIntegrator;
    std::vector<Eigen::VectorXd> mBakedStates;

    std::vector<dynamics::SkeletonDynamics*> mSkels;
    dynamics::ContactDynamics *mCollisionHandle;
    std::vector<Eigen::VectorXd> mDofVels;
    std::vector<Eigen::VectorXd> mDofs;
    double mTimeStep;
    Eigen::Vector3d mGravity;
    Eigen::Vector3d mForce;
    std::vector<int> mIndices;

    dynamics::SkeletonDynamics *atlas;

    void initDyn();
    void bake();

    void shakeInit();
    void shakeHips();

    std::vector<Eigen::VectorXd> mStates;
    std::vector<Eigen::Matrix4d> mTb;

    double theta;
    Eigen::VectorXd lastDofs;
    Eigen::Vector3d com;
};
