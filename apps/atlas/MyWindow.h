#ifndef _MYWINDOW_
#define _MYWINDOW_

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

class MyWindow : public yui::Win3D, public integration::IntegrableSystem {
public:
 MyWindow(dynamics::SkeletonDynamics* ground, dynamics::SkeletonDynamics* robot): Win3D() {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
        
        mSim = false;
        mPlay = false;
        mSimFrame = 0;
        mPlayFrame = 0;
        mShowMarkers = true;

        mPersp = 45.f;
        mTrans[1] = 300.f;
    
        mGravity = Eigen::Vector3d(0.0, 0.0, -9.8);
        mTimeStep = 1.0/1000.0;
        mForce = Eigen::Vector3d::Zero();

        mSkels.push_back(ground);
        mSkels.push_back(robot);

        atlas = robot;

        //FIXME: stdargs fails with -O2 optimization turned on
        // if (_mList) {
        //     //FIXME: vargs returns too many args!!!
        //     mSkels.push_back(_mList);
        //     cout << "skel" << _mList->getName() << endl;
        //     va_list ap;
        //     va_start(ap, _mList);
        //     while (true) {
        //         dynamics::SkeletonDynamics *skel = va_arg(ap, dynamics::SkeletonDynamics*);
                
        //         if(skel) {
        //             cout << "skel" << skel->getName() << endl;
        //             mSkels.push_back(skel);
        //         }
        //         else
        //             break;
        //     }
        //     va_end(ap);
        // }
        
        int sumNDofs = 0;
        mIndices.push_back(sumNDofs);
        for (unsigned int i = 0; i < mSkels.size(); i++) {
            int nDofs = mSkels[i]->getNumDofs();
            sumNDofs += nDofs;
            mIndices.push_back(sumNDofs);
        }
        initDyn();
    }

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);

    // Needed for integration
    virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(const Eigen::VectorXd &state);    

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
};

#endif
