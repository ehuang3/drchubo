#include "robot_kinematics.h"
#include "utils/math_utils.h"

#include <kinematics/Skeleton.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>

#include <iostream>
#include <math.h>
#include <complex>

#include "robot_state.h"
#include "utils/robot_configs.h"
#include "robot_jacobian.h"
#define DEBUG
#define MODULE_NAME "robot-kinematics"
#include "utils/debug_utils.h"

using namespace Eigen;
using namespace kinematics;
using namespace std;

namespace robot {

////////////////////////////////////////////////////////////////////////////////
/// ARM CONSTANTS
////////////////////////////////////////////////////////////////////////////////
    arm_constants_t::arm_constants_t() {
    }

    Matrix62d arm_constants_t::getLimits(int side) const {
        if (side == SIDE_RIGHT)
            return right_limits;
        return left_limits;
    }
    
    Vector6d arm_constants_t::getOffset(int side) const {
        if (side == SIDE_RIGHT)
            return right_offset;
        return left_offset;
    }

    Vector6d arm_constants_t::getMirror(int side) const {
        if(side == SIDE_RIGHT)
            return right_mirror;
        return left_mirror;
    }

////////////////////////////////////////////////////////////////////////////////
/// LIFECYCLE
////////////////////////////////////////////////////////////////////////////////
robot_kinematics_t::robot_kinematics_t() {
}

robot_kinematics_t::~robot_kinematics_t() {
}

////////////////////////////////////////////////////////////////////////////////
/// SPECIAL IK
////////////////////////////////////////////////////////////////////////////////
    bool robot_kinematics_t::com_ik(const Eigen::Vector3d& world_com,
                                    const Eigen::Isometry3d end_effectors[NUM_MANIPULATORS],
                                    robot::IK_Mode ik_mode[NUM_MANIPULATORS],
                                    robot_state_t& state)
    {
        int COM_ITERS = 1000;
        double COM_TOL = 1e-3;
        double alpha = 0.5;
        bool stance_ok = false;
        bool com_ok = false;
        for(int i=0; i < COM_ITERS; ++i)
        {
            stance_ok = stance_ik(end_effectors, ik_mode, state);
            
            Vector3d com = state.robot()->getWorldCOM();
            Vector3d com_error = world_com - com;

            if(com_error.norm() < COM_TOL && stance_ok) {
                com_ok = true;
                break;
            }
            
            Isometry3d Twb;
            state.get_body(Twb);
            Twb.translation() += alpha * com_error;
            state.set_body(Twb);
        }

        return com_ok;
    }

    bool robot_kinematics_t::stance_ik(const Isometry3d end_effector[NUM_MANIPULATORS],
                                       robot::IK_Mode mode[NUM_MANIPULATORS], robot_state_t& state)
    {
        return manip_ik(end_effector, mode, state);
    }

    bool robot_kinematics_t::manip_ik(const Isometry3d end_effectors[NUM_MANIPULATORS],
                                      robot::IK_Mode mode[NUM_MANIPULATORS], robot_state_t& state)
    {
        Skeleton *robotSkel = state.robot();
        robotSkel->setPose(state.dart_pose());
        bool ok = true;
        for(int mi = 0; mi < NUM_MANIPULATORS; ++mi) {
            Isometry3d B;
            Isometry3d Twb;
            B = end_effectors[mi];
            switch (mode[mi]) {
            case IK_MODE_BODY:
                // Translate to global
                state.get_body(Twb);
                B = Twb*B;
            case IK_MODE_SUPPORT:
            case IK_MODE_WORLD:
                // Do ik
                switch(mi) {
                case MANIP_L_FOOT:
                    ok &= leg_ik(B, SIDE_LEFT, state);
                    break;
                case MANIP_R_FOOT:
                    ok &= leg_ik(B, SIDE_RIGHT, state);
                    break;
                case MANIP_L_HAND:
                    ok &= arm_ik(B, SIDE_LEFT, state);
                    break;
                case MANIP_R_HAND:
                    ok &= arm_ik(B, SIDE_RIGHT, state);
                    break;
                default:
                    break;
                }
                break;
            case IK_MODE_FIXED:
            case IK_MODE_FREE:
            default:
                break;
            }
        }
        robotSkel->setPose( state.dart_pose() );
        return ok;
    }

////////////////////////////////////////////////////////////////////////////////
/// ARM FK/IK SOLVER
////////////////////////////////////////////////////////////////////////////////
    double zeroSize = 1e-9;
    void robot_kinematics_t::arm_fk(Isometry3d& B, bool left, robot_state_t& state, bool use_hubo_fk)
    {
        if(use_hubo_fk)
        {
            VectorXd q(6);
            Vector6d q6;
            state.get_manip(q, left ? MANIP_L_HAND : MANIP_R_HAND);
            q6 = q.block<6,1>(0,0);
            armFK(B, q6, left ? SIDE_LEFT : SIDE_RIGHT);
            Isometry3d Tw_dsy, Tdh_wrist;
            xform_w_dsy(Tw_dsy, left, state);
            xform_dh_wrist(Tdh_wrist, left);
            B = Tw_dsy*B*Tdh_wrist; //< inaccurate!
        }
        else
        {
            Skeleton *robotSkel = state.robot();
            robotSkel->setPose(state.dart_pose());
            B = robotSkel->getNode(left ? ROBOT_LEFT_HAND : ROBOT_RIGHT_HAND)->getWorldTransform();
        }
    }

    bool robot_kinematics_t::arm_ik(const Isometry3d& B, bool left, robot_state_t& state)
    {
        // Setup joint angles
        VectorXd q(6);
        Vector6d q6, qPrev;
        state.get_manip(q, left ? MANIP_L_HAND : MANIP_R_HAND);
        q6 = q.block<6,1>(0,0);
        qPrev = q6;
        
        // Transform B to DH coordinates
        Isometry3d Tw_dsy, Tdh_wrist; //< world to DH shoulder yaw, DH wrist orientation
        xform_w_dsy(Tw_dsy, left, state);
        xform_dh_wrist(Tdh_wrist, left);
        Isometry3d B_dh = Tw_dsy.inverse()*B*Tdh_wrist.inverse(); //< B in DH space

        // Solve analytical IK
        bool ok = armIK(q6, B_dh, qPrev, left ? SIDE_LEFT : SIDE_RIGHT);

        // Copy solution back into state
        q = q6;
        state.set_manip(q, left ? MANIP_L_HAND : MANIP_R_HAND);

        return ok;
    }
    
    bool robot_kinematics_t::arm_jac_ik(const Isometry3d& B, bool left, robot_state_t& state)
    {
        // Get close with analytic IK
        bool ok = arm_ik(B, left, state);

        // No need for jacobians here.
        if (ok)
            return ok;

        // Finish it with jacobian IK
        Skeleton *robotSkel = state.robot();
        
        robot_jacobian_t rj; //< jacobian ik solver
        rj.init(robotSkel);
        
        // Init parameters
        BodyNode *hand = robotSkel->getNode(left ? ROBOT_LEFT_HAND : ROBOT_RIGHT_HAND);
        vector<int> arm;
        state.get_manip_indexes(arm, left ? MANIP_L_HAND : MANIP_R_HAND);
        
        // Solve IK
        rj.manip_jacobian_ik(B, arm, hand, state);
        
        return ok;
    }

    void robot_kinematics_t::DH2HG(Isometry3d &B, double t, double f, double r, double d) {
        // Convert DH parameters (standard convention) to Homogenuous transformation matrix.
        B = Eigen::Matrix4d::Identity();

        B.translate(Eigen::Vector3d(0.,0.,d));
        B.rotate(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitZ()));
        B.translate(Eigen::Vector3d(r,0,0));
        B.rotate(Eigen::AngleAxisd(f, Eigen::Vector3d::UnitX()));
    }

    void robot_kinematics_t::armFK(Isometry3d &B, const Vector6d &q, int side) const {
        Isometry3d hand;
        hand(0,0) =  1; hand(0,1) =  0; hand(0,2) = 0; hand(0,3) =   0;
        hand(1,0) =  0; hand(1,1) =  0; hand(1,2) =-1; hand(1,3) =   0;
        hand(2,0) =  0; hand(2,1) =  1; hand(2,2) = 0; hand(2,3) =   0;
        hand(3,0) =  0; hand(3,1) =  0; hand(3,2) = 0; hand(3,3) =   1;
        armFK(B, q, side, hand);
    }

    void robot_kinematics_t::armFK(Isometry3d &B, const Vector6d &q, int side,
                                       const Isometry3d &endEffector) const {

        // Declarations
        Isometry3d neck, hand, T;

        const double& l1 = 0;
        const double& l2 = arm.sez + arm.ssz;
        const double& l3 = arm.ewz;
        const double& l4 = arm.whz;
        //const Matrix62d limits = arm.getLimits(side);
        const Vector6d offset = arm.getOffset(side);
        const Vector6d mirror = arm.getMirror(side);

        Vector6d t, f, r, d;
        t <<  M_PI/2, -M_PI/2,  M_PI/2,       0,       0,  M_PI/2;
        f <<  M_PI/2,  M_PI/2, -M_PI/2,  M_PI/2, -M_PI/2,       0;
        r <<       0,       0,       0,       0,       0,      l4;
        d <<       0,       0,     -l2,       0,     -l3,       0;

        if (side == SIDE_RIGHT) {
            neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
            neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) = -l1;
            neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
            neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
        } else {
            // Transformation from Neck frame to left shoulder pitch frame
            neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
            neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) =  l1;
            neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
            neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
        }

        // Calculate forward kinematics
        B = neck;
        for (int i = 0; i < 6; i++) {
            DH2HG(T, t(i)+mirror(i)*(q(i)-offset(i)), f(i), r(i), d(i));
            B = B*T;
        }
        B = B*endEffector;

    }

    bool robot_kinematics_t::armIK(Vector6d &q, const Isometry3d& B, const Vector6d& qPrev, int side) const {
        // Hand
        Isometry3d hand;
        hand(0,0) =  1; hand(0,1) =  0; hand(0,2) = 0; hand(0,3) =   0;
        hand(1,0) =  0; hand(1,1) =  0; hand(1,2) =-1; hand(1,3) =   0;
        hand(2,0) =  0; hand(2,1) =  1; hand(2,2) = 0; hand(2,3) =   0;
        hand(3,0) =  0; hand(3,1) =  0; hand(3,2) = 0; hand(3,3) =   1;
        return armIK(q, B, qPrev, side, hand);
    }

    bool robot_kinematics_t::armIK(Vector6d &q, const Isometry3d& B, const Vector6d& qPrev, int side,
                                       const Isometry3d &endEffector) const {

        Eigen::ArrayXXd qAll(6,8);

        // Declarations
        Isometry3d neck, neckInv, endEffectorInv, BInv;
        double nx, sx, ax, px;
        double ny, sy, ay, py;
        double nz, sz, az, pz;
        double q1, q2, q3, q4, q5, q6;
        double qP1, qP3;
        double qT;
        Eigen::Matrix<int, 8, 3> m;

        double S2, S4, S5, S6;
        double C2, C4, C5, C6;

        const double& l1 = 0;
        const double& l2 = arm.sez + arm.ssz;
        const double& l3 = arm.ewz;
        const double& l4 = arm.whz;
        Matrix62d limits = arm.getLimits(side);
        Vector6d offset = arm.getOffset(side);
        Vector6d mirror = arm.getMirror(side);

        if (side == SIDE_RIGHT) {
            // Transformation from Neck frame to right shoulder pitch frame
            neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
            neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) = -l1;
            neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
            neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
        } else {
            // Transformation from Neck frame to left shoulder pitch frame
            neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
            neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) =  l1;
            neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
            neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
        }
        neckInv = neck.inverse();

        endEffectorInv = endEffector.inverse();


        // Variables
        //    B = neckInv*B*endEffectorInv;
        BInv = (neckInv*B*endEffectorInv).inverse();

        nx = BInv(0,0); sx = BInv(0,1); ax = BInv(0,2); px = BInv(0,3);
        ny = BInv(1,0); sy = BInv(1,1); ay = BInv(1,2); py = BInv(1,3);
        nz = BInv(2,0); sz = BInv(2,1); az = BInv(2,2); pz = BInv(2,3);


        qP1 = qPrev(0); qP3 = qPrev(2);

        m <<
                1,  1,  1,
                1,  1, -1,
                1, -1,  1,
                1, -1, -1,
                -1,  1,  1,
                -1,  1, -1,
                -1, -1,  1,
                -1, -1, -1;

        // Calculate inverse kinematics
        for (int i = 0; i < 8; i++) {

            // Solve for q4
            C4 = max(min((2*l4*px - l2*l2 - l3*l3 + l4*l4 + px*px + py*py + pz*pz)/(2*l2*l3),1.0),-1.0);
            if (fabs(C4 - 1) < zeroSize) { // Case 1: q4 == 0

                // Set q4
                q4 = 0;

                // Set q3
                q3 = qP3;

                // Solve for q6
                S6 = max(min( py/(l2 + l3), 1.0),-1.0);
                C6 = max(min( -(l4 + px)/(l2 + l3), 1.0), -1.0);
                q6 = atan2(S6,C6);


                // Solve for q2
                S2 = max(min( C4*C6*ax - C4*S6*ay, 1.0),-1.0);
                if (fabs(S2 - 1) < zeroSize) {
                    q2 = M_PI/2;
                } else if (fabs(S2 + 1) < zeroSize) {
                    q2 = -M_PI/2;
                } else {
                    complex<double> radical = 1-S2*S2;
                    q2 = atan2(S2,m(i,2)*real(sqrt(radical)));
                }

                // Solve for q5
                qT = atan2(-C6*ay - S6*ax,az);
                C2 = cos(q2);

                if (fabs(C2) < zeroSize) { // Case 3: q2 = pi/2 or -pi/2

                    q1 = qP1;
                    q3 = qP3;

                    // Solve for q5
                    if (S2 > 0) { // Case 3a: q2 = pi/2
                        qT = atan2(nz,-sz);
                        q5 = clamp2pi(q1 - q3 - qT);
                    } else { // Case 3b: q2 = -pi/2
                        qT = atan2(-nz,sz);
                        q5 = clamp2pi(qT - q1 - q3);
                    }


                } else {

                    if (C2 < 0) {
                        qT = qT + M_PI;
                    }
                    q5 = clamp2pi(qT - q3);

                    // Solve for q1
                    q1 = atan2(S6*ny - C6*nx,C6*sx - S6*sy);
                    if (C2 < 0) {
                        q1 = q1 + M_PI;
                    }
                    q1 = clamp2pi(q1);
                }

            } else {

                // Solve for q4
                complex<double> radical = 1-C4*C4;
                q4 = atan2(m(i,0)*real(sqrt(radical)),C4);

                // Solve for q5
                S4 = sin(q4);
                S5 = pz/(S4*l2);
                if (fabs(S5 - 1) < zeroSize) {
                    q5 = M_PI/2;
                } else if (fabs(S5 + 1) < zeroSize) {
                    q5 = -M_PI/2;
                } else {
                    radical = 1-S5*S5;
                    q5 = atan2(S5,m(i,1)*real(sqrt(radical)));
                }

                // Solve for q6
                C5 = cos(q5);
                S6 =max(min( (C5*S4*l2 + (py*(l3 + C4*l2 - (C5*S4*l2*py)/(l4 + px)))/(l4 + px + py*py/(l4 + px)))/(l4 + px), 1.0),-1.0);
                C6 = max(min( -(l3 + C4*l2 - (C5*S4*l2*py)/(l4 + px))/(l4 + px + py*py/(l4 + px)), 1.0),-1.0);
                q6 = atan2(S6,C6);

                // Solve for q2
                S2 = max(min(ax*(C4*C6 - C5*S4*S6) - ay*(C4*S6 + C5*C6*S4) - S4*S5*az,1.0),-1.0);
                if (fabs(S2 - 1) < zeroSize) {
                    q2 = M_PI/2;
                } else if (fabs(S2 + 1) < zeroSize) {
                    q2 = -M_PI/2;
                } else {
                    radical = 1-S2*S2;
                    q2 = atan2(S2,m(i,2)*real(sqrt(radical)));
                }

                // Solve for q3
                C2 = cos(q2);

                if (fabs(C2) < zeroSize) { // Case 2: q2 = pi/2 or -pi/2

                    q3 = qP3;
                    // Solve for q1
                    if (S2 > 0) { // Case 2a: q2 = pi/2
                        qT = atan2(S6*sy - C6*sx,S6*ny - C6*nx);
                        if (S4 < 0) {
                            qT = qT + M_PI;
                        }
                        q1 = clamp2pi(qT + q3);
                    } else { // Case 2b: q2 = -pi/2
                        qT = atan2(S6*sy - C6*sx,S6*ny - C6*nx);
                        if (S4 < 0) {
                            qT = qT + M_PI;
                        }
                        q1 = clamp2pi(qT - q3);
                    }

                } else {
                    q3 = atan2(S4*S6*ay - C4*S5*az - C6*S4*ax - C4*C5*C6*ay - C4*C5*S6*ax,C5*az - C6*S5*ay - S5*S6*ax);
                    if (C2 < 0) {
                        q3 = q3 - M_PI;
                    }
                    q3 = clamp2pi(q3);

                    // Solve for q1
                    q1 = atan2(C4*S6*ny - C4*C6*nx + S4*S5*nz + C5*C6*S4*ny + C5*S4*S6*nx,C4*C6*sx - C4*S6*sy - S4*S5*sz - C5*C6*S4*sy - C5*S4*S6*sx);
                    if (C2 < 0) {
                        q1 = q1 + M_PI;
                    }
                    q1 = clamp2pi(q1);
                }
            }

            qAll(0,i) = q1;
            qAll(1,i) = q2;
            qAll(2,i) = q3;
            qAll(3,i) = q4;
            qAll(4,i) = q5;
            qAll(5,i) = q6;

        }
        // Set to offset
        for( int j=0; j<8; j++)
            for (int i = 0; i < 6; i++)
                    qAll(i,j) = clamp2pi(mirror(i)*qAll(i,j) + offset(i));

        // TODO: Find best solution using better method

        Eigen::ArrayXd qDiff(6,1); qDiff.setZero();
        Eigen::ArrayXd qDiffSum(8,1);
        bool withinLim[8];
        int minInd;

        // if any joint solution is infintesimal, set it to zero
        for(int i=0; i<8; i++)
            for(int j=0; j<6; j++)
                if(qAll(j,i) < zeroSize && qAll(j,i) > -zeroSize)
                    qAll(j,i) = 0.0;

        // Initialize withinLim to all trues for all eight solutions
        for(int i=0; i<8; i++)
            withinLim[i] = true;

        // Check each set of solutions to see if any are outside the limits
        for(int i=0; i<8; i++)
            for(int j=0; j<6; j++)
                if( limits(j,0) > qAll(j,i) || qAll(j,i) > limits(j,1) )
                    withinLim[i] = false;

        // Initialze anyWithin boolean array to all trues
        bool anyWithin=false;
        for(int i=0; i<8; i++)
            if( withinLim[i] )
                anyWithin = true;

        // If any solution has all joints within the limits...
        if(anyWithin)
        {
            // for each solution...
            for (int i = 0; i < 8; i++) {
                // if all the joints of solution i are within the limits...
                if( withinLim[i] )
                {
                    // calculate the differences between solution angles, j, and previous angles
                    for (int j=0; j < 6; j++)
                        qDiff(j) = clamp2pi(qAll(j,i) - qPrev(j));
                    // sum the absolute values of the differences to get total difference
                    qDiffSum(i) = qDiff.abs().sum();
                }
                // if the solution doesn't have all the joints within the limits...
                else
                    // set the difference for that solution to infinity
                    qDiffSum(i) = std::numeric_limits<double>::infinity();
            }
            // and take the solution closest to previous solution
            qDiffSum.minCoeff(&minInd);
            q = qAll.col(minInd);
        }
        // if no solution has all the joints within the limits...
        else
        {
            // then for each solution...
            for( int i=0; i<8; i++)
            {
                // create a 6d vector of angles of solution i
                Vector6d qtemp = qAll.col(i).matrix();
                // take the min of the angles and the joint upper limits
                qtemp = qtemp.cwiseMin(limits.col(1));
                // then take the max of those angles and the joint lower limits
                qtemp = qtemp.cwiseMax(limits.col(0));
                // create an Isometry3d 4x4 matrix for the temp pose
                Isometry3d Btemp;
                // find the pose associated with the temp angles
                armFK( Btemp, qtemp, side );
                // calculate the distance from previous pose to temp pose locations
                qDiffSum(i) = (Btemp.translation() - B.translation()).norm();
            }
            // find the solution that's closest the previous position
            qDiffSum.minCoeff(&minInd);
            q = qAll.col(minInd);
        }
        // set the final joint angles to the solution closest to the previous solution
        for( int i=0; i<6; i++ )
            q(i) = max( min( q(i), limits(i,1)), limits(i,0) );

        //q = q.cwiseMin(limits.col(1)); //TODO: Put these back
        //q = q.cwiseMax(limits.col(0));
        return anyWithin;
    }

////////////////////////////////////////////////////////////////////////////////
/// LEG FK/IK SOLVER
////////////////////////////////////////////////////////////////////////////////
    bool robot_kinematics_t::leg_ik(const Isometry3d& B, bool left, robot_state_t& state)
    {
        // Find B relative to the body
        Matrix4d Twb = state.robot()->getNode(ROBOT_BODY)->getWorldTransform();
        Matrix4d Tbe = Twb.inverse() * B.matrix(); // body -> end effector
        // solve ik
        VectorXd q(6);
        state.get_manip(q, left ? MANIP_L_FOOT : MANIP_R_FOOT);
        Vector6d q6 = q.block<6,1>(0,0);
        bool ok = legIK(leg_world_to_dh(Tbe), left, q6, q6);
        q = q6;
        state.set_manip(q, left ? MANIP_L_FOOT : MANIP_R_FOOT);
        return ok;
    }

    // misnamed
    Matrix4d robot_kinematics_t::leg_world_to_dh(const Matrix4d& B)
    {
        Matrix3d R;
        R <<0, 0, 1,
            0, 1, 0,
            -1, 0, 0;
        Matrix4d Tf = B.matrix();
        Tf.block<3,3>(0,0) *= R;
        return Tf;
    }



    Matrix4d robot_kinematics_t::legT(int _frame, double _u) {
        return _legT(_frame, _u + leg_u_off[_frame-1]);
    }

    Matrix4d robot_kinematics_t::legFK(const Vector6d& _u, bool _left) {
        return _legFK(_u, _left);
    }

    bool robot_kinematics_t::legIK(const Matrix4d& _Tbf, bool _left, const Vector6d& _p, Vector6d& _u) {
        MatrixXd tmp;
        return _legIK(_Tbf, _left, _p, true, _u, tmp);
    }

    Matrix4d robot_kinematics_t::_legT(int _f, double _u) {
        return dh2transform(leg_dh[_f-1].r, leg_dh[_f-1].a, leg_dh[_f].t+_u, leg_dh[_f].d);
    }

    Matrix4d robot_kinematics_t::_legFK(const Vector6d& _u, bool _left) {
        Matrix4d Tf, T01, T12, T23, T34, T45, T56;
        T01 = _legT(1, _u(0) + leg_u_off[0]);
        T12 = _legT(2, _u(1) + leg_u_off[1]);
        T23 = _legT(3, _u(2) + leg_u_off[2]);
        T34 = _legT(4, _u(3) + leg_u_off[3]);
        T45 = _legT(5, _u(4) + leg_u_off[4]);
        T56 = _legT(6, _u(5) + leg_u_off[5]);
        Tf = T01*T12*T23*T34*T45*T56;
        double l0 = leg_link_disp[0](1); // pelvis to hip
        Tf(1,3) += _left ? l0 : -l0;
        return Tf;
    }

    bool robot_kinematics_t::_legIK(Matrix4d _Tf,
                                    bool _left,
                                    const Vector6d& _p,
                                    bool _nearest,
                                    Vector6d& _u,
                                    MatrixXd& _U) {
        // link lengths
        double l0 = leg_link_disp[0](1); // pelvis to hip
        double l1 = leg_link_disp[2](2); // hip yaw to hip pitch z
        double l2 = leg_link_disp[2](0); // hip yaw to hip pitch x
        double l3 = leg_link_disp[3](2); // hip pitch to knee pitch
        double l4 = leg_link_disp[4](2); // knee pitch to ankle pitch

        // invert right limits
        int mirror[] = {1, 1, 0, 0, 0, 1}; //< limits that need to be inverted
    
        Matrix4d T0f = _Tf;
        T0f(1,3) += _left ? -l0 : l0;
        
        // % C++ implementation of script/AtlasLegIK.m
        // for convenience
        double x0,x1,x2,y0,y1,y2,z0,z1,z2,t0,t1,t2;
        x0 = T0f(0,0); y0 = T0f(0,1); z0 = T0f(0,2); t0 = T0f(0,3);
        x1 = T0f(1,0); y1 = T0f(1,1); z1 = T0f(1,2); t1 = T0f(1,3);
        x2 = T0f(2,0); y2 = T0f(2,1); z2 = T0f(2,2); t2 = T0f(2,3);
        
        // solve u6, u2, u1
        // % For solution method, see script/AtlasLegSymbolicKinematics.m
        
        // index mapping
        int m6[8] = { 0, 1, 0, 1, 0, 1, 0, 1 };  // map of u6
        int m2[8] = { 0, 1, 2, 3, 0, 1, 2, 3 };  // map of u2
        int m1[8] = { 0, 1, 2, 3, 0, 1, 2, 3 };  // map of u1
        
        // sign flips for permutations
        int s2[4] = { 1, 1, -1, -1 };  // cos signs of u2
        int s4[8] = { 1, 1, 1, 1, -1, -1, -1, -1 };  // cos signs of u4
        
        // u6 - ankle roll
        double C1, C2;
        C1 = t0*y0 + t1*y1 + t2*y2;
        C2 = t0*x0 + t1*x1 + t2*x2;

        double u6[2];
        u6[0] = atan(-C1/C2);
        u6[1] = clamp2pi(u6[0] + M_PI);
        
        // u2 - hip roll
        double u2[4];
        for(int i=0; i < 4; ++i) {
            int i6 = m6[i];
            u2[i] = s2[i] * acos(y2*cos(u6[i6]) + x2*sin(u6[i6]));
        }
        
        // u1 - hip yaw
        double u1[4];
        for(int i=0; i < 4; ++i) {
            int i6 = m6[i];
            u1[i] = atan2( -( y1*cos(u6[i6]) + x1*sin(u6[i6]) ) / sin(u2[i]),
                           -( y0*cos(u6[i6]) + x0*sin(u6[i6]) ) / sin(u2[i]) );
        }
        
        // zero angles
        for(int i=0; i < 4; i++) {
            u2[i] = clamp2pi(u2[i] + M_PI/2);
            u1[i] = clamp2pi(u1[i] - M_PI/2);
        }
        
        // solve u3, u4, u5
        // % Geometric solution in chap 4.4 from "Introduction to Robotics" by Craig
        
        // u4 - knee pitch
        // u3 - hip pitch
        // u5 - ankle pitch
        double u4[8];
        double u3[8];
        double u5[8];
        Matrix4d T01, T12, T2f;
        Vector4d p2f;
        double x, y, phi, beta, psi;
        double c4, cpsi;
        complex<double> radical;
        for(int i=0; i < 8; i++) {
            int i1 = m1[i];
            int i2 = m2[i];
            
            T01 = _legT(1, u1[i1]);
            T12 = _legT(2, u2[i2]);
            T2f = T12.inverse() * T01.inverse() * T0f;
            p2f = T2f.block(0,3,4,1);
            
            x = p2f(0) - l1;
            y = -p2f(2);
            
            c4 = (x*x + y*y - l3*l3 - l4*l4) / (2*l3*l4);
            radical = 1 - c4*c4;
            u4[i] = atan2( s4[i]*real(sqrt(radical)), c4 );
            
            //u4[i] = s4[i] * acos( c4 );
            
            phi = atan2( T2f(0,2), T2f(2,2) );
            beta = atan2(y,x);
            
            cpsi = (x*x + y*y + l3*l3 - l4*l4) / (2*l3*sqrt(x*x + y*y));
            radical = 1 - cpsi*cpsi;
            psi = atan2( real(sqrt(radical)), cpsi );
            
            //psi = acos( cpsi );
            
            psi = clamp2pi(psi);
            if(psi < 0)
                psi += M_PI;
            
            if(u4[i] < 0)
                u3[i] = beta + psi;
            else
                u3[i] = beta - psi;
            
            u5[i] = phi - u4[i] - u3[i];
            
            u3[i] = clamp2pi(u3[i]);
            u5[i] = clamp2pi(u5[i]);
        }
        
        // select solution
        MatrixXd U(6,8);
        bool within_lim[8] = {1,1,1,1,1,1,1,1};
        bool any_within = false;
        bool nearest_pos[8] = {0,0,0,0,0,0,0,0};
        double pos_delta[8];
        int within_count = 0;
        const double POS_TOL = 1e-10;
        const double JOINT_TOL = 1e-2;
        Matrix4d T06;
        // offset to joint zeros
        for(int i=0; i < 8; i++) {
            int i1 = m1[i];
            int i2 = m2[i];
            int i6 = m6[i];
            U(0,i) = u1[i1] - leg_u_off[0];
            U(1,i) = u2[i2] - leg_u_off[1];
            U(2,i) = u3[i] - leg_u_off[2];
            U(3,i) = u4[i] - leg_u_off[3];
            U(4,i) = u5[i] - leg_u_off[4];
            U(5,i) = u6[i6] - leg_u_off[5];
        }
        // clamp epsilon errors around joint limits
        // (for foot positions in workspace, but at joint limits)
        for(int i=0; i < 8; i++)
            for(int j=0; j < 6; j++)
                for(int k=0; k < 2; ++k)
                    if(fabs(U(j,i) - leg_u_lim[j][k]) < JOINT_TOL)
                        U(j,i) = leg_u_lim[j][k];
        // check joint limits
        for(int i=0; i < 8; i++) {
            for(int j=0; j < 6; j++) {
                if(_left && !(leg_u_lim[j][0] <= U(j,i) && U(j,i) <= leg_u_lim[j][1]))
                    within_lim[i] = false;
                if(!_left) {
                    if(mirror[j]) {
                        std::cout << "mirror " << j << std::endl;
                        if(!(-leg_u_lim[j][1] <= U(j,i) && U(j,i) <= -leg_u_lim[j][0])) {
                            std::cout << "failed " << j << std::endl;
                            within_lim[i] = false;
                        }
                    }
                    else
                        if(!(leg_u_lim[j][0] <= U(j,i) && U(j,i) <= leg_u_lim[j][1]))
                            within_lim[i] = false;
                }
            }
            if(within_lim[i]) {
                any_within = true;
            }
        }
        // clamp to joint limits
        for(int i=0; i < 8; i++)
            for(int j=0; j < 6; j++) {
                if(_left) {
                    if(U(j,i) < leg_u_lim[j][0])
                        U(j,i) = leg_u_lim[j][0];
                    if(leg_u_lim[j][1] < U(j,i))
                        U(j,i) = leg_u_lim[j][1];

                    //U(j,i) = max(min(U(j,i), leg_u_lim[j][0]), leg_u_lim[j][1]);
                }
                if(!_left) {
                    if(mirror[j])
                        U(j,i) = min(max(U(j,i), -leg_u_lim[j][1]), -leg_u_lim[j][0]);
                    else
                        U(j,i) = min(max(U(j,i), leg_u_lim[j][0]), leg_u_lim[j][1]);
                }
            }
        // // complain if out of workspace
        // if(!any_within) {
        //     cerr << "AtlasKinematics: IK Warning - Out of workspace\n"
        //          << "left= " << _left << "\n"
        //          << "T0f=\n" << T0f << "\n"
        //          << "p=\n" << _p << "\n"
        //          << "u=\n" << U << "\n"
        //          << "\n";
        // }
        if(!_nearest) {
            _U = U; // return clamped U
            return any_within; // complain if out of workspace
        }
        if(!any_within) {
            // no solutions within limits
            // find nearest(s) in position
            double min_delta = -1.0;
            for(int i=0; i < 8; i++) {
                T06 = _legFK(U.block<6,1>(0,i), _left);
                pos_delta[i] = (T06-_Tf).block<3,1>(0,3).norm();
                if(min_delta == -1.0 || pos_delta[i] < min_delta)
                    min_delta = pos_delta[i];
            }
            for(int i=0; i < 8; i++) {
                if(fabs(min_delta - pos_delta[i]) < POS_TOL)
                    nearest_pos[i] = true;
            }
        }
        // select closest to previous joint angles
        int sol = -1;
        double min_delta;
        for(int i=0; i < 8; i++) {
            if(within_lim[i] || nearest_pos[i]) {
                double delta = (U.block<6,1>(0,i) - _p).norm();
                if(sol == -1 || delta < min_delta) {
				sol = i;
				min_delta = delta;
                }
            }
        }
        _u = U.block<6,1>(0,sol);
        // // complain about discontinuity
        // if(false && min_delta >= JOINT_TOL) {
        //     cerr << "AtlasKinematics: IK Warning - Solution discontinuity\n"
        //          << "T0f=\n" << T0f << "\n"
        //          << "u delta=" << min_delta << "\n"
        //          << "\n";
        // }
        return any_within;
    }

////////////////////////////////////////////////////////////////////////////////
/// DEPRECIATED FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
bool robot_kinematics_t::comIK(Skeleton *_robot,
							const Vector3d& _dcom,
							Matrix4d& _Twb,
							IK_Mode _mode[NUM_MANIPULATORS],
							const Matrix4d _Twm[NUM_MANIPULATORS],
							VectorXd& _dofs) {

	//TODO: Use COM Jacobians to turn the pelvis.
	// Or, pass in a rotated Twb and stanceIK to that.

	// Position robot in world frame
	_dofs.block<3,1>(0,0) = _Twb.block<3,1>(0,3);

	// Do gradient descent on COM error
	bool debug = false;
	int COM_ITER = 10000; // max iterations
	double COM_PTOL = 1e-3; // err tolerance
	double alpha = 0.5; // descent scaling factor
	bool ik_valid = true;
	bool ik_found = false;
	Vector3d com = Vector3d::Ones();
	Vector3d com_err;
	//TODO: Replace dof conversions with a dof class
	VectorXd u(24); // vector of manipulator joint angles
	for(int i=0; i < NUM_MANIPULATORS; ++i) {
		for(int j=0; j < 6; ++j) {
			int ind = dart_dof_ind[i][j];
			u(i*6 + j) = _dofs(ind); // init joint angles
		}
	}
	int i;
	for(i=0; i < COM_ITER; ++i) {
		// Solve for leg angles
		ik_valid = stanceIK(_Twb, _Twm[MANIP_L_FOOT], _Twm[MANIP_R_FOOT], u, u);

		// Position robot in world frame
		for(int i=0; i < NUM_MANIPULATORS; ++i) {
			for(int j=0; j < 6; ++j) {
				int ind = dart_dof_ind[i][j];
				_dofs(ind) = u(i*6 + j);
			}
		}
		_dofs.block<3,1>(0,0) = _Twb.block<3,1>(0,3);
		// Copy angles to skeleton
		_robot->setPose(_dofs, true, false);

		// COM in world frame
		com = _robot->getWorldCOM();

		// COM error
		com_err = _dcom - com;

		if(com_err.norm() < COM_PTOL) {
			ik_found = true;
			break;
		}

		// descent
		_Twb.block(0,3,3,1) += alpha * com_err;

		if(debug) {
			cout << "COM_IK: iteration " << i << endl;
			for(int i=0; i < NUM_MANIPULATORS; ++i) {
				cout << "manip " << i << "= " << u.block(i*6,0,6,1).transpose() << "\n";
			}
			cout << "dcom=\n" << _dcom << endl;
			cout << "com=\n" << com << endl;
			cout << endl;
		}
	}
	if(debug) {
		cout << "\nconverged at iteration " << i << "\n"
			 << "err= " << com_err.norm() << "\n\n";
	}

	return ik_found && ik_valid;
}

bool robot_kinematics_t::stanceIK(const Matrix4d& _Twb,
							   const Matrix4d& _Twl,
							   const Matrix4d& _Twr,
							   const VectorXd& _p,
							   VectorXd& _u) {
	Matrix4d Tbl = _Twb.inverse() * _Twl;
	Matrix4d Tbr = _Twb.inverse() * _Twr;

	Vector6d ul, ur;
	bool valid = legIK(Tbl, true, _p.block<6,1>(0,0), ul);
	valid &= legIK(Tbr, false, _p.block<6,1>(6,0), ur);
	_u.block<6,1>(0,0) = ul;
	_u.block<6,1>(6,0) = ur;

	return valid;
}

}
