#pragma once

#include <iostream>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <atlas/atlas_kinematics.h>
#include <utils/math_utils.h>
#include <utils/data_paths.h>
#include <math/EigenHelper.h>

#include <math.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <simulation/World.h>
#include <kinematics/Skeleton.h>
#include <kinematics/Dof.h>
#include <kinematics/BodyNode.h>
#include <kinematics/Joint.h>
#include <kinematics/Transformation.h>
#include <dynamics/SkeletonDynamics.h>

/* ********************************************************************************************* */
// Loads in parameters for switching between atlas and hubo
#include "utils/robot_configs.h"
/* ********************************************************************************************* */
kinematics::Skeleton* _robot_;
kinematics::Skeleton* PREPARE_ROBOT() {
    if(_robot_ == 0) {
        DartLoader dart_loader;
        simulation::World *mWorld = dart_loader.parseWorld(VRC_DATA_PATH ROBOT_URDF);
		_robot_ = mWorld->getSkeleton(ROBOT_NAME);
    }
    return _robot_;
}
/* ********************************************************************************************* */
ROBOT_JACOBIAN_T* _rj_;
robot::robot_jacobian_t* PREPARE_ROBOT_JACOBIAN() {
    if(_rj_ == 0) {
        _rj_ = new ROBOT_JACOBIAN_T;
        _rj_->init(PREPARE_ROBOT());
    }
    return _rj_;
}
/* ********************************************************************************************* */
ROBOT_STATE_T* _rs_;
robot::robot_state_t* PREPARE_ROBOT_STATE() {
    if(_rs_ == 0) {
        _rs_ = new ROBOT_STATE_T;
        _rs_->init(PREPARE_ROBOT());
    }
    return _rs_;
}
/* ********************************************************************************************* */
ROBOT_KINEMATICS_T* _rk_;
robot::robot_kinematics_t* PREPARE_ROBOT_KINEMATICS() {
	if(_rk_ == 0) {
		_rk_ = new ROBOT_KINEMATICS_T;
		_rk_->init(PREPARE_ROBOT());
	}
	return _rk_;
}
/* ********************************************************************************************* */
double _TOLERANCE_ = 1e-9;
void ASSERT_MATRIX_EQ(Eigen::MatrixXd A, Eigen::MatrixXd B, double tol = 1e-9) 
{
    ASSERT_EQ(A.rows(), B.rows());
    ASSERT_EQ(A.cols(), B.cols());
    for(int i=0; i < A.rows(); ++i) {
        for(int j=0; j < A.cols(); ++j) {
            ASSERT_NEAR(A(i,j), B(i,j), tol);
        }
    }
}
/* ********************************************************************************************* */
