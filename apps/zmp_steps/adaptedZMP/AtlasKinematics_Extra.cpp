/**
 * @file AtlasKinematics_Extra.cpp
 * @brief
 */

#include "AtlasKinematics_Extra.h"

/**
 * @function KState
 */
Eigen::Affine3d KState::xform() const {

  Eigen::Affine3d xform = Eigen::Affine3d::Identity();
  xform.linear() = body_rot.toRotationMatrix();
  xform.translation() = body_pos;
  
  return xform;
}

/**
 * @function setXform
 */
void KState::setXform( const Eigen::Affine3d& _t ) {
  body_pos = _t.translation();
  body_rot = _t.rotation();
}

void KState::setXform( const Eigen::Matrix4d& _t ) {

  body_pos = _t.block(0,3,3,1);
  Eigen::Matrix3d rot = _t.block(0,0,3,3);
  body_rot = Eigen::Quaterniond( rot );
}
