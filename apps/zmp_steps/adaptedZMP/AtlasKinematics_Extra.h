/**
 * @file AtlasKinematics_Extra.h
 * @brief
 */

#ifndef __ATLAS_KINEMATICS_EXTRA__
#define __ATLAS_KINEMATICS_EXTRA__

#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @class KState
 */
class KState {

 public:
  Eigen::Quaterniond body_rot;
  Eigen::Vector3d body_pos;

  Eigen::VectorXd jvalues;

  Eigen::Affine3d xform() const;
  void setXform( const Eigen::Affine3d& _x );
  void setXform( const Eigen::Matrix4d& _x );

};


#endif /* __ATLAS_KINEMATICS_EXTRA__ */
