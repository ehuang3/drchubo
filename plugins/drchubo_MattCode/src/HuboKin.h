#ifndef _HUBOKIN_H_
#define _HUBOKIN_H_

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <complex>

#define HUBOKIN_USE_KCONSTANTS

namespace HK {

  typedef Eigen::Matrix< double, 6, 1 > Vector6d;
  typedef Eigen::Vector3d Vector3d;
  typedef Eigen::Isometry3d Isometry3d;
  typedef Eigen::Matrix< double, 6, 2 > Matrix62d;
  typedef std::vector<int> IntArray;

  class HuboKin {
  public:

    enum {
      SIDE_RIGHT = 0,
      SIDE_LEFT = 1
    };

    struct KinConstants {

      double arm_l1, arm_l2, arm_l3, arm_l4;
      double leg_l1, leg_l2, leg_l3, leg_l4, leg_l5, leg_l6;

      Matrix62d arm_limits;
      Matrix62d leg_limits;
    
      Vector6d  arm_offset;
      Vector6d  leg_offset;

      IntArray arm_mirror;
      IntArray leg_mirror;

      KinConstants();


      Matrix62d getArmLimits(int side) const;
      Matrix62d getLegLimits(int side) const;
      Vector6d  getArmOffset(int side) const;
      Vector6d  getLegOffset(int side) const;

    };

    KinConstants kc;

    static Matrix62d mirrorLimits(const Matrix62d& orig, const IntArray& mirror);
    static Vector6d  mirrorAngles(const Vector6d& orig, const IntArray& mirror);

    static void DH2HG(Isometry3d &B, double t, double f, double r, double d);

    void armFK(Isometry3d &B, const Vector6d &q, int side) const;

    void armFK(Isometry3d &B, const Vector6d &q, int side, 
               const Isometry3d &endEffector) const;

    void armIK(Vector6d &q, const Isometry3d& B, 
               const Vector6d& qPrev, int side) const;

    void armIK(Vector6d &q, const Isometry3d& B, 
               const Vector6d& qPrev, int side, 
               const Isometry3d &endEffector) const;

    void legFK(Isometry3d &B, const Vector6d &q, int side) const;

    void legIK(Vector6d &q, const Isometry3d& B, 
               const Vector6d& qPrev, int side) const;


  };



}

#endif


