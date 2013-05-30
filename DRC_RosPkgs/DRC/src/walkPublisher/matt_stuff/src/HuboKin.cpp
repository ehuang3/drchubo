#include "HuboKin.h"

using namespace std;

namespace HK { 

static inline double mod(double x, double y) {
  if (0 == y)
    return x;
  return x - y * floor(x/y);
}

static inline double wrapToPi(double fAng) {
  return mod(fAng + M_PI, 2*M_PI) - M_PI;
}

static const double zeroSize = 1e-9;

HuboKin::KinConstants::KinConstants() {

  /*

    pos of LSP = (3.46945e-18, 0.1415,   1.23497e-18)
    pos of LSR = (0.0269,      0.2135,   1.30104e-18)
    pos of LSY = (3.46945e-18, 0.2135,  -0.0245)
    pos of LEP = (0.0219936,   0.1955,  -0.182005)
    pos of LWY = (5.20417e-18, 0.2135,  -0.293413)
    pos of LWP = (5.20417e-18, 0.20477, -0.345963)

  */

  leg_l1 = 0.201256; // neck -> waist Z
  leg_l2 = 0.0885021; // waist -> hip Y
  leg_l3 = 0.167473; // waist -> hip Z
  leg_l4 = 0.280007; // hip -> knee Z
  leg_l5 = 0.279942; // knee -> ankle Z
  leg_l6 = 0.105888; // ankle to foot Z

  leg_limits << 
    -1.5708, 1.5708,      // RHY
    -0.488692, 0.488692,  // RHR
    -1.48353, 1.6057,     // RHP
    -0.0698132, 2.60054,  // RKN
    -1.29154, 1.69297,    // RAP
    -0.191986, 0.191986;  // RAR

  arm_l1 = 214.5/1000.0;        // neck to shoulder Y
  arm_l2 = 179.14/1000.0;       // ?
  arm_l3 = 181.59/1000.0;       // ?
  arm_l4 = 4.75*25.4/1000.0;    // ?

  arm_limits << 
    -2,   2,
    -2,  .3,
    -2,   2,
    -2,   0.01,
    -2,   2,
    -1.4, 1.2;


  arm_offset.setZero();
  leg_offset.setZero();

  arm_offset[1] = 0.3;

  arm_mirror.push_back(1);
  arm_mirror.push_back(2);
  arm_mirror.push_back(4);

  leg_mirror.push_back(0);
  leg_mirror.push_back(1);
  leg_mirror.push_back(5);

}

 
Matrix62d HuboKin::mirrorLimits(const Matrix62d& orig,
                                         const IntArray& mirror) {
  
  Matrix62d limits = orig;
  
  for (size_t i=0; i<mirror.size(); ++i) {
    int ax = mirror[i];
    double& lo = limits(ax,0);
    double& hi = limits(ax,1);
    swap(lo,hi);
    lo *= -1;
    hi *= -1;
  }
  
  return limits;

}

Vector6d HuboKin::mirrorAngles(const Vector6d& orig,
                                        const IntArray& mirror) { 
  
  Vector6d angles = orig;
  
  for (size_t i=0; i<mirror.size(); ++i) {
    int ax = mirror[i];
    angles[ax] *= -1;
  }
  
  return angles;

}

Matrix62d HuboKin::KinConstants::getArmLimits(int side) const {
  if (side == SIDE_RIGHT) {
    return arm_limits;
  } else {
    return HuboKin::mirrorLimits(arm_limits, arm_mirror);
  }
}

Matrix62d HuboKin::KinConstants::getLegLimits(int side) const {
  if (side == SIDE_RIGHT) {
    return leg_limits;
  } else {
    return HuboKin::mirrorLimits(leg_limits, leg_mirror);
  }
}

Vector6d HuboKin::KinConstants::getArmOffset(int side) const {
  if (side == SIDE_RIGHT) {
    return arm_offset;
  } else {
    return HuboKin::mirrorAngles(arm_offset, arm_mirror);
  }
}

Vector6d HuboKin::KinConstants::getLegOffset(int side) const {
  if (side == SIDE_RIGHT) {
    return leg_offset;
  } else {
    return HuboKin::mirrorAngles(leg_offset, leg_mirror);
  }
}


void HuboKin::DH2HG(Isometry3d &B, double t, double f, double r, double d) {

  // Convert DH parameters (standard convention) to Homogenuous transformation matrix.
  B = Eigen::Matrix4d::Identity();
    
  B.translate(Eigen::Vector3d(0.,0.,d));
  B.rotate(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitZ()));
  B.translate(Eigen::Vector3d(r,0,0));
  B.rotate(Eigen::AngleAxisd(f, Eigen::Vector3d::UnitX()));
    
}

void HuboKin::armFK(Isometry3d &B, const Vector6d &q, int side) const {
  Isometry3d hand;
  hand(0,0) =  1; hand(0,1) =  0; hand(0,2) = 0; hand(0,3) =   0;
  hand(1,0) =  0; hand(1,1) =  0; hand(1,2) =-1; hand(1,3) =   0;
  hand(2,0) =  0; hand(2,1) =  1; hand(2,2) = 0; hand(2,3) =   0;
  hand(3,0) =  0; hand(3,1) =  0; hand(3,2) = 0; hand(3,3) =   1;
  armFK(B, q, side, hand);
}

void HuboKin::armFK(Isometry3d &B, const Vector6d &q, int side, 
                    const Isometry3d &endEffector) const {

  // Declarations
  Isometry3d neck, hand, T;

#ifndef HUBOKIN_USE_KCONSTANTS

  Eigen::MatrixXd limits(6,2);
  Vector6d offset; offset.setZero();

  // Parameters
  double l1 = 214.5/1000.0;
  double l2 = 179.14/1000.0;
  double l3 = 181.59/1000.0;
  double l4 = 4.75*25.4/1000.0;

#else

  const double& l1 = kc.arm_l1;
  const double& l2 = kc.arm_l2;
  const double& l3 = kc.arm_l3;
  const double& l4 = kc.arm_l4;
  //const Matrix62d limits = kc.getArmLimits(side);
  const Vector6d offset = kc.getArmOffset(side);

#endif
    
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
        
#ifndef HUBOKIN_USE_KCONSTANTS
    limits <<
      -2,   2,
      -2,  .3,
      -2,   2,
      -2,   0.01,
      -2,   2,
      -1.4, 1.2;
        
    // Set offset
    offset(1) = limits(1,1); // Note: I think this might be backwards
    //        offset(1) = -limits(1,1);
#endif
        
  } else {
    // Transformation from Neck frame to left shoulder pitch frame
    neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
    neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) =  l1;
    neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
    neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
        
#ifndef HUBOKIN_USE_KCONSTANTS
    limits <<
      -2,   2,
      -.3,   2,
      -2,   2,
      -2,   0.01,
      -2,   2,
      -1.4, 1.2;
        
    // Set offset
    offset(1) = limits(1,0); // Note: I think this might be backwards
    //        offset(1) = -limits(1,0);
#endif

  }
     
  // Calculate forward kinematics
  B = neck;
  for (int i = 0; i < 6; i++) {
    DH2HG(T, t(i)+q(i)-offset(i), f(i), r(i), d(i));
    B = B*T;
  }
  B = B*endEffector;
    
}

void HuboKin::armIK(Vector6d &q, const Isometry3d& B, const Vector6d& qPrev, int side) const {
  // Hand	
  Isometry3d hand;
  hand(0,0) =  1; hand(0,1) =  0; hand(0,2) = 0; hand(0,3) =   0;
  hand(1,0) =  0; hand(1,1) =  0; hand(1,2) =-1; hand(1,3) =   0;
  hand(2,0) =  0; hand(2,1) =  1; hand(2,2) = 0; hand(2,3) =   0;
  hand(3,0) =  0; hand(3,1) =  0; hand(3,2) = 0; hand(3,3) =   1;
  armIK(q, B, qPrev, side, hand);
}
  

void HuboKin::armIK(Vector6d &q, const Isometry3d& B, const Vector6d& qPrev, int side, 
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
    
#ifndef HUBOKIN_USE_KCONSTANTS

  Eigen::MatrixXd limits(6,2);
  Vector6d offset; offset.setZero();
  // Parameters
  double l1 = 214.5/1000.0;
  double l2 = 179.14/1000.0;
  double l3 = 181.59/1000.0;
  double l4 = 4.75*25.4/1000.0;

#else

  const double& l1 = kc.arm_l1;
  const double& l2 = kc.arm_l2;
  const double& l3 = kc.arm_l3;
  const double& l4 = kc.arm_l4;
  Matrix62d limits = kc.getArmLimits(side);
  Vector6d offset = kc.getArmOffset(side);

#endif

    
  if (side == SIDE_RIGHT) {
    // Transformation from Neck frame to right shoulder pitch frame
    neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
    neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) = -l1;
    neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
    neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;

#ifndef HUBOKIN_USE_KCONSTANTS

    limits <<
      -2,   2,
      -2,  .3,
      -2,   2,
      -2,   0.01,
      -2,   2,
      -1.4, 1.2;
        
    // Set offset
    offset(1) = limits(1,1); 

#endif
        
  } else {
    // Transformation from Neck frame to left shoulder pitch frame
    neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
    neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) =  l1;
    neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
    neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
        
#ifndef HUBOKIN_USE_KCONSTANTS

    limits <<
      -2,   2,
      -.3,   2,
      -2,   2,
      -2,   0.01,
      -2,   2,
      -1.4, 1.2;
        
    // Set offset
    offset(1) = limits(1,0); 

#endif

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
          q5 = wrapToPi(q1 - q3 - qT);
        } else { // Case 3b: q2 = -pi/2
          qT = atan2(-nz,sz);
          q5 = wrapToPi(qT - q1 - q3);
        }
                
                
      } else {
                
        if (C2 < 0) {
          qT = qT + M_PI;
        }
        q5 = wrapToPi(qT - q3);
                
        // Solve for q1
        q1 = atan2(S6*ny - C6*nx,C6*sx - S6*sy);
        if (C2 < 0) {
          q1 = q1 + M_PI;
        }
        q1 = wrapToPi(q1);
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
          q1 = wrapToPi(qT + q3);
        } else { // Case 2b: q2 = -pi/2
          qT = atan2(S6*sy - C6*sx,S6*ny - C6*nx);
          if (S4 < 0) {
            qT = qT + M_PI;
          }
          q1 = wrapToPi(qT - q3);
        }
                
      } else {
        q3 = atan2(S4*S6*ay - C4*S5*az - C6*S4*ax - C4*C5*C6*ay - C4*C5*S6*ax,C5*az - C6*S5*ay - S5*S6*ax);
        if (C2 < 0) {
          q3 = q3 - M_PI;
        }
        q3 = wrapToPi(q3);
                
        // Solve for q1
        q1 = atan2(C4*S6*ny - C4*C6*nx + S4*S5*nz + C5*C6*S4*ny + C5*S4*S6*nx,C4*C6*sx - C4*S6*sy - S4*S5*sz - C5*C6*S4*sy - C5*S4*S6*sx);
        if (C2 < 0) {
          q1 = q1 + M_PI;
        }
        q1 = wrapToPi(q1);
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
  for( int j=0; j<8; j++) {
    for (int i = 0; i < 6; i++) {
      if (side==SIDE_RIGHT) {
        qAll(i,j) = wrapToPi(qAll(i,j) + offset(i));
      } else {
        qAll(i,j) = wrapToPi(qAll(i,j) + offset(i));
      } 
    }
  }
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
              qDiff(j) = wrapToPi(qAll(j,i) - qPrev(j));
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
}

void HuboKin::legFK(Isometry3d &B, const Vector6d &q, int side) const {
  // Declarations
  Isometry3d neck, waist, T;

#ifndef HUBOKIN_USE_KCONSTANTS

  Eigen::MatrixXd limits(6,2);
  Vector6d offset; offset.setZero();
  
  // Parameters
  double l1 = (79.5+107)/1000.0;
  double l2 = 88.43/1000.0;
  double l3 = (289.47-107)/1000.0;
  double l4 = 300.03/1000.0;
  double l5 = 300.38/1000.0;
  double l6 = 94.97/1000.0;
  
#else

  const double& l1 = kc.leg_l1;
  const double& l2 = kc.leg_l2;
  const double& l3 = kc.leg_l3;
  const double& l4 = kc.leg_l4;
  const double& l5 = kc.leg_l5;
  const double& l6 = kc.leg_l6;
  //const Matrix62d limits = kc.getLegLimits(side);
  const Vector6d offset = kc.getLegOffset(side);

#endif

  // Denavit-Hartenberg parameters 
  Vector6d t, f, r, d;
  t <<       0, -M_PI/2,       0,       0,       0,       0;
  f <<  M_PI/2, -M_PI/2,       0,       0,  M_PI/2,       0;
  r <<       0,       0,      l4,      l5,       0,      l6;
  d <<       0,       0,       0,       0,       0,       0;
    
  // Transformation from Neck frame to Waist frame
  neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
  neck(1,0) = 0; neck(1,1) =  1; neck(1,2) = 0; neck(1,3) =   0;
  neck(2,0) = 0; neck(2,1) =  0; neck(2,2) = 1; neck(2,3) = -l1;
  neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
    
  if (side == SIDE_RIGHT) {
    // Transformation from Waist frame to right hip yaw frame
    waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
    waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) = -l2;
    waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
    waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;
        
#ifndef HUBOKIN_USE_KCONSTANTS

    limits <<
      -1.80,   0.0,
      -0.58,   0.0,
      -1.30,   1.30,
      0.0,     2.50,
      -1.26,   1.80,
      -0.23,   0.31;

#endif
        
  } else {
    // Transformation from Waist frame to left hip yaw frame
    waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
    waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) =  l2;
    waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
    waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;

#ifndef HUBOKIN_USE_KCONSTANTS

    limits <<
      0.0,     1.80,
      0.0,     0.58,
      -1.30,   1.30,
      0.0,     2.50,
      -1.26,   1.80,
      -0.31,   0.23;

#endif

  }
    
  // Calculate forward kinematics
  B = waist*neck;
  for (int i = 0; i < 6; i++) {
    DH2HG(T, t(i)+q(i)+offset(i), f(i), r(i), d(i));
    B = B*T;
  }
}

void HuboKin::legIK(Vector6d &q, const Isometry3d& B, const Vector6d& qPrev, int side) const {
  Eigen::ArrayXXd qAll(6,8);
    
  // Declarations
  Isometry3d neck, neckInv, waist, waistInv, BInv;
  double nx, sx, ax, px;
  double ny, sy, ay, py;
  double nz, sz, az, pz;
  double q1, q2, q3, q4, q5, q6;
  double C45, psi, q345;
  Eigen::Matrix<int, 8, 3> m;
    
  double S2, S4, S6;
  double C2, C4, C5, C6;

#ifndef HUBOKIN_USE_KCONSTANTS    

  Eigen::MatrixXd limits(6,2);
  Vector6d offset; offset.setZero();

  // Parameters
  double l1 = (79.5+107)/1000.0;
  double l2 = 88.43/1000.0;
  double l3 = (289.47-107)/1000.0;
  double l4 = 300.03/1000.0;
  double l5 = 300.38/1000.0;
  double l6 = 94.97/1000.0;

#else

  const double& l1 = kc.leg_l1;
  const double& l2 = kc.leg_l2;
  const double& l3 = kc.leg_l3;
  const double& l4 = kc.leg_l4;
  const double& l5 = kc.leg_l5;
  const double& l6 = kc.leg_l6;
  const Matrix62d limits = kc.getLegLimits(side);
  const Vector6d offset = kc.getLegOffset(side);

#endif

  // Transformation from Neck frame to Waist frame
  neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
  neck(1,0) = 0; neck(1,1) =  1; neck(1,2) = 0; neck(1,3) =   0;
  neck(2,0) = 0; neck(2,1) =  0; neck(2,2) = 1; neck(2,3) = -l1;
  neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
    
  if (side == SIDE_RIGHT) {
    // Transformation from Waist frame to right hip yaw frame
    waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
    waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) = -l2;
    waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
    waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;

#ifndef HUBOKIN_USE_KCONSTANTS        

    limits <<
      -1.80,   0.0,
      -0.58,   0.0,
      -1.30,   1.30,
      0.0,     2.50,
      -1.26,   1.80,
      -0.23,   0.31;

#endif
        
  } else {
    // Transformation from Waist frame to left hip yaw frame
    waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
    waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) =  l2;
    waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
    waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;

#ifndef HUBOKIN_USE_KCONSTANTS        

    limits <<
      0.0,     1.80,
      0.0,     0.58,
      -1.30,   1.30,
      0.0,     2.50,
      -1.26,   1.80,
      -0.31,   0.23;

#endif

  }
  neckInv = neck.inverse();
  waistInv = waist.inverse();
    
  // Variables
  BInv = (neckInv*waistInv*B).inverse();
    
  nx = BInv(0,0); sx = BInv(0,1); ax = BInv(0,2); px = BInv(0,3);
  ny = BInv(1,0); sy = BInv(1,1); ay = BInv(1,2); py = BInv(1,3);
  nz = BInv(2,0); sz = BInv(2,1); az = BInv(2,2); pz = BInv(2,3);

  m <<
    1,  1,  1,
    1,  1, -1,
    1, -1,  1,
    1, -1, -1,
    -1,  1,  1,
    -1,  1, -1,
    -1, -1,  1,
    -1, -1, -1;

  for (int i = 0; i < 8; i++)
    {
      C4 = ((l6 + px)*(l6 + px) - l4*l4 - l5*l5 + py*py + pz*pz)/(2*l4*l5);
      complex<double> radical = 1-C4*C4;
      q4 = atan2(m(i,0)*real(sqrt(radical)),C4);

      S4 = sin(q4);
      psi = atan2(S4*l4, C4*l4+l5);
      radical = ((px+l6)*(px+l6)+(py*py));
      q5 = wrapToPi(atan2(-pz, m(i,1)*real(sqrt(radical)))-psi);

      q6 = atan2(py, -px-l6);
      C45 = cos(q4+q5);
      C5 = cos(q5);
      if (C45*l4 + C5*l5 < 0)
        {
          q6 = wrapToPi(q6 + M_PI);
        }

      S6 = sin(q6);
      C6 = cos(q6);

      S2 = C6*ay + S6*ax;
      radical = 1-S2*S2;
      q2 = atan2(S2,m(i,2)*real(sqrt(radical)));

      q1 = atan2(C6*sy + S6*sx,C6*ny + S6*nx);
      C2 = cos(q2);
      if (C2 < 0) {
        q1 = wrapToPi(q1 + M_PI);
      }

      q345 = atan2(-az/C2,-(C6*ax - S6*ay)/C2);
      q3 = wrapToPi(q345-q4-q5);

      qAll(0,i) = q1;
      qAll(1,i) = q2;
      qAll(2,i) = q3;
      qAll(3,i) = q4;
      qAll(4,i) = q5;
      qAll(5,i) = q6;
    }

  // Set to offset
  for (int i = 0; i < 6; i++) {
    if (side==SIDE_RIGHT) {
      q(i) = wrapToPi(q(i) + offset(i));
    } else {
      q(i) = wrapToPi(q(i) + offset(i));
    }
  }

  // Find best solution
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
      for (int i = 0; i < 8; i++)
        {
          // if all the joints of solution i are within the limits...
          if( withinLim[i] )
            {
              // calculate the differences between solution angles, j, and previous angles
              for (int j=0; j < 6; j++)
                {
                  qDiff(j) = wrapToPi(qAll(j,i) - qPrev(j));
                }
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
      for(int i=0; i<8; i++)
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
          legFK( Btemp, qtemp, side );
          // calculate the distance from previous pose to temp pose locations
          qDiffSum(i) = (Btemp.translation() - B.translation()).norm();
        }
      // find the solution that's closest the previous position
      qDiffSum.minCoeff(&minInd);
      q = qAll.col(minInd);
    }
  // set the final joint angles to the solution closest to the previous solution
  for( int i=0; i<6; i++)
    q(i) = max( min( q(i), limits(i,1)), limits(i,0) ); 
}

}
