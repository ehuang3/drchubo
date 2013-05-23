/**
 * @file ZmpPreview.h
 */

#ifndef __ZMPPREVIEW_H__
#define __ZMPPREVIEW_H__

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>

/*

  Class to implement a ZMP preview controller.

  This class is based on the papers:

  [1] Kajita, Shuuji, et al. "Biped walking pattern generation by
      using preview control of zero-moment point." Proc. IEEE Int'l
      Conf. on Robotics and Automation (ICRA), IEEE 2003.

  [2] Park, Jonghoon, and Youngil Youm. "General ZMP preview control
      for bipedal walking." Proc. IEEE Int'l Conf. on Robotics and
      Automation (ICRA), IEEE 2007

  Written for E91 Humanoid Robotics, Spring 2013, by Matt Zucker."""

*/

/**
 * @class ZmpPreview
 */
class ZmpPreview {

 public:
  
  double T, h, g;
  size_t nl;
  
  // drift, control and output matrices for state equation
  Eigen::Matrix3d A;
  Eigen::Vector3d B;
  Eigen::RowVector3d C;
  
  // drift and control matrices for augmented state equation
  Eigen::Matrix4d AA;
  Eigen::Vector4d BB;
  
  // LQR terms
  double RR;
  Eigen::Matrix4d QQ;
  
  // Unknown in Riccati equation
  Eigen::Matrix4d PP;
  
  // integral gain and state feedback gain
  double Ks;
  Eigen::RowVector3d Kx;
  
  // Preview controller gain term
  Eigen::RowVectorXd G;
  
  bool debug;
  
  ZmpPreview( double timestep,
	      double height,
	      size_t numlookahead,
	      double R=1e-6,
	      double Qe=1,
	      double Qdpos=0,
	      double Qdvel=0,
	      double Qdaccel=0,
	      double gaccel=9.8,
	      bool d=false ) {

    debug = d;
    

    T = timestep;
    h = height;
    nl = numlookahead;
    g = gaccel;

    // state equation terms
    A << 
      1, T, T*T/2, 
      0, 1, T,
      0, 0, 1;

    B << T*T*T/6, T*T/2, T;

    C << 1, 0, -h/g;
    
    // augmented state equation terms
    AA(0,0) = 1;
    AA.block<1,3>(0,1) = C*A;
    AA.block<3,1>(1,0).setZero();
    AA.block<3,3>(1,1) = A;
    
    BB(0,0) = C*B;
    BB.block<3,1>(1,0) = B;

    // LQR terms in eq. J = x^T*Q*x + u^T*R*u
    QQ << 
      Qe, 0, 0, 0,
      0, Qdpos, 0, 0, 
      0, 0, Qdvel, 0,
      0, 0, 0, Qdaccel;

    RR = R;
    

    // solve the Riccati equation iteratively for PP
    PP.setIdentity();
    bool converged = false;
    
    for (size_t i=0; i<10000; ++i) {
      Eigen::Matrix4d AX = AA.transpose() * PP;
      Eigen::Matrix4d AXA = AX * AA;
      Eigen::Vector4d AXB = AX * BB;
      double M = RR + BB.transpose()*PP*BB;
      Eigen::Matrix4d PPnew = AXA - AXB*(1/M)*AXB.transpose() + QQ;
      double relerr = (PPnew-PP).norm() / PPnew.norm();
      PP = PPnew;
      if (relerr < 1e-10) {
	std::cerr << "DARE solver converged after " << i << " iterations.\n";
	converged = true;
	break;
      }
    }

    if (!converged) {
      std::cerr << "*** WARNING: DARE solver failed to converge in ZmpPreview ***\n";
    }

    
    // construct K in the state feedback equation u = -Kx
    double SS = 1.0/(RR + BB.transpose() * PP * BB);
    Eigen::RowVector4d KK = SS * BB.transpose()*PP*AA;
    Ks = KK(0,0); // intergral gain
    Kx = KK.block<1,3>(0,1); // state feedback gain
    
    // new state feedback drift term (A matrix)
    Eigen::Matrix4d Ac = AA - BB*KK;
    Eigen::Vector4d CC;
    CC << 1, 0, 0, 0;

    // construct preview action gain terms
    Eigen::Vector4d XX = -Ac.transpose() * PP * CC;
    G = Eigen::RowVectorXd(nl);
    G(0) = -Ks;
    for (size_t i=1; i<nl; ++i) {
      G(i) = (SS * BB.transpose() * XX);
      XX = Ac.transpose() * XX;
    }

    
    if (debug) {
      std::cerr << "A=\n" << A << "\n\n";
      std::cerr << "B=\n" << B << "\n\n";
      std::cerr << "C=\n" << C << "\n\n";
      std::cerr << "AA=\n" << AA << "\n\n";
      std::cerr << "BB=\n" << BB << "\n\n";
      std::cerr << "QQ=\n" << QQ << "\n\n";
      std::cerr << "RR=\n" << RR << "\n\n";
      std::cerr << "PP=\n" << PP << "\n\n";
      std::cerr << "G(start) = " << G.leftCols(4) << "\n";
      std::cerr << "G(end) = " << G.rightCols(4) << "\n";
      std::cerr << "G size = " << G.cols() << "\n";
      std::cerr << "G.sum() = " << std::setprecision(10) << double(G.sum()) << "\n";
    }
    
  }
  
  /*
   * @function: update(Eigen::Vector3d& X,
   *                   double& e,
   *                   const Eigen::ArrayBase<Derived>& zmpref))
   * @brief: updates the zmp error and zmp
   */
  template <class Derived> double update(Eigen::Vector3d& X,
					 double& e,
					 const Eigen::ArrayBase<Derived>& zmpref) const {
    
    size_t nref = zmpref.rows();
    assert(nref);
    
    // controller: u_k = -K_i*e(k) - K_x*x(k)
    double u = -Ks*e - Kx*X;
    double zs = 0;

    // add on preview action gain using future reference: -sum(G_p(j)*zmp_ref(k+j)
    for (size_t i=0; i<nl; ++i) {
      double zr = zmpref(i < nref ? i : nref-1);
      u -= zr * G(i);
      zs +=  zr;
    }
 
    // final state equation
    X = A*X + B*u;
    double zmp = C*X;

    // zmp error
    e += zmp - zmpref(0);

    if (debug) {
      std::cerr << "zrng.sum() = " << zs << "\n";
      std::cerr << "u = " << u << "\n";
      std::cerr << "X = " << X.transpose() << "\n";
      std::cerr << "zmp = " << zmp << "\n";
      std::cerr << "zmpref[0] = " << zmpref(0) << "\n";
      std::cerr << "e = " << e << "\n\n";
    }
    
    return zmp;
  }
	     
};

#endif /** __ZMPPREVIEW_H__ */
