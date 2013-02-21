#include <iostream>
#include<stdio.h>
#include<cmath>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class limb {

public:
  MatrixXf trans;// Contains all 6 4x4 matrices
  MatrixXf fk;
  MatrixXf J;
  MatrixXf pose;

  MatrixXf jacobian(VectorXf q){ 

    trans.resize(24,4);

    MatrixXf temp(4,4);
    temp << MatrixXf::Identity(4,4);
    J.resize(6,6);

    fk.resize(4,4);
    fk << MatrixXf::Identity(4,4);
    for (int i=0;i<6;i++){
      fk = fk*trans.block<4,4>(4*i,0);
      for (int j=i;j<6;j++){
         temp = temp*trans.block<4,4>(4*j,0);
      }
      temp = temp.inverse();
      J.block<3,1>(0,i) << temp.block<3,1>(0,3).cross(fk.block<3,1>(0,2));
      J.block<3,1>(3,i) << fk.block<3,1>(0,2);
      temp << MatrixXf::Identity(4,4);

      return J
    }
  }
} lleg, rleg, larm,rarm;

void trans(float q[]){

  // Define homogeneous transformation matrices for the left leg  
  lleg.trans << -sin(q(4)), -cos(q(4)), 0, 0, cos(q(4)) ,  -sin(q(4)), 0, 0.089, 0,  0, 1, 0, 0,0, 0, 1,
    cos(q(5)), -sin(q(5)),0, 0,0, 0, -1, 0, sin(q(5)), cos(q(5)), 0, 0, 0, 0, 0, 1,
    0, 0, 1, 0, -cos(q(6)), sin(q(6)),0, -0.05,-sin(q(6)), -cos(q(6)), 0, -0.05, 0, 0, 0,1,
    cos(q(7)), -sin(q(7)), 0, 0.374, sin(q(7)),  cos(q(7)), 0,   -0.05, 0,0, 1,0,0,0, 0, 1,
    cos(q(8)), -sin(q(8)), 0, 0.422, sin(q(8)),  cos(q(8)), 0, 0,0, 0, 1, 0,0, 0, 0, 1,
    cos(q(9)), -sin(q(9)),0, 0,0, 0, -1, 0,sin(q(9)), cos(q(9)),0, 0,0, 0,  0, 1;

  // Define homogeneous transformation matrices for the right leg  
  rleg.trans << -sin(q(10)), -cos(q(10)), 0, 0, cos(q(10)) ,  -sin(q(10)), 0, -0.089, 0,  0, 1, 0, 0,0, 0, 1,
    cos(q(11)), -sin(q(11)),0, 0,0, 0, -1, 0, sin(q(11)), cos(q(11)), 0, 0, 0, 0, 0, 1,
    0, 0, 1, 0, -cos(q(12)), sin(q(12)),0, -0.05,-sin(q(12)), -cos(q(12)), 0, -0.05, 0, 0, 0,1,
    cos(q(13)), -sin(q(13)), 0, 0.374, sin(q(13)),  cos(q(13)), 0,   -0.05, 0,0, 1,0,0,0, 0, 1,
    cos(q(14)), -sin(q(14)), 0, 0.422, sin(q(14)),  cos(q(14)), 0, 0,0, 0, 1, 0,0, 0, 0, 1,
    cos(q(15)), -sin(q(15)),0, 0,0, 0, -1, 0,sin(q(15)), cos(q(15)),0, 0,0, 0,  0, 1;

  // Define homogeneous transformation matrices for the left arm
  larm.trans << cos(q(16)),-sin(q(16)),0, 0,0.866*sin(q(16)), 0.866*cos(q(16)),0.5, 0,-0.5*sin(q(16)),-0.5*cos(q(16)), 0.866, 0,0, 0,0, 1,
    0, 0,1,0,0.5*sin(q(17)) + 0.866*cos(q(17)),0.5*cos(q(17)) - 0.866*sin(q(17)), 0, 0,0.5*cos(q(17)) - 0.866*sin(q(17)),-0.5*sin(q(17) - 0.866*cos(q(17))),   0, 0,0,0,0,1,
    0,0,-1, 0.185,sin(q(18)),cos(q(18)),0,0,cos(q(18)),-sin(q(2)),0,0,0,0,0,1,
    0,0,1,0,sin(q(19)), cos(q(19)),0,-0.013,-cos(q(19)), sin(q(19)), 0, -0.121,0,0,0,1,
    0,0,-1, 0.188,sin(q(20)),cos(q(20)),0, 0.013,cos(q(20)),-sin(q(20)), 0,0,0,0,0,1,
    0,0,1,0,sin(q(21)),cos(q(21)),0,0,-cos(q(21)),sin(q(21)),0, -0.058,0,0,0,1;

  // Define homogeneous transformation matrices for the right arm
  // Due to unconventional notation, this is probably wrong
  rarm.trans << -cos(-q(22)),sin(-q(22)),0, 0,-0.866*sin(-q(22)), -0.866*cos(-q(22)),-0.5, 0,-0.5*sin(-q(22)),-0.5*cos(-q(22)),0.866, 0,0,0,0, 1,
    0, 0,1,0,0.5*sin(-q(23)) + 0.866*cos(-q(23)),0.5*cos(-q(23)) - 0.866*sin(-q(23)), 0, 0,0.5*cos(-q(23)) - 0.866*sin(-q(23)),-0.5*sin(-q(23)) - 0.866*cos(-q(23)),   0, 0,0,0,0,1,
    0,0,-1, 0.185,sin(-q(24)),cos(-q(24)),0,0,cos(-q(24)),-sin(-q(24)),0,0,0,0,0,1,
    0,0,1,0,sin(-q(25)), cos(-q(25)),0,-0.013,-cos(-q(25)), sin(-q(25)), 0, -0.121,0,0,0,1,
    0,0,-1, 0.188,sin(-q(26)),cos(-q(26)),0, 0.013,cos(-q(26)),-sin(-q(26)), 0,0,0,0,0,1,
    0,0,1,0,sin(-q(27)),cos(-q(27)),0,0,-cos(-q(27)),sin(-q(27)),0, -0.058,0,0,0,1;

}



int main(){

  VectorXf q(6);
  q<< 1.5708,1.0472,0.7854,0.6283,0.3491,0.2618;
  MatrixXf J;
  trans();
  J = lleg.jacobian(q);

  VectorXf delQ(6);
  VectorXf T(6);
  VectorXf  z(6);

  delQ << 0.005,-0.003, 0.007,0.006,0.003,0.004;
  T << 0.002,0,0,0,0,0;

  delQ = lleg.J.inverse()*T;
  z =J*delQ;

  std::cout << z << std::endl;

}	
