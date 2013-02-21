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

} lleg;

void jacobian(VectorXf q){

  lleg.trans.resize(24,4);// <<MatrixXf::Zero(24,4);

//Define Homogeneous transformation matrices for the left leg  
  lleg.trans << -sin(q(0)), -cos(q(0)), 0, 0, cos(q(0)) ,  -sin(q(0)), 0, 0.089, 0,  0, 1, 0, 0,0, 0, 1,
    cos(q(1)), -sin(q(1)),0, 0,0, 0, -1, 0, sin(q(1)), cos(q(1)), 0, 0, 0, 0, 0, 1,
    0, 0, 1, 0, -cos(q(2)), sin(q(2)),0, -0.05,-sin(q(2)), -cos(q(2)), 0, -0.05, 0, 0, 0,1,
    cos(q(3)), -sin(q(3)), 0, 0.374, sin(q(3)),  cos(q(3)), 0,   -0.05, 0,0, 1,0,0,0, 0, 1,
    cos(q(4)), -sin(q(4)), 0, 0.422, sin(q(4)),  cos(q(4)), 0, 0,0, 0, 1, 0,0, 0, 0, 1,
    cos(q(5)), -sin(q(5)),0, 0,0, 0, -1, 0,sin(q(5)), cos(q(5)),0, 0,0, 0,  0, 1;

  MatrixXf temp(4,4);
  temp << MatrixXf::Identity(4,4);
  lleg.J.resize(6,6);

  lleg.fk.resize(4,4);
  lleg.fk << MatrixXf::Identity(4,4);
  for (int i=0;i<6;i++){
    lleg.fk = lleg.fk*lleg.trans.block<4,4>(4*i,0);
    for (int j=i;j<6;j++){
        temp = temp*lleg.trans.block<4,4>(4*j,0);
    }
    if (i==0){
//std::cout << lleg.trans.block<4,4>(0,0) << std::endl;
    }
    temp = temp.inverse();
	/*if (i ==0){
	  std::cout << temp.block<3,1>(0,3) << std::endl;
          std::cout << lleg.fk.block<3,1>(0,2) << std::endl;
	}*/
    lleg.J.block<3,1>(0,i) << temp.block<3,1>(0,3).cross(lleg.fk.block<3,1>(0,2));
    lleg.J.block<3,1>(3,i) << lleg.fk.block<3,1>(0,2);
    temp << MatrixXf::Identity(4,4);
  }
}
int main(){

VectorXf q(6);
q<< 1.5708,1.0472,0.7854,0.6283,0.3491,0.2618;

jacobian(q);

VectorXf delQ(6);
VectorXf T(6);
VectorXf  z(6);

delQ << 0.005,-0.003, 0.007,0.006,0.003,0.004;
T << 0.002,0,0,0,0,0;

delQ = lleg.J.inverse()*T;
z =lleg.J*delQ;

std::cout << z << std::endl;

}	
