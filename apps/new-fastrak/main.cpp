#include "Fastrak.h"
#include <Eigen/Dense>
#include <iostream>

Fastrak fastrak;

int main() {
    
    Eigen::Isometry3d pose;
    
    while(1) {
        fastrak.getPose(pose, 1);
        std::cout << "pose = \n" << Eigen::Quaterniond(pose.linear()).vec().transpose() << std::endl;
    }
    

}
