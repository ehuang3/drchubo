#include "robot_jacobians.h"

using namespace Eigen;
using namespace std;

namespace robot {

robot_jacobians_t::robot_jacobians_t() {
}

robot_jacobians_t::~robot_jacobians_t() {}

void robot_jacobians_t::getJacobian(MatrixXd& J, const MatrixXd& indexes, const VectorXd& dofs) {
    // Set robot pose
    robot->setPose(dofs);
    // Resize J to correct r c
    int rows = indexes.rows();
    int cols = indexes.cols();
    J.resize(rows, cols*6);
    // 
    
}

}
