#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {
  /**
   * Compute the Jacobian Matrix
   */

  // predicted state example
  // px = 1, py = 2, vx = 0.2, vy = 0.4
  VectorXd x_predicted(4);
  x_predicted << 1, 2, 0.2, 0.4;

  MatrixXd Hj = CalculateJacobian(x_predicted);

  cout << "Hj:" << endl << Hj << endl;

  return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  if( (px == 0) && (py == 0) ) {
      cout << "Divide by zero error";
  } else {
      float px2y2 = px*px + py*py;
      float px2y2_root = sqrt(px2y2);
      float px2y2_32 = pow(px2y2_root, 3);

      Hj << px/px2y2_root, py/px2y2_root, 0, 0,
            -py/px2y2, px/px2y2, 0, 0,
            py*(vx*py-vy*px)/px2y2_32, px*(vy*px-vx*py)/px2y2_32, px/px2y2_root, py/px2y2_root;
  }

  return Hj;
}