#include <iostream>
#include "tools.h"

using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

Eigen::VectorXd Tools::CalculateRMSE(const vector<Eigen::VectorXd> &estimations,
                              const vector<Eigen::VectorXd> &ground_truth) {
  Eigen::VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  int n1 = estimations.size();
  int n2 = ground_truth.size();

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if((n1+n2 == 0) || (n1 != n2)) {
    std::cout << "estimations and ground truth vectors must not be empty and must be of equal dimensions." << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  Eigen::VectorXd residual;
  for (int i = 0; i < n1; ++i) {
    residual = estimations[i]-ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  rmse = rmse.array()/n1; //calculates the mean
  rmse = rmse.array().sqrt(); //calculates the squared root
  return rmse;

}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd &x_state) {
  Eigen::MatrixXd Hj(3,4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  if(px+py == 0) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  //compute the Jacobian matrix
  float px_py_sqrd_prod = pow(px, 2.0) + pow(py, 2.0);
  float sqrt_px_py_sqrd_prod = sqrt(px_py_sqrd_prod);
  float pow_px_py_sqrd_prod = pow(px_py_sqrd_prod, 3.0/2.0);

  Hj(0, 0) = px/sqrt_px_py_sqrd_prod; // dh(row)/px
  Hj(0, 1) = py/sqrt_px_py_sqrd_prod; // dh(row)/py
  Hj(0, 2) = 0; // dh(row)/vx
  Hj(0, 3) = 0; // dh(row)/vy

  Hj(1, 0) = -py/px_py_sqrd_prod; // dh(phi)/xpx
  Hj(1, 1) = px/px_py_sqrd_prod; // dh(phi)/xpy
  Hj(1, 2) = 0; // dh(phi)/vx = d/dvx*arctan(py/px)
  Hj(1, 3) = 0; // dh(phi)/vy = d/dvy*arctan(py/px)

  Hj(2, 0) = (py*(vx*py-vy*px))/pow_px_py_sqrd_prod; // dh(rho_dot)/px
  Hj(2, 1) = (px*(vy*px-vx*py))/pow_px_py_sqrd_prod; // dh(rho_dot)/py
  Hj(2, 2) = px/sqrt_px_py_sqrd_prod; // dh(rho_dot)/vx
  Hj(2, 3) = py/sqrt_px_py_sqrd_prod; // dh(rho_dot)/vy

  return Hj;
}
