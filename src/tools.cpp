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

  //TODO: YOUR CODE HERE

  //check division by zero
  if(px+py == 0) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }

  //compute the Jacobian matrix
  double px_py_sqrd_sum = (px*px + py*py);
  double rho = sqrt(px_py_sqrd_sum);
  double phi = atan(py/px);
  double rho_dot = (px*vx+py*vy)/rho;
  double pow_px_py_sqrd_sum = pow(px_py_sqrd_sum, 3.0/2.0);

  Hj(0, 0) = px/rho; // dh(rho)/dpx == "derivative of rho with respect to px"
  Hj(0, 1) = py/rho; // dh(rho)/py == "derivative of rho with respect to py"
  Hj(0, 2) = 0; // dh(rho)/dvx == "derivative of rho with respect to vx"
  Hj(0, 3) = 0; // dh(rho)/dvy == "derivative of rho with respect to vy"

  Hj(1, 0) = -py/px_py_sqrd_sum; // dh(phi)/dpx == "derivative of phi with respect to px"
  Hj(1, 1) = px/px_py_sqrd_sum; // dh(phi)/dpy == "derivative of phi with respect to py"
  Hj(1, 2) = 0; // dh(phi)/vx = dh(phi)/dvx*arctan(py/px) == "derivative of rho with respect to px"
  Hj(1, 3) = 0; // dh(phi)/vy = dh(phi)/dvy*arctan(py/px)

  Hj(2, 0) = (py*(vx*py-vy*px))/pow_px_py_sqrd_sum; // dh(rho_dot)/px
  Hj(2, 1) = (px*(vy*px-vx*py))/pow_px_py_sqrd_sum; // dh(rho_dot)/py
  Hj(2, 2) = px/rho; // dh(rho_dot)/vx
  Hj(2, 3) = py/rho; // dh(rho_dot)/vy

  return Hj;
}
