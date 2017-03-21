#include <iostream>
#include "kalman_filter.h"

using namespace std;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
                        Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  Eigen::MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const Eigen::VectorXd &z) {
  Eigen::VectorXd z_pred = H_ * x_;
  Eigen::VectorXd y = z - z_pred; // error calculation given our new measurement z
  Eigen::MatrixXd Ht = H_.transpose(); // H matrix transposed
  Eigen::MatrixXd S = H_ * P_ * Ht + R_; // S matrix
  Eigen::MatrixXd Si = S.inverse(); // S' inverse matrix
  Eigen::MatrixXd K = P_ * Ht * Si; // The Kalman Gain

  //new updated state x_ and covariance matrix P_
  x_ = x_ + (K * y);
  size_t x_size = x_.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

//  Predict();
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
    *
    * Classroom inspiration for this algorithm: https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/3612b91d-9c33-47ad-8067-a572a6c93837/concepts/f3b2b918-00d5-4af0-9363-410d01b0a1a7
  */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  // maps x_ from Cartesian coordinates to polar coordinates
  float rho = sqrt(px*px + py*py); // The range, ρ, is the distance to the pedestrian
  float theta = atan(py/px); // φ is the angle between ρ and the x direction
  float rho_dot = (px*vx + py*vy)/rho; // The range rate, ​ρ​˙, is the projection of the velocity, v

  // The second value in the polar coordinate vector is the angle ϕ. Need to make sure to normalize ϕ in the y vector
  // so that its angle is between -pi and pi; in other words, add or subtract 2*pi from ϕ until it is
  // between -pi and pi.
//  theta = (theta - -1*M_PI)/(M_PI - -1*M_PI);

  Eigen::VectorXd hx = Eigen::VectorXd(3);
  hx << rho, theta, rho_dot;

  Eigen::VectorXd y = z - hx;
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd S = H_ * P_ * Ht + R_;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K = P_ * Ht * Si;

  //new updated state x_ and covariance matrix P_
  x_ = x_ + (K * y);
  size_t x_size = x_.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

#pragma clang diagnostic pop