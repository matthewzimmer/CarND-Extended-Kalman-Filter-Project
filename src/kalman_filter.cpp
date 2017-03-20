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

//  cout << "x_= " << x_ << endl;
//  cout << "P_= " << P_ << endl;
//  cout << "F_= " << F_ << endl;
//  cout << "H_= " << H_ << endl;
//  cout << "R_= " << R_ << endl;
//  cout << "Q_= " << Q_ << endl << endl;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
//  cout << "x_ " << x_ << endl;

  Eigen::MatrixXd Ft = F_.transpose();
//  cout << "Ft= " << Ft << endl;

  P_ = F_ * P_ * Ft + Q_;
//  cout << "P_= " << P_ << endl;
}

void KalmanFilter::Update(const Eigen::VectorXd &z) {
  Eigen::VectorXd y = z - H_ * x_; // error calculation given our new measurement z
  Eigen::MatrixXd Ht = H_.transpose(); // H matrix transposed
  Eigen::MatrixXd S = H_ * P_ * Ht + R_; // S matrix
  Eigen::MatrixXd Si = S.inverse(); // S' inverse matrix
  Eigen::MatrixXd K = P_ * Ht * Si; // The Kalman Gain

  //new updated state x_ and covariance matrix P_
  x_ = x_ + (K * y);
  long x_size = x_.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Eigen::VectorXd y = z - H_ * x_;
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd S = H_ * P_ * Ht + R_;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K = P_ * Ht * Si;

  //new updated state x_ and covariance matrix P_
  x_ = x_ + (K * y);
  long x_size = x_.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

#pragma clang diagnostic pop