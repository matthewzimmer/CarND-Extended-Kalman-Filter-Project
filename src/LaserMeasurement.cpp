//
// Created by Matthew Zimmer on 3/21/17.
//

#include "LaserMeasurement.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"

LaserMeasurement::LaserMeasurement() {
  R_ = Eigen::MatrixXd(2, 2);
  H_ = Eigen::MatrixXd(2, 4);

  H_ << 1, 0, 0, 0,
          0, 1, 0, 0;

  //measurement covariance matrix - laser
  R_ << 0.0225, 0,
          0, 0.0225;
}

void LaserMeasurement::Update(KalmanFilter &ekf) {
  ekf.Init(ekf.x_, ekf.P_, ekf.F_, H_, R_, ekf.Q_);
  ekf.Update(raw_measurements_);
}

#pragma clang diagnostic pop