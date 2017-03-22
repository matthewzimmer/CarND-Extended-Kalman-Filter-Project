//
// Created by Matthew Zimmer on 3/21/17.
//

#include "RadarMeasurement.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"

RadarMeasurement::RadarMeasurement() {
  H_ = Eigen::MatrixXd(3, 4);
  R_ = Eigen::MatrixXd(3, 3);

  //measurement covariance matrix - radar
  R_ << 0.09, 0, 0,
          0, 0.0009, 0,
          0, 0, 0.09;
}

void RadarMeasurement::Update(KalmanFilter &ekf) {
  H_ = tools.CalculateJacobian(ekf.x_);
  ekf.Init(ekf.x_, ekf.P_, ekf.F_, H_, R_, ekf.Q_);
  ekf.UpdateEKF(raw_measurements_);
}

#pragma clang diagnostic pop