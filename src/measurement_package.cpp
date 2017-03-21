//
// Created by Matthew Zimmer on 3/18/17.
//

#include "measurement_package.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"

Eigen::VectorXd MeasurementPackage::estimations() {
  Eigen::VectorXd result;
  if(sensor_type_ == RADAR) {
    // RADAR estimation
    result = Eigen::VectorXd(3);

    float rho = raw_measurements_(0);
    float phi = raw_measurements_(1);
    float rho_dot = raw_measurements_(2);

    result << rho,
              phi,
              rho_dot;
  } else {
    // LIDAR estimation
    result = Eigen::VectorXd(2);

    float px = raw_measurements_(0);
    float py = raw_measurements_(1);

    result << px,
              py;
  }

  return result;
}

#pragma clang diagnostic pop