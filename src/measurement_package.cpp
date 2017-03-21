//
// Created by Matthew Zimmer on 3/18/17.
//

#include "measurement_package.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"
Eigen::VectorXd MeasurementPackage::estimations() {
  Eigen::VectorXd result;

  // If RADAR sensor type, convert estimation from
  // polar (rho, phi) to cartesian (x, y) coordinates.
  if(sensor_type_ == SensorType::RADAR) {
    result = Eigen::VectorXd(3);

    // RADAR estimation
    float rho = raw_measurements_(0);
    float phi = raw_measurements_(1);
    float rho_dot = raw_measurements_(2);

//    const float TWO_PI = 2*M_PI;
//    phi = (phi - -1*M_PI)/(M_PI- -1*M_PI);

    result << rho,
              phi,
              rho_dot;

  } else {
    result = Eigen::VectorXd(2);

    // LIDAR estimation
    float px = raw_measurements_(0);
    float py = raw_measurements_(1);

    result << px,
              py;
  }


  return result;
}
#pragma clang diagnostic pop