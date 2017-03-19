//
// Created by Matthew Zimmer on 3/18/17.
//

#include "measurement_package.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"
Eigen::VectorXd MeasurementPackage::estimations() {
  Eigen::VectorXd result;
  result = Eigen::VectorXd(2);

  // LIDAR estimation
  float px = raw_measurements_(0);
  float py = raw_measurements_(1);

  // If RADAR sensor type, convert estimation from
  // polar (rho, phi) to cartesian (x, y) coordinates.
  if(sensor_type_ == SensorType::RADAR) {
    px = px * cos(py); // px == rho
    py = px * sin(py); // py == phi
  }

  result << px,
            py;

  return result;
}
#pragma clang diagnostic pop