//
// Created by Matthew Zimmer on 3/21/17.
//

#ifndef EXTENDEDKF_LASER_MEASUREMENT_H
#define EXTENDEDKF_LASER_MEASUREMENT_H


#include "measurement_package.h"
#include "kalman_filter.h"

class LaserMeasurement : public MeasurementPackage {
public:
    LaserMeasurement();

    virtual ~LaserMeasurement() {};

    void Update(KalmanFilter &ekf);

private:
    Eigen::MatrixXd R_;
    Eigen::MatrixXd H_;
};


#endif //EXTENDEDKF_LASER_MEASUREMENT_H
