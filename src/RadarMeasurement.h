//
// Created by Matthew Zimmer on 3/21/17.
//

#ifndef EXTENDEDKF_RADAR_MEASUREMENT_H
#define EXTENDEDKF_RADAR_MEASUREMENT_H


#include "measurement_package.h"
#include "tools.h"
#include "kalman_filter.h"

class RadarMeasurement : public MeasurementPackage {
public:
    RadarMeasurement();

    virtual ~RadarMeasurement() {};

    void Update(KalmanFilter &ekf);

private:
    Tools tools;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd H_;
};


#endif //EXTENDEDKF_RADAR_MEASUREMENT_H
