#include "FusionEKF.h"
#include "measurement_package.h"
#include <iostream>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  /**
   * Set the process and measurement noises
   */

  // first measurement
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 1, 1, 1, 1;


  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);

  // noise covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);

  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

bool FusionEKF::ProcessMeasurement(MeasurementPackage &measurement_pack) {
  Eigen::VectorXd z = measurement_pack.raw_measurements_;

  // prevent devision by zero
  float px = z(0);
  float py = z(1);
  if (px == 0 || py == 0) {
    return false;
  }

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
    */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state
      // output the estimation in the cartesian coordinates. We do the conversion to
      // cartesian because we always want x_ in cartesian coordinates.
      float rho = z(0);
      float phi = z(1);
      ekf_.x_ << rho * cos(phi), rho * sin(phi), 0, 0;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << px, py, 0, 0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return true;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  //1. Modify the F matrix so that the time is integrated (Time is measured in seconds)
  ekf_.F_ << 1, 0, dt, 0,
          0, 1, 0, dt,
          0, 0, 1, 0,
          0, 0, 0, 1;

  //2. Set the process noise covariance matrix Q
  ekf_.Q_ << ((pow(dt, 4.0)) / 4.0) * noise_ax, 0, ((pow(dt, 3.0)) / 2.0) * noise_ax, 0,
          0, ((pow(dt, 4.0)) / 4.0) * noise_ay, 0, ((pow(dt, 3.0)) / 2.0) * noise_ay,
          ((pow(dt, 3.0)) / 2.0) * noise_ax, 0, pow(dt, 2.0) * noise_ax, 0,
          0, ((pow(dt, 3.0)) / 2.0) * noise_ay, 0, pow(dt, 2.0) * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  measurement_pack.Update(ekf_);

  return true;
}

#pragma clang diagnostic pop