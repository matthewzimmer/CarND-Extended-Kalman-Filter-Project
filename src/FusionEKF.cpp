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

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
//  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
                   0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
                 0, 0.0009, 0,
                 0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  //measurement matrix
  ekf_.H_ = MatrixXd(2, 4);
  ekf_.H_ << 1, 0, 0, 0,
             0, 1, 0, 0;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

bool FusionEKF::ProcessMeasurement(MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix H, F, R (they're initialized in constructor).
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    cout << "Initialize EKF: " << endl;

    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    Eigen::VectorXd measurement_estimations = measurement_pack.estimations();
    float px = measurement_estimations[0];
    float py = measurement_estimations[1];

    if (px == 0 || py == 0){
      cout << "Error in initializing state matrix";
      return false;
    }

    ekf_.x_ << px, py, 0, 0;

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return true;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
//  cout << "dt= " << dt << endl;

  //1. Modify the F matrix so that the time is integrated
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  //2. Set the process noise covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << ((pow(dt, 4.0)) / 4.0) * noise_ax, 0, ((pow(dt, 3.0)) / 2.0) * noise_ax, 0,
          0, ((pow(dt, 4.0)) / 4.0) * noise_ay, 0, ((pow(dt, 3.0)) / 2.0) * noise_ay,
          ((pow(dt, 3.0)) / 2.0) * noise_ax, 0, pow(dt, 2.0) * noise_ax, 0,
          0, ((pow(dt, 3.0)) / 2.0) * noise_ay, 0, pow(dt, 2.0) * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  Eigen::VectorXd z_;
  z_ = measurement_pack.estimations();
//  cout << "z= " << z_ << endl;

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
//    // Radar updates
//    Hj_ = tools.CalculateJacobian(ekf_.x_);
//
//    // y = z - Hj * x
//    // S = Hj * P * Hj^T + R_radar_
//    // K = P *Hj^T * S^-1
//    // x' = x + K * y
//    // P' = (I-K*Hj)*P
//
//    VectorXd y = z_ - Hj_ * ekf_.x_;
//    MatrixXd Ht = ekf_.H_.transpose();
//    MatrixXd S = ekf_.H_ * ekf_.P_ * Ht + R_laser_;
//    MatrixXd Sinv = S.inverse();
//    MatrixXd K = ekf_.P_ * Ht * Sinv;
//
//    // new state
//    ekf_.x_ = ekf_.x_ + K * y;
//    size_t sizeX = ekf_.x_.size();
//    MatrixXd I = MatrixXd::Identity(sizeX, sizeX);
//    ekf_.P_ = (I - K * ekf_.H_) * ekf_.P_;

//    ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, Hj_, R_radar_, ekf_.Q_);
//    ekf_.UpdateEKF(z_);

    return false;

  } else {
    // Laser updates
    // y = z - H * x
    // S = H * P * H^T + R_laser_
    // K = P *H^T * S^-1
    // x' = x + K * y
    // P' = (I-K*H)*P

//    // KF Measurement update step
//    VectorXd y = z_ - ekf_.H_ * ekf_.x_;
//    MatrixXd Ht = ekf_.H_.transpose();
//    MatrixXd S = ekf_.H_ * ekf_.P_ * Ht + R_laser_;
//    MatrixXd Sinv = S.inverse();
//    MatrixXd K = ekf_.P_ * Ht * Sinv;
//
//    // new state
//    ekf_.x_ = ekf_.x_ + K * y;
//    size_t sizeX = ekf_.x_.size();
//    MatrixXd I = MatrixXd::Identity(sizeX, sizeX);
//    ekf_.P_ = (I - K * ekf_.H_) * ekf_.P_;

    ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, ekf_.H_, R_laser_, ekf_.Q_);
    ekf_.Update(z_);
  }

  // KF Prediction step
//  ekf_.x_ = ekf_.F_ * x_prime;// + u;
//  ekf_.P_ = ekf_.F_ * P_prime * ekf_.F_.transpose() + ekf_.Q_;

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;

  return true;
}

#pragma clang diagnostic pop