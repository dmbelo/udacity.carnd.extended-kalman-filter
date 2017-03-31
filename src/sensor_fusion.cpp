#include "sensor_fusion.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using namespace Eigen;

/*
 * Constructor.
 */
SensorFusion::SensorFusion() {
  is_initialized_ = false;

  previous_timestamp_ = 0;
  dt = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // process noise characteristics
  S_ax = 0.0;
  S_ay = 0.0;

}

/**
* Destructor.
*/
SensorFusion::~SensorFusion() {}

void SensorFusion::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    kf_.x_ = VectorXd(4);
    kf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  /*****************************************************************************
  *  Prediction
  ****************************************************************************/

  /*
  Update the state transition matrix F according to the new elapsed time.
  - Time is measured in seconds.
  Update the process noise covariance matrix.
  */

  dt = (measurement_pack.timestamp_ - previous_timestamp_); // Assumed seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  kf_.F_(0, 2) = dt;
  kf_.F_(1, 3) = dt;
  kf_.Q_(0, 0) = pow(dt, 4) * S_ax / 4;
  kf_.Q_(0, 2) = pow(dt, 3) * S_ax / 2;
  kf_.Q_(1, 1) = pow(dt, 4) * S_ay;
  kf_.Q_(1, 3) = pow(dt, 3) * S_ay / 2;
  kf_.Q_(2, 0) = pow(dt, 3) * S_ax / 2;
  kf_.Q_(2, 2) = pow(dt, 2) * S_ax;
  kf_.Q_(3, 1) = pow(dt, 3) * S_ay / 2;
  kf_.Q_(3, 3) = pow(dt, 2) * S_ay;

  kf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /*
  Use the sensor type to perform the update step.
  Update the state and covariance matrices.
  */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    kf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    kf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << kf_.x_ << endl;
  cout << "P_ = " << kf_.P_ << endl;
}