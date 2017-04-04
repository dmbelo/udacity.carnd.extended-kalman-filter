#include "sensor_fusion.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using namespace Eigen;

/*
 * Constructor.
 */
SensorFusion::SensorFusion() {
  is_initialized_ = false;

  previous_timestamp_ = 0;
  dt_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // process noise characteristics
  S_ax = 0.0;
  S_ay = 0.0;

}

/*
 * Destructor.
 */
SensorFusion::~SensorFusion() {}

void SensorFusion::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    VectorXd x_measurement, x;
    x = VectorXd(4);
    MatrixXd F = MatrixXd(4, 4);
    F << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    // Decide how to initialize depending on what type of measurement we have
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x_measurement = VectorXd(2);
      x_measurement << measurement_pack.raw_measurements_;
      float px = x_measurement(0);
      float py = x_measurement(1);
      x << px, py, 0, 0; // Assuming starting at 0 velocity
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      x_measurement = VectorXd(3);
      x_measurement << measurement_pack.raw_measurements_;
      float rho = x(0);
      float phi = x(1);
      float rho_dot = x(2);
      float px = rho * sin(phi);
      float py = rho * cos(phi);
      float vx = rho_dot * sin(phi);
      float vy = rho_dot * cos(phi);
      x << px, py, vx, vy;
    }

    kf_.Initialize(x, F); // Perform initiliaziation of kf object
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
  *  Prediction
  ****************************************************************************/
  // Calculate elapsed time
  dt_ = (measurement_pack.timestamp_ - previous_timestamp_); // Assumed seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update the F, Q matrices given elapsed time
  kf_.F_(0, 2) = dt_;
  kf_.F_(1, 3) = dt_;
  kf_.Q_(0, 0) = pow(dt_, 4) * S_ax / 4;
  kf_.Q_(0, 2) = pow(dt_, 3) * S_ax / 2;
  kf_.Q_(1, 1) = pow(dt_, 4) * S_ay;
  kf_.Q_(1, 3) = pow(dt_, 3) * S_ay / 2;
  kf_.Q_(2, 0) = pow(dt_, 3) * S_ax / 2;
  kf_.Q_(2, 2) = pow(dt_, 2) * S_ax;
  kf_.Q_(3, 1) = pow(dt_, 3) * S_ay / 2;
  kf_.Q_(3, 3) = pow(dt_, 2) * S_ay;

  kf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar H, R matrices
    kf_.H_ = H_laser_;
    kf_.R_ = R_laser_;
  } else {
    // Laser H, R matrices
    kf_.H_ = Hj_;
    kf_.R_ = R_radar_;
  }

  kf_.Update(measurement_pack.raw_measurements_);

  // print the output
  cout << "x_ = " << kf_.x_ << endl;
  cout << "P_ = " << kf_.P_ << endl;
}
