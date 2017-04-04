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

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.01, 0,
              0, 0.01;

  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.01, 0, 0,
              0, 0.01, 0,
              0, 0, 0.01;

  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // process noise characteristics
  S_ax = 9;
  S_ay = 9;

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
    MatrixXd P = MatrixXd(4, 4);
    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1e3, 0,
         0, 0, 0, 1e3;
    MatrixXd Q = MatrixXd(4, 4);
    Q << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

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
      float rho = x_measurement(0);
      float phi = x_measurement(1);
      float rho_dot = x_measurement(2);
      float px = rho * cos(phi);
      float py = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      x << px, py, vx, vy;
    }


    kf_.Initialize(x, F, P, Q); // Perform initiliaziation of kf object
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    cout << "x = " << x.transpose() << endl;
    return;
  }

  /*****************************************************************************
  *  Prediction
  ****************************************************************************/
  // Calculate elapsed time
  dt_ = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0; // Assumed seconds
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

  // cout << "kf_.F_ =" << endl << kf_.F_ << endl;
  // cout << "kf_.Q_ =" << endl << kf_.Q_ << endl;

  kf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar H, R matrices
    kf_.H_ = tools.CalculateJacobian(kf_.x_);;
    kf_.R_ = R_radar_;
  } else {
    // Laser H, R matrices
    kf_.H_ = H_laser_;
    kf_.R_ = R_laser_;
  }

  // cout << "kf_.H_ =" << endl << kf_.H_ << endl;
  // cout << "kf_.R_ =" << endl << kf_.R_ << endl;
  // TODO Needs to handle the RADAR case when Hj needs to be implemented...
  /*
  // AA: calc rho_pred, phi_pred, rhodot_pred
  float rho_pred    = sqrt(pow(x_[0], 2) + pow(x_[1], 2));
  float phi_pred    = 0.0;
  if (fabs(x_[0]) > 0.001) {
    phi_pred  = atan2(x_[1], x_[0]);    // arctan(py/px)
  }

  float rhodot_pred = 0.0;
  if (fabs(rho_pred) > 0.001) {
    rhodot_pred = (x_[0]*x_[2] + x_[1]*x_[3]) / rho_pred; // (px * vy + py*vx)/rho_pred
  }

  VectorXd  z_pred(3);
  z_pred    << rho_pred, phi_pred, rhodot_pred;
  */
  kf_.Update(measurement_pack.raw_measurements_);

  // print the output
  cout << "x = " << kf_.x_.transpose() << endl;
  cout << "P_ = " << endl << kf_.P_ << endl;
}
