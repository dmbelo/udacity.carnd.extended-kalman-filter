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
  R_laser_ << 1, 0,
              0, 1;

  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 1e7, 0, 0,
              0, 1e7, 0,
              0, 0, 1e7;

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
    VectorXd x;
    x = VectorXd(4);
    MatrixXd F = MatrixXd(4, 4);
    // F << 1, 0, 0, 0,
    //      0, 1, 0, 0,
    //      0, 0, 1, 0,
    //      0, 0, 0, 1;
    MatrixXd P = MatrixXd(4, 4);
    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1e3, 0,
         0, 0, 0, 1e3;
    MatrixXd Q = MatrixXd(4, 4);
    // Q << 0, 0, 0, 0,
    //      0, 0, 0, 0,
    //      0, 0, 0, 0,
    //      0, 0, 0, 0;

    // Decide how to initialize depending on what type of measurement we have
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x << measurement_pack.raw_measurements_, 0, 0; // Assuming starting at 0 velocity
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = measurement_pack.raw_measurements_(0);
      double phi = measurement_pack.raw_measurements_(1);
      double rho_dot = measurement_pack.raw_measurements_(2);
      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double vx = 0;
      double vy = 0;
      // double vx = rho_dot * cos(phi);
      // double vy = rho_dot * sin(phi);
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
  // kf_.F_(0, 2) = dt_;
  // kf_.F_(1, 3) = dt_;
  kf_.F_ << 1, 0, dt_, 0,
            0, 1, 0, dt_,
            0, 0, 1, 0,
            0, 0, 0, 1;
  // kf_.Q_(0, 0) = pow(dt_, 4) * S_ax / 4;
  // kf_.Q_(0, 2) = pow(dt_, 3) * S_ax / 2;
  // kf_.Q_(1, 1) = pow(dt_, 4) * S_ay;
  // kf_.Q_(1, 3) = pow(dt_, 3) * S_ay / 2;
  // kf_.Q_(2, 0) = pow(dt_, 3) * S_ax / 2;
  // kf_.Q_(2, 2) = pow(dt_, 2) * S_ax;
  // kf_.Q_(3, 1) = pow(dt_, 3) * S_ay / 2;
  // kf_.Q_(3, 3) = pow(dt_, 2) * S_ay;
  double dt2 = dt_ * dt_;
  double dt3 = dt2 * dt_;
  double dt4 = dt3 * dt_;
  kf_.Q_ << dt4 * S_ax / 4, 0, dt3 * S_ax / 2, 0,
            0, dt4 * S_ay / 4, 0, dt3 * S_ay / 2,
            dt3 * S_ax / 2, 0, dt2 * S_ax, 0,
            0, dt3 * S_ay / 2, 0, dt2 * S_ay;

  // cout << "kf_.F_ =" << endl << kf_.F_ << endl;
  // cout << "kf_.Q_ =" << endl << kf_.Q_ << endl;

  kf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  VectorXd y(3);
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar H, R matrices
    double rho_pred, phi_pred, rhodot_pred;

    // sqrt(px^2 + py^2)
    rho_pred = sqrt(pow(kf_.x_[0], 2) + pow(kf_.x_[1], 2));

    // arctan(py/px)
    phi_pred = 0.0;
    if (fabs(kf_.x_[0]) > 0.001) {
      phi_pred  = atan2(kf_.x_[1], kf_.x_[0]);
    }

    // (px*vx + py*vy)/rho_pred
    rhodot_pred = 0.0;
    if (fabs(rho_pred) > 0.001) {
      rhodot_pred = (kf_.x_[0] * kf_.x_[2] + kf_.x_[1] * kf_.x_[3]) / rho_pred;
    }

    kf_.H_ = tools.CalculateJacobian(kf_.x_);;
    kf_.R_ = R_radar_;
    VectorXd z_pred(3);
    z_pred << rho_pred, phi_pred, rhodot_pred;
    cout << "zpred = " << z_pred.transpose() << endl;
    y = measurement_pack.raw_measurements_ - z_pred;
    kf_.Update(y);

  }
  else {

    // Laser H, R matrices
    kf_.H_ = H_laser_;
    kf_.R_ = R_laser_;
    y = measurement_pack.raw_measurements_ - kf_.H_ * kf_.x_;
    kf_.Update(y);

  }

  // print the output
  cout << endl;
  cout << "m = " << measurement_pack.raw_measurements_.transpose();
  cout << endl;
  cout << "P = " << kf_.P_;
  cout << endl;
  cout << "x = " << kf_.x_.transpose();
  cout << endl;
  // cout << "P_ = " << endl << kf_.P_ << endl;
}
