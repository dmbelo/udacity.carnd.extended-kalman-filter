#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {

  // Initializing the Kalman Filter object
  // kf_.x_ = VectorXd(4);
  // kf_.F_ = MatrixXd(4, 4);
  // kf_.F_ << 1, 0, 0, 0,
  //           0, 1, 0, 0,
  //           0, 0, 1, 0,
  //           0, 0, 0, 1;
  // kf_.P_ = MatrixXd(4, 4);
  // kf_.P_ << 1e4, 1e4, 1e4, 1e4,
  //           1e4, 1e4, 1e4, 1e4,
  //           1e4, 1e4, 1e4, 1e4,
  //           1e4, 1e4, 1e4, 1e4;
  // kf_.Q_ = MatrixXd(4, 4);
  // kf_.Q_ << 0, 0, 0, 0,
  //           0, 0, 0, 0,
  //           0, 0, 0, 0,
  //           0, 0, 0, 0;

}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Initialize(VectorXd &x, MatrixXd &F) {
  int n_states = F_.size();

  x_ = VectorXd(n_states);
  F_ = MatrixXd(n_states, n_states);
  P_ = MatrixXd(n_states, n_states);
  Q_ = MatrixXd(n_states, n_states);

  // x_ << 0, 0, 0, 0;
  x_ = x;
  F_ = F;
  P_ << 1e4, 1e4, 1e4, 1e4,
        1e4, 1e4, 1e4, 1e4,
        1e4, 1e4, 1e4, 1e4,
        1e4, 1e4, 1e4, 1e4;
  Q_ << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
}

void KalmanFilter::Predict() {
  // State estimate prediction
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // State estimate update using Kalman Filter equations
	VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd K = P_ * Ht * S.inverse();

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}
