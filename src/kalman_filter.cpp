#include "kalman_filter.h"
#include <iostream>

using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Initialize(VectorXd &x, MatrixXd &F, MatrixXd &P,
                              MatrixXd &Q) {
  x_ = x;
  F_ = F;
  P_ = P;
  Q_ = Q;
}

void KalmanFilter::Predict() {
  // State estimate prediction
  cout << "x_ = " << endl << x_ << endl;
  cout << "F_ = " << endl << F_ << endl;
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // State estimate update using Kalman Filter equations
  cout << "x_ = " << endl << x_ << endl;
	VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd K = P_ * Ht * S.inverse();

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  cout << "S = " << endl << S << endl;
  cout << "inv(S) = " << endl << S.inverse() << endl;
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}
