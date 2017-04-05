#ifndef SensorFusion_H_
#define SensorFusion_H_

#include "measurement_package.h"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;

class SensorFusion
{
public:
  /**
  * Constructor.
  */
  SensorFusion();

  /**
  * Destructor.
  */
  virtual ~SensorFusion();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter kf_;

private:
  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  double previous_timestamp_;

  // elapsed time
  double dt_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  MatrixXd R_laser_;
  MatrixXd R_radar_;
  MatrixXd H_laser_;

  // Process noise using constant acceleration
  double S_ax; // Variance of longitudinal acceleration
  double S_ay; // Variance of lateral acceleration

};

#endif /* SensorFusion_H_ */
