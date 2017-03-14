#include <iostream>
#include "tools.h"

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  // Initialize
  Eigen::VectorXd rmse(4);
  rmse << 0.0, 0.0, 0.0, 0.0;

  /* Check that estimations and groud_truth are not zero and are equal to each
  other in size */
  if(estimations.size() != ground_truth.size() ||
     estimations.size() == 0 ||
     ground_truth.size() == 0){
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  // Accumulate square of residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculate mean
  rmse = rmse / estimations.size();

  // Calculate squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  Eigen::VectorXd rmse(4);
  rmse << 1, 2, 3, 4;
  return rmse;
}
