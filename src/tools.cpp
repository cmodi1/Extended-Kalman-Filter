#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if (estimations.size() != ground_truth.size() || estimations.size() == 0)
  {
    std::cout << "Invalid data" << std::endl;
    return rmse;
  }
  
  for(unsigned int i=0; i<estimations.size(); i++)
  {
    VectorXd error = estimations[i] - ground_truth[i];
    error = error.array() * error.array(); 
    rmse += error;
  }
  rmse =  rmse/estimations.size();
  return rmse.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj = MatrixXd(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  if((px == 0) || (py == 0) || (px*px + py*py == 0))
  {
    std::cout << "Division by Zero!\n";
    Hj << 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;
    throw std::runtime_error("Division By Zero!");
  }
  float h00 = px/sqrt(px*px + py*py);
  float h01 = py/sqrt(px*px + py*py);
  float h10 = -py/(px*px + py*py);
  float h11 = px/(px*px + py*py);
  float h20 = py*(vx*py - vy*px)/pow((px*px + py*py), 1.5);
  float h21 = px*(vy*px - vx*py)/pow((px*px + py*py), 1.5);
  Hj << h00, h01, 0, 0,
        h10, h11, 0, 0,
        h20, h21, h00, h01;
  return Hj;
}
