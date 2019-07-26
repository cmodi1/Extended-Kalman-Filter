#include "kalman_filter.h"
#include "tools.h"
#include <math.h>
#include <iostream>
#define _USE_MATH_DEFINES

using Eigen::MatrixXd;
using Eigen::VectorXd;
Tools tools;
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  std::cout<<"Predict Step\n";
  x_ = F_ * x_;
  P_ = F_*P_* F_.transpose() + Q_;
  std::cout << "x_ =\n" << x_ << std::endl;
  std::cout << "P_ =\n" << P_ << std::endl << std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  std::cout<<"Laser Update Step\n";
  VectorXd y = z - H_*x_;
  MatrixXd S = H_*P_*H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + (K*y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K*H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  std::cout<<"Radar Update Step\n";
  MatrixXd Hj = tools.CalculateJacobian(x_);
  MatrixXd hx(3,1);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rho = sqrt(px*px + py*py);
  float phi = atan2(py,px);
  float rho_dot = (px*vx + py*vy)/rho;
  hx << rho,
        phi,
        rho_dot;
  
  VectorXd y = z - hx;
  while(y[1] > M_PI)
    y[1] -= (2*M_PI);
  while(y[1] < -M_PI)
    y[1] += (2*M_PI);
  
  MatrixXd S = Hj*P_*Hj.transpose() + R_;
  MatrixXd K = P_ * Hj.transpose() * S.inverse();
  x_ = x_ + (K*y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K*Hj) * P_;
}
