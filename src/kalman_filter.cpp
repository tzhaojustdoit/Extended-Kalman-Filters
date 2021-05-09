#include "kalman_filter.h"

#include <math.h>

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
/**
 * Predict
 * Predict the state
 */
void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}
/**
 * Update
 * Update the state by using Kalman Filter equations
 */
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd s = H_ * P_ * Ht + R_;
  MatrixXd k = P_ * Ht * s.inverse();

  x_ = x_ + k * y;
  P_ = (I_ - k * H_) * P_;
}
/**
 * UpdateEKF
 * update the state by using Extended Kalman Filter equations
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float x = x_(0), y = x_(1), vx = x_(2), vy = x_(3);
  if (fabs(x) < 0.0001 || fabs(y) < 0.0001) {
    std::cout << "UpdateEKF () - Error - Division by Zero" << std::endl;
    return;
  }
  float rho = sqrt(x * x + y * y);
  float phi = atan2(y, x);
  float rho_dot = (x * vx + y * vy) / rho;
  VectorXd hx = VectorXd(3);
  hx << rho, phi, rho_dot;
  VectorXd Y = z - hx;
  if (Y(1) <= -M_PI) {
    Y(1) += M_PI * 2;
  } else if (Y(1) > M_PI) {
    Y(1) -= M_PI * 2;
  }
  MatrixXd Ht = H_.transpose();
  MatrixXd s = H_ * P_ * Ht + R_;
  MatrixXd k = P_ * Ht * s.inverse();

  x_ = x_ + k * Y;
  P_ = (I_ - k * H_) * P_;
}
