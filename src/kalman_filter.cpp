#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &Q_in, const int nin)
{
  // std::cout << "x_in = "<<x_in<<std::endl;
  n_ = nin;
  I_ = MatrixXd::Identity(n_, n_);
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
}
void KalmanFilter::UpdateF(const double dt)
{
  F_(0, 2) = dt;
  F_(1, 3) = dt;
}
void KalmanFilter::Predict()
{
  // std::cout << "x_ = " <<std::endl<<x_<<std::endl;
  // std::cout << "F_ = " <<std::endl<<F_<<std::endl;
  // std::cout << "P_ = " <<std::endl<<P_<<std::endl;
  // std::cout << "F_.transpose = " <<std::endl<<F_.transpose()<<std::endl;
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &H, const VectorXd &Hx, const MatrixXd &R)
{

  /**
   * TODO: update the state by using Kalman Filter equations
   */
  const MatrixXd PHt = P_ * H.transpose();
  const MatrixXd S = H * PHt + R;
  const MatrixXd K = PHt * S.inverse();
  VectorXd y = z - Hx;

  // Assume this is radar measurement
  // y(1) is an angle (phi), it shoulde be normalized
  // refer to the comment at the bottom of this file
  if (y.size() == 3)
    y(1) = atan2(sin(y(1)), cos(y(1)));

  x_ = x_ + K * y;
  P_ = (I_ - K * H) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
