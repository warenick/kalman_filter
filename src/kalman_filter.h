#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter
{
public:
  KalmanFilter();
  virtual ~KalmanFilter();
  void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &Q_in, const int nin);
  void Predict();

  void UpdateF(const double dt);
  void Update(const Eigen::VectorXd &z, const MatrixXd& H, const VectorXd& Hx, const MatrixXd& R);
  void UpdateEKF(const Eigen::VectorXd &z);

  VectorXd x_;
  MatrixXd P_;
  MatrixXd F_;
  MatrixXd Q_;
  int n_;
  Eigen::MatrixXd I_;

};

#endif // KALMAN_FILTER_H_
