#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

FusionEKF::FusionEKF()
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  P_ = MatrixXd(4, 4);
  F_ = MatrixXd::Identity(4, 4);
  Q_ = MatrixXd::Zero(4, 4);

  R_laser_ << 0.0225, 0,
      0, 0.0225;

  R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  P_ << 1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1000.0, 0.0,
      0.0, 0.0, 0.0, 1000;

  H_laser_ << 1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::UpdateQ(const double dt)
{

  const double dt2 = dt * dt;
  const double dt3 = dt * dt2;
  const double dt4 = dt * dt3;

  const double r11 = dt4 * ax_ / 4;
  const double r13 = dt3 * ax_ / 2;
  const double r22 = dt4 * ay_ / 4;
  const double r24 = dt3 * ay_ / 2;
  const double r31 = dt3 * ax_ / 2;
  const double r33 = dt2 * ax_;
  const double r42 = dt3 * ay_ / 2;
  const double r44 = dt2 * ay_;

  Q_ << r11, 0.0, r13, 0.0,
      0.0, r22, 0.0, r24,
      r31, 0.0, r33, 0.0,
      0.0, r42, 0.0, r44;

  ekf_.Q_ = Q_;
}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  VectorXd state(4);
  if (measurement_pack.sensor_type_== MeasurementPackage::RADAR){
    state = PolarToCartesian(measurement_pack.raw_measurements_);
  }
  else{
    double x = measurement_pack.raw_measurements_(0);
    double y = measurement_pack.raw_measurements_(1);
    state << x,y,0,0;
  }
  /**************************************************************************
   * INITIALISATION STEP
   **************************************************************************/
  if (!is_initialized_)
  {
    previous_timestamp_ = measurement_pack.timestamp_; //measurement_pack.get_timestamp();
    VectorXd xi = state; // state
    ekf_.Init(xi, P_, F_, Q_, 4);
    is_initialized_ = true;
    cout << "ekf init" << endl;
    return;
  }
  /**************************************************************************
   * PREDICTION STEP
   **************************************************************************/
  const double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1.e6;
  previous_timestamp_ = measurement_pack.timestamp_;
  UpdateQ(dt);
  ekf_.UpdateF(dt);
  ekf_.Predict();

  /**************************************************************************
   * UPDATE STEP
   **************************************************************************/
  const VectorXd z = measurement_pack.raw_measurements_; // measurement from sensor
  const VectorXd x = ekf_.x_;                            // predicted state

  VectorXd Hx;
  MatrixXd R;
  MatrixXd H;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    VectorXd s = state; // state
    H = CalculateJacobian_(s);
    Hx = CartesianToPolar_(x);
    R = R_radar_;
  }
  else
  {
    H = H_laser_;
    Hx = H_laser_ * x;
    R = R_laser_;
  }
  ekf_.Update(z, H, Hx, R);

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
