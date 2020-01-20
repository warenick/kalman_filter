#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

MatrixXd CalculateJacobian_(const VectorXd &x_state)
{

  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  // check division by zero
  if (fabs(c1) < 0.0001)
  {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
      -(py / c1), (px / c1), 0, 0,
      py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  return Hj;
}
VectorXd CartesianToPolar_(const VectorXd &v)
{

  const double THRESH = 0.0001;
  VectorXd polar_vector(3);

  const double px = v(0);
  const double py = v(1);
  const double vx = v(2);
  const double vy = v(3);

  const double rho = sqrt(px * px + py * py);

  const bool degenerate = (py == 0.0 && px == 0.0);
  const double phi = degenerate ? 0.0 : atan2(py, px);
  // Avoids degenerate case, see error handling in
  // http://en.cppreference.com/w/cpp/numeric/math/atan2

  const double drho = (rho > THRESH) ? (px * vx + py * vy) / rho : 0.0;
  // Above line avoids dividing by zero

  polar_vector << rho, phi, drho;
  return polar_vector;
}


VectorXd PolarToCartesian(const VectorXd& v){

  VectorXd cartesian_vector(4);

  const double rho = v(0);
  const double phi = v(1);
  const double drho = v(2);

  const double px = rho * cos(phi);
  const double py = rho * sin(phi);
  const double vx = drho * cos(phi);
  const double vy = drho * sin(phi);

  cartesian_vector << px, py, vx, vy;
  return cartesian_vector;
}


Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth)
{
   /**
   * TODO: Calculate the RMSE here.
   */
   VectorXd rmse(4);
   rmse << 0, 0, 0, 0;

   // check the validity of the following inputs:
   //  * the estimation vector size should not be zero
   //  * the estimation vector size should equal ground truth vector size
   if (estimations.size() != ground_truth.size() || estimations.size() == 0)
   {
      cout << "Invalid estimation or ground_truth data" << endl;
      return rmse;
   }

   // accumulate squared residuals
   for (unsigned int i = 0; i < estimations.size(); ++i)
   {

      VectorXd residual = estimations[i] - ground_truth[i];

      // coefficient-wise multiplication
      residual = residual.array() * residual.array();
      rmse += residual;
   }

   // calculate the mean
   rmse = rmse / estimations.size();

   // calculate the squared root
   rmse = rmse.array().sqrt();

   // return the result
   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
{
   /**
   * TODO:
   * Calculate a Jacobian here.
   */
   MatrixXd Hj(3, 4);
   // recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   // pre-compute a set of terms to avoid repeated calculation
   float c1 = px * px + py * py;
   float c2 = sqrt(c1);
   float c3 = (c1 * c2);

   // check division by zero
   if (fabs(c1) < 0.0001)
   {
      cout << "CalculateJacobian () - Error - Division by Zero" << endl;
      return Hj;
   }

   // compute the Jacobian matrix
   Hj << (px / c2), (py / c2), 0, 0,
       -(py / c1), (px / c1), 0, 0,
       py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

   return Hj;
}
