#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

Eigen::MatrixXd CalculateJacobian_(const Eigen::VectorXd &x_state);
Eigen::VectorXd CartesianToPolar_(const Eigen::VectorXd &v);
Eigen::VectorXd PolarToCartesian(const Eigen::VectorXd &v);

class Tools
{
public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                const std::vector<Eigen::VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state);
};

#endif // TOOLS_H_
