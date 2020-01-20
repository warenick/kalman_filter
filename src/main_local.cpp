#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
// #include "tracking.h"
#include "FusionEKF.h"
#include "json.hpp"
#include <uWS/uWS.h>



using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;
using json = nlohmann::json;

int main()
{

  /**
   * Set Measurements
   */
  vector<MeasurementPackage> measurement_pack_list;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  Tools tools;

  // hardcoded input file with laser and radar measurements
  string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
  ifstream in_file(in_file_name_.c_str(), ifstream::in);

  if (!in_file.is_open())
  {
    cout << "Cannot open input file: " << in_file_name_ << endl;
  }

  string line;
  // set i to get only first 3 measurments
  int i = 0;
  // Create a Tracking instance
  FusionEKF tracking;
  while (getline(in_file, line))
  {

    MeasurementPackage meas_package;

    istringstream iss(line);
    string sensor_type;
    iss >> sensor_type; // reads first element from the current line
    int64_t timestamp;
    if (sensor_type.compare("L") == 0)
    { // laser measurement
      // read measurements
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x, y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }
    else if (sensor_type.compare("R") == 0)
    {
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float theta;
      float ro_dot;
      iss >> ro;
      iss >> theta;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, theta, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    VectorXd gt_values(4);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt;
    gt_values(2) = vx_gt;
    gt_values(3) = vy_gt;
    ground_truth.push_back(gt_values);

    tracking.ProcessMeasurement(measurement_pack_list[i]);
    VectorXd estimate(4);

    double p_x = tracking.ekf_.x_(0);
    double p_y = tracking.ekf_.x_(1);
    double v1 = tracking.ekf_.x_(2);
    double v2 = tracking.ekf_.x_(3);

    estimate(0) = p_x;
    estimate(1) = p_y;
    estimate(2) = v1;
    estimate(3) = v2;

    estimations.push_back(estimate);

    VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

    json msgJson;
    msgJson["estimate_x"] = p_x;
    msgJson["estimate_y"] = p_y;
    msgJson["rmse_x"] = RMSE(0);
    msgJson["rmse_y"] = RMSE(1);
    msgJson["rmse_vx"] = RMSE(2);
    msgJson["rmse_vy"] = RMSE(3);
    auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
    std::cout << msg << std::endl;
    ++i;
  }
  if (in_file.is_open())
  {
    in_file.close();
  }
  return 0;
}