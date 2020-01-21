#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "FusionEKF.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null == string::npos)
  {
    // if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  vector<VectorXd> RMSEs;
  int count = 0;
  h.onMessage([&fusionEKF, &tools, &estimations, &ground_truth, &RMSEs, &count](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data));
      // auto s = string(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        std::cout << "j = " << j << std::endl;
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          // std::cout << "j size "<<j.size()<< std::endl;
          if (j.size() < 3)
          {
            string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            // json msgJson;
            // msgJson["estimate_x"] = 1;
            // msgJson["estimate_y"] = 1;
            // msgJson["rmse_x"] = 1;
            // msgJson["rmse_y"] = 1;
            // msgJson["rmse_vx"] = 1;
            // msgJson["rmse_vy"] = 1;
            // auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            // ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            VectorXd RMSE = RMSEs[count];
            VectorXd estimate(4);
            estimate = estimations[count];
            double p_x = estimate(0);
            double p_y = estimate(1);
            json msgJson;
            msgJson["estimate_x"] = p_x;
            msgJson["estimate_y"] = p_y;
            msgJson["rmse_x"] = RMSE(0);
            msgJson["rmse_y"] = RMSE(1);
            msgJson["rmse_vx"] = RMSE(2);
            msgJson["rmse_vy"] = RMSE(3);
            auto response = "42[\"estimate_marker\"," + msgJson.dump() + "]";
            std::cout << response << std::endl;
            ws.send(response.data(), response.length(), uWS::OpCode::TEXT);
            count++;
            if (count==RMSEs.size()){
              count=0;
            }
            return;
          }
          string sensor_measurement = j[1]["sensor_measurement"];

          MeasurementPackage meas_package;
          std::istringstream iss(sensor_measurement);

          long long timestamp;

          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;

          if (sensor_type.compare("L") == 0)
          {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
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

          // Call ProcessMeasurement(meas_package) for Kalman filter
          fusionEKF.ProcessMeasurement(meas_package);

          // Push the current estimated x,y positon from the Kalman filter's
          //   state vector

          VectorXd estimate(4);

          double p_x = fusionEKF.ekf_.x_(0);
          double p_y = fusionEKF.ekf_.x_(1);
          double v1 = fusionEKF.ekf_.x_(2);
          double v2 = fusionEKF.ekf_.x_(3);

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
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        } // end "telemetry" if
      }
      else
      {
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  /**
   * Set Measurements
   */
  vector<MeasurementPackage> measurement_pack_list;
  // vector<VectorXd> estimations;
  // vector<VectorXd> ground_truth;
  // Tools tools;

  // hardcoded input file with laser and radar measurements
  string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
  std::ifstream in_file(in_file_name_.c_str(), std::ifstream::in);

  if (!in_file.is_open())
  {
    std::cout << "Cannot open input file: " << in_file_name_ << std::endl;
  }

  string line;
  // set i to get only first 3 measurments
  int i = 0;
  // Create a Tracking instance
  FusionEKF tracking;
  while (getline(in_file, line))
  {

    MeasurementPackage meas_package;

    std::istringstream iss(line);
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
    RMSEs.push_back(RMSE);
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
  std::cout << std::endl
            << "computation complete " << std::endl;

  h.run();
}