#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include "utils.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "car_state.h"
#include "generate_path.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;
  double reference_velocity_mps = mph2mps(49.5);  // 49.5 mph in m/s

  int lane = 1;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }


  h.onMessage([lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                  &map_waypoints_dx,
                  &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                     size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw_rad"];
          double car_speed = j[1]["speed"];
          CarState sdc_state(car_x, car_y, car_s, car_d, car_yaw, car_speed);

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          Path prev_path_map = PointsFromVectors(previous_path_x,
                                                 previous_path_y);
          Path prev_path_car = MapPathToCarPath(sdc_state,
                                                prev_path_map);

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          std::vector<CarState> others;
          auto sensor_fusion = j[1]["sensor_fusion"];
          for (auto other : sensor_fusion) {
            others.emplace_back(other[0], other[1], other[2], other[3],
                                other[4], other[5], other[6]);
          }

          json msgJson;

          // Generate a reference path for the current lane in map co-ordinates.
          Path reference_path_map = GenerateReferencePath(prev_path_map,
                                                          sdc_state,
                                                          1,
                                                          map_waypoints_s,
                                                          map_waypoints_x,
                                                          map_waypoints_y);

          Path reference_path_car = MapPathToCarPath(sdc_state, reference_path_map);
          tk::spline s;
          pair<vector<double>, vector<double>> ref= VectorsFromPath(
              reference_path_car);
          s.set_points(ref.first, ref.second);


          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double dist_inc = 0.5;
          for (int ii = 1; ii < 50; ii++) {
            Point point = getXY(car_s + ii * dist_inc, car_d, map_waypoints_s,
                                map_waypoints_x, map_waypoints_y);

            next_x_vals.push_back(point.x);
            next_y_vals.push_back(point.y);
          }

          // TODO: define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
