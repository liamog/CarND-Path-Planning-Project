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
#include "map_state.h"
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
  double reference_velocity_mps = mph_to_mps(49.5);  // 49.5 mph in m/s

  int lane = 1;

  // Load map state
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");

  h.onMessage([lane, &map_state](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                 size_t length, uWS::OpCode opCode) {
    cout << endl;
    cout << "-------------------------------------------------------------";
    cout << endl;

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
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          CarState sdc_state(car_x, car_y, car_s, car_d, car_yaw, car_speed);
          cout << sdc_state.ToString() << endl;
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          Path prev_path_map = PathFromVectors(previous_path_x,
                                               previous_path_y);

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          std::vector<CarState> others;
//          auto sensor_fusion = j[1]["sensor_fusion"];
//          for (auto other : sensor_fusion) {
//            others.emplace_back(other[0], other[1], other[2], other[3],
//                                other[4], other[5], other[6]);
//          }

          json msgJson;

          // Generate a reference path for the current lane in map co-ordinates.
          Path ref_path_map = GenerateReferencePath(prev_path_map,
                                                    sdc_state,
                                                    1,
                                                    map_state);
          Path drivable_path = GeneratePathByTimeSamples(ref_path_map,
                                                         prev_path_map,
                                                         sdc_state,
                                                         8.0,
                                                         mph_to_mps(49.5));


          std::pair<vector<double>, vector<double>> next = VectorsFromPath(
              drivable_path);
          // TODO: define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds
          msgJson["next_x"] = next.first;
          msgJson["next_y"] = next.second;

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
