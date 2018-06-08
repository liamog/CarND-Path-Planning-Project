#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>

#include "utils.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "car_state.h"
#include "map_state.h"
#include "path.h"

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
  double reference_velocity_mps = mph_to_mps(45);  // 49.5 mph in m/s

  // Load map state
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");

  ofstream car_state_stream("car_state.csv");
  car_state_stream << CarState::CsvStringHeader() << endl;

  //  ofstream refpath_stream("ref_path.csv");
  //
  int count = 0;
  h.onMessage([&map_state, &car_state_stream, &count](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
    constexpr double kTimeStep = 1.0 / 50;
    constexpr double kTimeHorizon = 2.0;
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        cout << "-----------------------------------iteration " << count++
             << endl;
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
          CarState sdc_state(car_x, car_y, car_s, car_d, car_speed, car_yaw);

          car_state_stream << sdc_state.ToCsvString() << endl;
          car_state_stream.flush();

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          Path prev_path_map =
              PathFromVectors(previous_path_x, previous_path_y);
          DumpPath("prev_path_map", prev_path_map);

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          std::tuple<double, double> end_path_s_d(end_path_s, end_path_d);
          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          std::vector<Path> others;
          auto sensor_fusion = j[1]["sensor_fusion"];
          for (auto other : sensor_fusion) {
            //[ id, x, y, vx, vy, s, d]
            const auto id = other[0];
            const double x = other[1];
            const double y = other[2];
            const double vx = other[3];
            const double vy = other[4];
            const double s = other[5];
            const double d = other[6];

            const double v = sqrt((vx * vx) + (vy * vy));
            CarState state(x, y, s, d, v);
            others.emplace_back(GenerateOtherPathByTimeSamples(
                state, kTimeStep, kTimeHorizon, map_state));
          }

          json msgJson;

          double kMaxAccel = 6.0;
          double kNoAccel = 0.0;
          double kMinAccel = -3.0;
          double kMaxSpeed = mph_to_mps(45);

          // Using lambda to compare elements.
          auto cmp = [](Plan left, Plan right) {
            return get<1>(left) > get<1>(right);
          };

          std::priority_queue<Plan, std::deque<Plan>, decltype(cmp)> plans(cmp);

          vector<int> lanes = sdc_state.GetPossibleLanes();

          for (int lane : lanes) {
              // Accel
              plans.push(GeneratePathAndCost(
                      "Accel:1", prev_path_map, sdc_state, others, map_state,
                      end_path_s_d, lane, kMaxAccel, kMaxSpeed, kTimeStep,
                      kTimeHorizon));
          }
          // Maintain speed
          plans.push(GeneratePathAndCost(
              "Maintain:1", prev_path_map, sdc_state, others, map_state,
              end_path_s_d, sdc_state.Lane(), kNoAccel, kMaxSpeed, kTimeStep,
              kTimeHorizon));

          // Brake
          plans.push(GeneratePathAndCost(
              "brake:1", prev_path_map, sdc_state, others, map_state,
              end_path_s_d, sdc_state.Lane(), kMinAccel, kMaxSpeed, kTimeStep,
              kTimeHorizon));

          cout << "Selected " << get<0>(plans.top());
          std::pair<vector<double>, vector<double>> next =
              VectorsFromPath(get<2>(plans.top()));
          // TODO: define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds
          msgJson["next_x"] = next.first;
          msgJson["next_y"] = next.second;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
        cout << endl;
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
