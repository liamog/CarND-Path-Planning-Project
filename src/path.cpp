//
// Created by Liam O'Gorman on 6/2/18.
//
#include "path.h"

#include "car_state.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "path.h"
#include "spline.h"

using namespace std;

void DumpPath(const char *name, const Path &path) {
  cout << name << endl;
  for (const Point &point : path) {
    cout << "x=" << point.x << ";y=" << point.y << "|";
  }
  cout << endl;
}

vector<Point> MapPathToCarPath(const CarState &car,
                               const std::vector<Point> &map_path) {
  vector<Point> car_path;
  car_path.reserve(map_path.size());
  for (const Point &map_point : map_path) {
    car_path.emplace_back(map2car(car.point(), car.yaw_rad(), map_point));
  }
  //  DumpPath("MapPathToCarPath", car_path);
  return car_path;
}

vector<Point> CarPathToMapPath(const CarState &car,
                               const std::vector<Point> &car_path) {
  vector<Point> map_path;
  map_path.reserve(car_path.size());
  for (const Point &car_point : car_path) {
    map_path.emplace_back(car2map(car.point(), car.yaw_rad(), car_point));
  }
  //  DumpPath("CarPathToMapPath", map_path);
  return map_path;
}

std::pair<std::vector<double>, std::vector<double>> VectorsFromPath(
    const Path &path) {
  std::vector<double> x;
  std::vector<double> y;
  x.resize(path.size());
  y.resize(path.size());

  for (int ii = 0; ii < path.size(); ++ii) {
    x[ii] = path[ii].x;
    y[ii] = path[ii].y;
  }
  return std::make_pair(x, y);
};

Path PathFromVectors(const std::vector<double> &x,
                     const std::vector<double> &y) {
  std::vector<Point> points;
  points.reserve(x.size());
  for (int ii = 0; ii < x.size(); ++ii) {
    points.emplace_back(x[ii], y[ii]);
  }
  return points;
}

vector<Point> GenerateReferencePath(const std::vector<Point> &prev_map_path,
                                    const CarState &sdc_state,
                                    const int target_lane,
                                    const MapState &map_state) {
  // Create a reference set of waypoints that includes the current
  // car position.
  Path path;

  if (prev_map_path.size() < 2) {
    // Create an initial reference path.
    path.emplace_back(sdc_state.point());
    double kSpacing = 30.0;
    for (int ii = 1; ii <= 3; ii++) {
      Point point = getXY(sdc_state.s() + (kSpacing * ii),
                          lane_to_frenet_d(target_lane), map_state);
      path.push_back(point);
    }
  } else {
    // Add samples from previous path to the new reference path.
    //    path.emplace_back(car_state.point());

    int steps = prev_map_path.size() / 3;
    for (int ii = steps; ii < prev_map_path.size() - 1; ii += steps) {
      path.emplace_back(prev_map_path[ii]);
    }
    // Make sure the last point in the previous path is on the new reference
    // path to ensure a smooth transition when growing the trajectory.
    const Point &last_point = prev_map_path.back();
    path.emplace_back(last_point);
    vector<double> last_frenet =
        getFrenet(last_point.x, last_point.y, sdc_state.yaw_rad(), map_state);

    // Add a point at the end of the previous trajectory + 10 m.
    Point point =
        getXY(last_frenet[0] + 10.0, lane_to_frenet_d(target_lane), map_state);
    path.push_back(point);
  }
  Path car_ref_path = MapPathToCarPath(sdc_state, path);
  double x = -1.0;
  cout << endl << "X:";
  for (const Point &p : car_ref_path) {
    cout << p.x << ",";
    if (p.x <= x) {
      cout << "WARNING " << p.x << " LT " << x;
    }
  }
  cout << endl;

  //  path.emplace_back(car_state.point());

  // Next create 3 points for the forward path.

  //  DumpPath("GenerateReferencePath", path);
  return path;
}

Path GenerateSDCPathByTimeSamples(const Path &ref_path_map,
                                  const Path &prev_path_map,
                                  const CarState &sdc_state, double accel,
                                  double max_speed, double time_step,
                                  double time_horizon) {
  Path prev_path_car = MapPathToCarPath(sdc_state, prev_path_map);
  //  DumpPath("prev_path_car", prev_path_car);
  Path reference_path_car = MapPathToCarPath(sdc_state, ref_path_map);
  tk::spline spline;
  pair<vector<double>, vector<double>> ref =
      VectorsFromPath(reference_path_car);
  spline.set_points(ref.first, ref.second);

  const int kTimeSamples = time_horizon / time_step;
  const double kMaxDistanceHorizon = time_horizon * max_speed;
  int kNumImmutable = 30;

  double v = std::min(sdc_state.v(), max_speed);
  // Build a path of way points sampled from the spline waypoints (spatial).
  // so they cover the distance we want to travel in the time step.
  Path time_sampled_path;
  double x = 0.0, y = 0.0;
  for (int ii = 0, jj = 0; ii < kTimeSamples; ++ii) {
    if (ii < prev_path_car.size()) {
      time_sampled_path.emplace_back(prev_path_car[ii]);
      v = distance(x, y, prev_path_car[ii].x, prev_path_car[ii].y) / time_step;
      x = prev_path_car[ii].x;
      y = prev_path_car[ii].y;
      continue;
    }

    double target_distance =
        (v * time_step) + (0.5 * accel * time_step * time_step);
    // First sample the spline with x= our target distance.
    // This gives us too large a distance but we can use this to approximate the
    // angle at this point and then use some trig to get the
    // actual x and sample the spline again.
    double approx_y = spline(x + target_distance);
    double theta = atan((approx_y - y) / (target_distance - x));
    double incremental_x = (target_distance * cos(theta));
    x = x + incremental_x;
    y = spline(x);

    time_sampled_path.emplace_back(x, y);
    // Update our speed for the next waypoint
    v = std::max(0.0, std::min(v + accel * time_step, max_speed));
  }
  //  DumpPath("time_sampled_path_car", time_sampled_path);
  // Convert the time_sampled_path back to map coords.
  return CarPathToMapPath(sdc_state, time_sampled_path);
}

Path GenerateOtherPathByTimeSamples(const CarState &other_state,
                                    const double time_step,
                                    const double time_horizon,
                                    const MapState &map_state) {
  // Build a simple predicted path assuming no acceleration for the other agent
  // Start with just frenet coorindates and increment the s according to v.
  Path path;
  for (double t = 0; t < time_horizon; t += time_step) {
    Point point = getXY(other_state.s() + (other_state.v() * t),
                        other_state.d(), map_state);
    path.push_back(point);
  }
  return path;
}

double PathCost(const Path &time_path, const std::vector<Path> &others) {
  constexpr double kProximityCostWeight = 100.0;
  constexpr double kDistanceTravelledCostWeight = 1.0;

  // Incoming paths should be time sampled paths of the same size.
  double min_distance = std::numeric_limits<double>::infinity();
  for (int ii = 0; ii < time_path.size(); ii += 2) {
    for (const Path &other_path : others) {
      // Assume that the other car is following it's lane from it's current
      // position with its current speed.
      // Then compare this with our location.
      const double dist = distance(other_path[ii], time_path[ii]);
      // If we get within 10 meters of another vehicle, then take a cost.
      if (dist < 10.0) {
        min_distance = std::min(min_distance, dist);
      }
    }
  }
  const double min_dist_to_car_cost = (1 / min_distance) * kProximityCostWeight;

  const double distance_travelled =
      distance(time_path.front(), time_path.back());
  // Calculate total distance travelled.
  const double distance_travelled_cost = (1.0 / distance_travelled) * kDistanceTravelledCostWeight;

  return min_dist_to_car_cost + distance_travelled_cost;
}

Plan GeneratePathAndCost(const std::string &plan_name,
                         const Path &prev_path_map, const CarState &sdc_state,
                         const std::vector<Path> &others,
                         const MapState &map_state, int target_lane,
                         double accel, double max_speed, double time_step,
                         double time_horizon) {
  // Generate a reference path for the current lane in map co-ordinates.
  Path ref_path_map =
      GenerateReferencePath(prev_path_map, sdc_state, target_lane, map_state);
  Path drivable_path =
      GenerateSDCPathByTimeSamples(ref_path_map, prev_path_map, sdc_state,
                                   accel, max_speed, time_step, time_horizon);

  double path_cost = PathCost(drivable_path, others);
  cout << plan_name << " cost = " << path_cost << endl;
  return std::make_tuple(plan_name, path_cost, drivable_path);
};
