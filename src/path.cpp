//
// Created by Liam O'Gorman on 6/2/18.
//
#include "path.h"

#include "car_state.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <tuple>
#include <vector>

#include "path.h"
#include "spline.h"
#include "utils.h"

using namespace std;

void DumpPath(const char *name, const Path &path) {
  cout << name << endl;
  for (const Point &point : path) {
    cout << "x=" << point.x << ";y=" << point.y << "|";
  }
  cout << endl;
}

Path MapPathToCarPath(const CarState &car, const Path &map_path) {
  Path car_path;
  car_path.reserve(map_path.size());
  for (const Point &map_point : map_path) {
    car_path.emplace_back(Map2Car(car.point(), car.yaw_rad(), map_point));
  }
  //  DumpPath("MapPathToCarPath", car_path);
  return car_path;
}

Path CarPathToMapPath(const MapState &map_state, const CarState &car,
                      const Path &car_path) {
  Path map_path;
  map_path.reserve(car_path.size());
  for (const Point &car_point : car_path) {
    map_path.emplace_back(Car2Map(car.point(), car.yaw_rad(), car_point));
  }
  // Update the frenet values.
  UpdateFrenet(map_state, &map_path);
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
  Path points;
  points.reserve(x.size());
  for (int ii = 0; ii < x.size(); ++ii) {
    double theta = (ii < x.size() - 1)
                       ? points[ii - 1].theta
                       : atan2(y[ii + 1] - y[ii], x[ii + 1] - x[ii]);
    points.emplace_back(x[ii], y[ii], theta);
  }
  return points;
}

Path GenerateReferencePath(const Path &prev_map_path, const CarState &sdc_state,
                           std::tuple<double, double> end_path_s_d,
                           const int target_lane, const MapState &map_state) {
  // Create a reference set of waypoints that includes the current
  // car position.
  Path path;

  if (prev_map_path.size() < 2) {
    // Create an initial reference path.
    path.emplace_back(sdc_state.point());
    double kSpacing = 30.0;
    for (int ii = 1; ii <= 3; ii++) {
      double s = sdc_state.s() + (kSpacing * ii);
      double d = lane_to_frenet_d(target_lane);
      Point point = GetXY(s, d, map_state);
      path.push_back(point);
    }
  } else {
    Path prev_car_path = MapPathToCarPath(sdc_state, prev_map_path);
    // DumpPath("prev_car_path", prev_car_path);
    // Add samples from previous path to the new reference path.
    // However the last path might be very short if the speed is low.
    // So make sure that we get to a minimum length
    double path_length = 0.0;
    constexpr double kMinRefPathLength = 100.0;

    double s, d;
    std::tie(s, d) = end_path_s_d;
    double path_s_length = s - sdc_state.s();

    // Once we reach a certain length path from the previous trajectory, we
    // add in a few of these points to the spline.
    if (path_s_length > 20.0) {
      int steps = prev_map_path.size() / 3;
      for (int ii = steps; ii < prev_map_path.size() - 1; ii += steps) {
        path.emplace_back(prev_map_path[ii]);
      }
    }
    // Make sure the last point in the previous path is on the new reference
    // path to ensure a smooth transition when growing the trajectory.
    const Point &last_point = prev_map_path.back();
    path.emplace_back(last_point);
    double kSpacing = 30.0;

    for (path_s_length += kSpacing; path_s_length < kMinRefPathLength;
         path_s_length += kSpacing) {
      double new_s = sdc_state.s() + path_s_length;
      double new_d = lane_to_frenet_d(target_lane);
      Point point = GetXY(new_s, new_d, map_state);

      path.push_back(point);
    }
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

Path GenerateSDCPathByTimeSamples(const MapState &map_state,
                                  const Path &ref_path_map,
                                  const Path &prev_path_map,
                                  const CarState &sdc_state, double accel,
                                  double max_speed, double time_step,
                                  double time_horizon) {
  Path prev_path_car = MapPathToCarPath(sdc_state, prev_path_map);
  //  DumpPath("prev_path_car", prev_path_car);
  Path ref_path_car = MapPathToCarPath(sdc_state, ref_path_map);
  // DumpPath("ref_path_car", ref_path_car);
  tk::spline spline;
  pair<vector<double>, vector<double>> ref = VectorsFromPath(ref_path_car);
  spline.set_points(ref.first, ref.second);

  const int kTimeSamples = time_horizon / time_step;
  const double kMaxDistanceHorizon = time_horizon * max_speed;
  int kNumImmutable = 30;

  double v = std::min(sdc_state.v(), max_speed);
  // Build a path of way points sampled from the spline waypoints (spatial).
  // so they cover the Distance we want to travel in the time step.
  Path time_sampled_path;
  double x = 0.0, y = 0.0;
  for (int ii = 0, jj = 0; ii < kTimeSamples; ++ii) {
    if (ii < prev_path_car.size()) {
      time_sampled_path.emplace_back(prev_path_car[ii]);
      v = Distance(x, y, prev_path_car[ii].x, prev_path_car[ii].y) / time_step;
      x = prev_path_car[ii].x;
      y = prev_path_car[ii].y;
      continue;
    }
    double theta = 0.0;

    double target_distance =
        (v * time_step) + (0.5 * accel * time_step * time_step);
    if (target_distance > 0.00001) {
      // First sample the spline with x= our target Distance.
      // This gives us too large a Distance but we can use this to approximate
      // the angle at this point and then use some trig to get the actual x and
      // sample the spline again.
      double approx_y = spline(x + target_distance);
      theta = atan2((approx_y - y), target_distance);
      double incremental_x = (target_distance * cos(theta));
      x = x + incremental_x;
    }
    y = spline(x);

    time_sampled_path.emplace_back(x, y, theta);
    // Update our speed for the next waypoint
    v = std::max(0.0, std::min(v + accel * time_step, max_speed));
  }
  for (int ii = 0; ii < time_sampled_path.size() - 1; ++ii) {
    assert(time_sampled_path[ii].x <= time_sampled_path[ii + 1].x);
  }
  //  DumpPath("time_sampled_path_car", time_sampled_path);
  // Convert the time_sampled_path back to map coords.
  return CarPathToMapPath(map_state, sdc_state, time_sampled_path);
}

Path GenerateOtherPathByTimeSamples(const CarState &other_state,
                                    const double time_step,
                                    const double time_horizon,
                                    const MapState &map_state) {
  // Build a simple predicted path assuming no acceleration for the other agent
  // Start with just frenet coorindates and increment the s according to v.
  Path path;
  for (double t = 0; t < time_horizon; t += time_step) {
    Point point = GetXY(other_state.s() + (other_state.v() * t),
                        other_state.d(), map_state);
    path.push_back(point);
  }
  return path;
}

double PathCost(const CarState &sdc_state, const Path &sdc_path,
                const std::vector<Path> &others, const double time_step) {
  // Adjust these weights and maxes so the cost function will choose
  // the most optimal path for progress and safety.
  constexpr double kProximityCostWeight = 50.0;
  constexpr double kMaxProximityFactor = 500.0;

  constexpr double kCollisionCostWeight = 100.0;
  constexpr double kMaxCollisionFactor = 1000.0;

  constexpr double kDistanceTravelledCostWeight = 1.0;
  constexpr double kMaxDistanceTravelledFactor = 100.0;

  constexpr double kMinDistanceTimeWeight = 10.0;
  constexpr double kMaxDistanceTimeFactor = 10.0;

  // NOTE: Incoming paths should be time sampled paths of the same size.

  //  This cost is relative to the cartesian Distance between us and them over
  //  time. We only incur this if we get within 10m.
  double min_distance = std::numeric_limits<double>::infinity();
  double min_distance_time = 0;
  for (int ii = 0; ii < sdc_path.size(); ii += 1) {
    for (const Path &other_path : others) {
      // Assume that the other car is following it's lane from it's current
      // position with its current speed.
      // Then compare this with our location.
      const double dist = Distance(other_path[ii], sdc_path[ii]);
      // If we get within 10 meters of another vehicle, then take a cost.
      if (dist < 10.0 && dist < min_distance) {
        min_distance_time = ii * time_step;
        min_distance = dist;
      }
    }
  }

  // This value is the distance to car ahead in lane, it only captures the
  // min distance if the states are in the same lane, so it will capture
  // a lane change behind another vehicle.
  double in_lane_proximity = std::numeric_limits<double>::infinity();
  double in_lane_min_proximity_time = 0;
  for (int ii = 0; ii < sdc_path.size(); ii += 1) {
    for (const Path &other_path : others) {
      if (frenet_d_to_lane(other_path[ii].d) !=
          frenet_d_to_lane(sdc_path[ii].d)) {
        continue;
      }
      const double dist = other_path[ii].s - sdc_path[ii].s;
      // If the other car is behind us don't count it.
      if (dist < 0.0) continue;

      // If we get within 10 meters of another vehicle, then take a cost.
      if (dist < 10.0 && dist < min_distance) {
        in_lane_min_proximity_time = ii * time_step;
        in_lane_proximity = dist;
      }
    }
  }

  if (min_distance < 1.0) {
    // We have a collision.

  }

  // Calculate total Distance travelled.
  const double distance_travelled = sdc_path.back().s - sdc_path.front().s;

  // Have a cost for changing lanes.

  const double min_distance_factor =
      std::min(kMaxProximityFactor, 1.0 / min_distance);
  const double min_distance_time_factor =
      std::min(kMaxDistanceTimeFactor, 1.0 / min_distance_time);
  const double distance_travelled_factor =
      std::min(kMaxDistanceTravelledFactor, 1.0 / distance_travelled);

  const double min_dist_to_car_cost =
      min_distance_factor * kProximityCostWeight;
  const double min_distance_time_cost =
      min_distance_time_factor * kMinDistanceTimeWeight;
  const double distance_travelled_cost =
      distance_travelled_factor * kDistanceTravelledCostWeight;

  cout << "COST:"
       << "min_dist_to_car:" << min_distance << ",f(" << min_distance_factor
       << "),c(" << min_dist_to_car_cost << ")" << endl
       << "\t"
       << "time_to_min_dist:" << min_distance_time * .02 << ",f("
       << min_distance_time_factor << "),c(" << min_distance_time_cost << endl
       << "\t"
       << "distance_travelled:" << distance_travelled << ",f("
       << distance_travelled_factor << "),c(" << distance_travelled_cost << ")"
       << endl;

  return min_dist_to_car_cost + distance_travelled_cost +
         min_distance_time_cost;
}

Plan GeneratePathAndCost(const std::string &plan_name,
                         const Path &prev_path_map, const CarState &sdc_state,
                         const std::vector<Path> &others,
                         const MapState &map_state,
                         std::tuple<double, double> end_path_s_d,
                         int target_lane, double accel, double max_speed,
                         double time_step, double time_horizon) {
  // Generate a reference path for the current lane in map co-ordinates.
  Path ref_path_map = GenerateReferencePath(
      prev_path_map, sdc_state, end_path_s_d, target_lane, map_state);
  Path drivable_path = GenerateSDCPathByTimeSamples(
      map_state, ref_path_map, prev_path_map, sdc_state, accel, max_speed,
      time_step, time_horizon);

  double path_cost = PathCost(sdc_state, drivable_path, others, time_step);
  cout << plan_name << " cost = " << path_cost << endl;
  return std::make_tuple(plan_name, path_cost, drivable_path);
};
