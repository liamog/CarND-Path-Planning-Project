//
// Created by Liam O'Gorman on 5/28/18.
//
#include "generate_path.h"

#include <cmath>
#include <iostream>
#include <vector>
#include "spline.h"
#include "path.h"

using namespace std;

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
                                    const CarState &car_state,
                                    const int target_lane,
                                    const MapState &map_state) {
  // Create a reference set of waypoints that includes the current
  // car position.
  Path path;

  if (prev_map_path.size() < 2) {
    // Create an initial reference path.
    path.emplace_back(car_state.point());
    double spacing = 30.0;
    for (int ii = 1; ii <= 3; ii++) {
      Point point = getXY(car_state.s() + (spacing * ii),
                          lane_to_frenet_d(target_lane), map_state);
      path.push_back(point);
    }
  } else {
    // Add samples from previous path to the new reference path.
    int steps = prev_map_path.size() / 3;
    for (int ii = 0; ii < prev_map_path.size() - 1; ii += steps) {
      path.emplace_back(prev_map_path[ii]);
    }
    // Make sure the last point in the previous path is on the new reference
    // path to ensure a smooth transition when growing the trajectory.
    path.emplace_back(prev_map_path.back());

    Point point =
        getXY(car_state.s() + 90.0, lane_to_frenet_d(target_lane), map_state);
    path.push_back(point);
  }

  //  path.emplace_back(car_state.point());

  // Next create 3 points for the forward path.

  //  DumpPath("GenerateReferencePath", path);
  return path;
}

Path GeneratePathByTimeSamples(const Path &ref_path_map,
                               const Path &prev_path_map,
                               const CarState &sdc_state, double accel,
                               double max_speed) {
  Path prev_path_car = MapPathToCarPath(sdc_state, prev_path_map);
  DumpPath("prev_path_car", prev_path_car);
  Path reference_path_car = MapPathToCarPath(sdc_state, ref_path_map);
  tk::spline spline;
  pair<vector<double>, vector<double>> ref =
      VectorsFromPath(reference_path_car);
  spline.set_points(ref.first, ref.second);

  constexpr double kTimeStep = 1.0 / 50;
  constexpr double kTimeHorizon = 2.0;
  const int kTimeSamples = kTimeHorizon * 50;
  const double kMaxDistanceHorizon = kTimeHorizon * max_speed;
  int kNumImmutable = 30;

  double v = std::min(sdc_state.v(), max_speed);
  // Build a path of way points sampled from the spline waypoints (spatial).
  // so they cover the distance we want to travel in the time step.
  Path time_sampled_path;
  double x = 0.0, y = 0.0;
  for (int ii = 0, jj = 0; ii < kTimeSamples; ++ii) {
    if (ii < prev_path_car.size()) {
      time_sampled_path.emplace_back(prev_path_car[ii]);
      v = distance(x, y, prev_path_car[ii].x, prev_path_car[ii].y) / kTimeStep;
      x = prev_path_car[ii].x;
      y = prev_path_car[ii].y;
      continue;
    }

    double target_distance =
        (v * kTimeStep) + (0.5 * accel * kTimeStep * kTimeStep);
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
    v = std::min(v + accel * kTimeStep, max_speed);
  }
  cout << endl;
  DumpPath("time_sampled_path_car", time_sampled_path);
  // Convert the time_sampled_path back to map coords.
  return CarPathToMapPath(sdc_state, time_sampled_path);
}

Path GeneratePathByTimeSamples2(const Path &ref_path_map,
                                const CarState &sdc_state, double accel,
                                double max_speed) {
  Path reference_path_car = MapPathToCarPath(sdc_state, ref_path_map);
  tk::spline spline;
  pair<vector<double>, vector<double>> ref =
      VectorsFromPath(reference_path_car);
  spline.set_points(ref.first, ref.second);
  constexpr double kTimeStep = 1.0 / 50;
  constexpr double kTimeHorizon = 1.0;
  const int kTimeSamples = kTimeHorizon * 50;
  const double kMaxDistanceHorizon = kTimeHorizon * max_speed;
  constexpr double kDistSample = 0.10;

  // Next we need to sample the spline at 0.2second intervals, applying the,
  // acceleration provided up to the max reference speed.
  double point_speed = sdc_state.v();
  double v = sdc_state.v();

  // if we assume a straight line with no curvature, then the max distance we
  // care abu
  Path spatial_path;
  vector<double> dist_step;
  double x = kDistSample, prev_x = 0.0;
  double y = 0.0, prev_y = 0.0;
  spatial_path.emplace_back(0.0, 0.0);
  while (x < kMaxDistanceHorizon) {
    y = spline(x);
    spatial_path.emplace_back(x, y);
    double dist_travelled = sqrt(pow(x - prev_x, 2) + pow(y - prev_y, 2));
    dist_step.push_back(dist_travelled);
    prev_x = x;
    prev_y = y;
    x += kDistSample;
  }
  cout << endl;

  // Build a path of way points sampled from the spline waypoints (spatial).
  // so they cover the distance we want to travel in the time step.
  Path time_sampled_path;
  time_sampled_path.push_back(spatial_path[0]);
  double target_distance = 0;
  for (int ii = 0, jj = 0; ii < kTimeSamples && jj < spatial_path.size();
       ++ii) {
    target_distance += (v * kTimeStep) + (0.5 * accel * kTimeStep * kTimeStep);

    double distance_travelled = 0.0;
    // Now walk the spatial waypoints until we have consumed the required
    // distance. We will use the nearest waypoint from the spatial set, which
    // may be slightly past the target distance.
    while (distance_travelled < target_distance) {
      distance_travelled += dist_step[jj];
      jj++;
    }
    // one of jj and jj-1 of the spatial path are closest to the target,
    // we will pick the closest.
    const double prev_dist_travelled = distance_travelled - dist_step[jj - 1];
    const double prev_dist_to_target = target_distance - prev_dist_travelled;
    const double dist_to_target = distance_travelled - target_distance;
    if (prev_dist_to_target < dist_to_target) {
      // We will use the point just before the target distance.
      jj--;
      distance_travelled = prev_dist_travelled;
    }
    target_distance -= distance_travelled;
    time_sampled_path.push_back(spatial_path[jj]);
    // Update our speed for the next waypoint
    v = std::min(v + accel * kTimeStep, max_speed);
    cout << "v=" << v << ";tdist=" << target_distance
         << ";x=" << spatial_path[jj].x << ";y=" << spatial_path[jj].y << "|";
  }
  cout << endl;
  DumpPath("time_sampled_path_car", time_sampled_path);
  // Convert the time_sampled_path back to map coords.
  return CarPathToMapPath(sdc_state, time_sampled_path);
}