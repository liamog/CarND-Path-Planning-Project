//
// Created by Liam O'Gorman on 5/28/18.
//
#include "generate_path.h"

#include <cmath>
#include <vector>
#include "spline.h"

using namespace std;

vector<Point> MapPathToCarPath(const CarState &car,
                               const std::vector<Point> &map_path) {
  vector<Point> car_path;
  car_path.reserve(map_path.size());
  for (const Point &map_point: map_path) {
    car_path.emplace_back(map2car(car.point(), car.yaw_rad(), map_point));
  }
  return car_path;
}

vector<Point> CarPathToMapPath(const CarState &car,
                               const std::vector<Point> &car_path) {
  vector<Point> map_path;
  map_path.reserve(car_path.size());
  for (const Point &car_point: car_path) {
    map_path.emplace_back(car2map(car.point(), car.yaw_rad(), car_point));
  }
  return map_path;
}

std::pair<std::vector<double>, std::vector<double>> VectorsFromPath(
    const Path &path) {
  std::vector<double> x;
  std::vector<double> y;
  x.resize(path.size());
  y.resize(path.size());

  for(int ii =0; ii <  path.size(); ++ii) {
    x[ii] = path[ii].x;
    y[ii] = path[ii].y;
  }
  return std::make_pair(x, y);
};

Path PointsFromVectors(const std::vector<double> &x,
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
  vector<Point> path_points;

  if (prev_map_path.size() < 2) {
    path_points.emplace_back(car_state.point().x - cos(car_state.yaw_rad()),
                             car_state.point().y - sin(car_state.yaw_rad()));

    path_points.emplace_back(car_state.point());
  } else {
    path_points.emplace_back(prev_map_path[0]);
    path_points.emplace_back(prev_map_path[1]);
  }

  // Next create 3 points for the forward path.

  double spacing = 30.0;
  for (int ii = 1; ii <= 3; ii++) {
    Point point = getXY(car_state.s() + (spacing * ii),
                        lane_to_frenet_d(target_lane),
                        map_state);
    path_points.push_back(point);
  }
  return path_points;
}

Path GeneratePathByTimeSamples(const Path &ref_path_map,
                               const CarState &sdc_state,
                               double accel, double max_speed) {
  Path reference_path_car = MapPathToCarPath(sdc_state, ref_path_map);
  tk::spline spline;
  pair<vector<double>, vector<double>> ref= VectorsFromPath(
      reference_path_car);
  spline.set_points(ref.first, ref.second);
  constexpr double kTimeStep = 1.0 / 50;
  constexpr double kTimeHorizon = 5.0;
  const int kTimeSamples =  kTimeHorizon * 50;
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
  double y = 0.0 , prev_y = 0.0;
  while (x < kMaxDistanceHorizon) {
    y = spline(x);
    spatial_path.emplace_back(x, y);
    dist_step.push_back(sqrt( pow(x-prev_x, 2) + pow(y - prev_y, 2)));
  }


  // Build a path of 50 way points sampled from the spline waypoints (spatial).
  // so they cover the distance we want to travel in the time step.
  Path time_sampled_path;
  time_sampled_path.push_back(spatial_path[0]);
  for (int ii=0, jj=0; ii < kTimeSamples && jj < spatial_path.size() ; ++ii) {
    double target_distance = (sdc_state.v() * kTimeStep) + (0.5 * accel * sdc_state.v() * sdc_state.v());
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
    const double prev_dist_travelled = distance_travelled - dist_step[jj];
    const double prev_dist_to_target = target_distance - prev_dist_travelled;
    const double dist_to_target = distance_travelled - target_distance;
    if (prev_dist_to_target < dist_to_target) {
      // We will use the point just before the target distance.
      jj--;
      distance_travelled = prev_dist_travelled;
    }
    time_sampled_path.push_back(spatial_path[jj]);
    // Update our speed for the next waypoint
    v = sqrt(v * v  + 2 * accel * distance_travelled);
  }
  // Convert the time_sampled_path back to map coords.
  return CarPathToMapPath(sdc_state, time_sampled_path);
}