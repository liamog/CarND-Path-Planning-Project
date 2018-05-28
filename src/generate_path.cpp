//
// Created by Liam O'Gorman on 5/28/18.
//
#include "generate_path.h"

#include <cmath>
#include <vector>

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
  x.reserve(path.size());
  y.reserve(path.size());

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