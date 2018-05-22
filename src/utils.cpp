//
// Created by Liam O'Gorman on 5/21/18.
//
#include "utils.h"

#include <cmath>

// For converting back and forth between radians and degrees.

double deg2rad(double x) { return x * M_PI / 180; }

double rad2deg(double x) { return x * 180 / M_PI; }

// Speed conversion
double mph2mps(double mph) { return mph * 0.44704; }

// Lane to lateral distance (frenet)
double lane2frenet_d(int lane) { return lane * 4.0 +2.0;}

Point::Point() {}
Point::Point(std::vector<double> xy ) : x(xy[0]), y(xy[1]) {}
Point::Point(double x_arg, double y_arg) : x(x_arg), y(y_arg) {}

std::vector<double> Point::ToVector() {
  std::vector<double> val({x, y});
  return val;
}

Point map2car(const Point &car_in_map, double psi, const Point &input) {
  double s = sin(-psi);
  double c = cos(-psi);
  // Move to origin
  double x = (input.x - car_in_map.x);
  double y = (input.y - car_in_map.y);
  Point point_in_car_coords;
  // Rotate by psi
  point_in_car_coords.x = (x * c) - (y * s);
  point_in_car_coords.y = (x * s) + (y * c);
  return point_in_car_coords;
}

Point car2map(const Point &car_in_map, double psi, const Point &input) {
  double s = sin(psi);
  double c = cos(psi);

  // Rotate
  double x_rot = input.x * c - input.y * s;
  double y_rot = input.x * s + input.y * c;

  // Move relative to car pos.
  Point point_in_map_coords;
  point_in_map_coords.x = car_in_map.x + x_rot;
  point_in_map_coords.y = car_in_map.y + y_rot;
  return point_in_map_coords;
}

