//
// Created by Liam O'Gorman on 5/28/18.
//
#include "point.h"
#include "map_state.h"

#include <vector>

Point::Point() {}

Point::Point(double x_arg, double y_arg, double theta_arg)
    : x(x_arg), y(y_arg), theta(theta_arg) {}

Point::Point(double x_arg, double y_arg, double theta_arg, double s_arg,
             double d_arg)
    : x(x_arg), y(y_arg), theta(theta_arg), s(s_arg), d(d_arg) {}

std::vector<double> Point::ToVector() {
  std::vector<double> val({x, y});
  return val;
}