//
// Created by Liam O'Gorman on 5/21/18.
//
#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>

struct Point {
  Point();
  Point(std::vector<double> xy );
  Point(double x, double y);

  std::vector<double> ToVector();

  double x = 0.0;
  double y = 0.0;
};


double deg2rad(double x);
double rad2deg(double x);
// Speed conversion
double mph2mps(double mph);

// Lane to lateral distance (frenet)
double lane2frenet_d(int lane);

Point map2car(const Point &car_in_map, double psi, const Point &input);

Point car2map(const Point &car_in_map, double psi, const Point &input);

#endif //PATH_PLANNING_UTILS_H
