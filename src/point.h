//
// Created by Liam O'Gorman on 5/28/18.
//

#ifndef PATH_PLANNING_POINT_H
#define PATH_PLANNING_POINT_H

#include <iostream>
#include <vector>

struct Point {
  Point();
  Point(const Point &) = default;
//  Point(std::vector<double> xy);
  Point(double x, double y, double theta);
  Point(double x, double y, double theta, double s, double d);

  Point &operator=(const Point &) = default;
  std::vector<double> ToVector();

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double s = 0.0;
  double d = 0.0;
};

template <class CharT, class Traits>
std::basic_ostream<CharT, Traits> &operator<<(
    std::basic_ostream<CharT, Traits> &os, const Point &p) {
  os << "x:" << p.x << " y:" << p.y << " theta:" << p.theta << " s:" << p.s
     << " d:" << p.d;
  return os;
}

#endif  // PATH_PLANNING_POINT_H
