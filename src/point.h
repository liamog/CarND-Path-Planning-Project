//
// Created by Liam O'Gorman on 5/28/18.
//

#ifndef PATH_PLANNING_POINT_H
#define PATH_PLANNING_POINT_H

#include <vector>
#include <iostream>

struct Point {
  Point();
  Point(const Point &) = default;
  Point(std::vector<double> xy);
  Point(double x, double y);

  Point &operator=(const Point &) = default;
  std::vector<double> ToVector();

  double x = 0.0;
  double y = 0.0;
};


template <class CharT, class Traits>
std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits>& os,
           const Point &p) {
  os << "x:" << p.x << " y:" << p.y;
  return os;
}

#endif  // PATH_PLANNING_POINT_H
