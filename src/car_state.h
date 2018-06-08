//
// Created by Liam O'Gorman on 5/28/18.
//

#ifndef PATH_PLANNING_CARSTATE_H
#define PATH_PLANNING_CARSTATE_H

#include "point.h"

#include <cmath>
#include <string>

class CarState {
 public:
  // Constructor for SDC
  CarState(double x, double y, double s, double d, double v, double yaw_deg);

  // Constructor for other road users.
  CarState(double x, double y, double s, double d, double v);

  double s() const { return point_.s; }
  double d() const { return point_.d; }
  double yaw_rad() const { return point_.theta;}
  double v() const { return v_; }
  const Point &point() const { return point_; }
  int Lane() const;

  std::string ToString() const;
  std::string ToCsvString() const;
  static std::string CsvStringHeader();

  std::vector<int> GetPossibleLanes();

 private:
  double v_ = 0.0;
  int lane_ = 1;
  Point point_;
};

#endif  // PATH_PLANNING_CARSTATE_H
