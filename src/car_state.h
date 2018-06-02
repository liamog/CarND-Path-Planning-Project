//
// Created by Liam O'Gorman on 5/28/18.
//

#ifndef PATH_PLANNING_CARSTATE_H
#define PATH_PLANNING_CARSTATE_H

#include "point.h"
#include "utils.h"

#include <cmath>
#include <string>

class CarState {
 public:
  // Constructor for SDC
  CarState(double x, double y, double s, double d, double v, double yaw_deg);

  // Constructor for other road users.
  CarState(double x, double y, double s, double d, double v);

  double s() const { return s_; }
  double d() const { return d_; }
  double yaw_rad() const { return yaw_rad_;}
  double v() const { return v_; }
  const Point &point() const { return point_; }
  int Lane() const { return frenet_d_to_lane(d_); }

  std::string ToString() const;
  std::string ToCsvString() const;
  static std::string CsvStringHeader();

 private:
  double s_ = 0.0;
  double d_ = 0.0;
  double v_ = 0.0;
  double yaw_rad_ = 0.0;
  int lane_ = 1;
  Point point_;
};

#endif  // PATH_PLANNING_CARSTATE_H
