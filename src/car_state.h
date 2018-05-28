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
  CarState(double x, double y, double s, double d, double yaw_deg, double v);

  // Constructor for other road users.
  CarState(const std::string &id,
           double x,
           double y,
           double vx,
           double vy,
           double s,
           double d);

  double s() const { return s_; }

  double d() const { return d_; }

  double yaw_rad() const { return yaw_rad_; }

  double v() const { return v_; }

  const Point &point() const { return point_;}

private:
  std::string id_;
  double s_ = 0.0;
  double d_ = 0.0;
  double yaw_rad_ = 0.0;
  double v_ = 0.0;
  int lane_ = 1;
  Point point_;
};


#endif //PATH_PLANNING_CARSTATE_H
