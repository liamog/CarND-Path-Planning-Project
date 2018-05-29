//
// Created by Liam O'Gorman on 5/28/18.
//

#include "car_state.h"
#include "utils.h"

#include <sstream>

CarState::CarState(double x, double y, double s, double d, double yaw_deg, double v) :
    id_("SDC"), s_(s), d_(d), yaw_rad_(deg_to_rad(yaw_deg)), v_(v) , point_(x,y) {
  lane_ = frenet_d_to_lane(d);
}

// Constructor for other road users.
CarState::CarState(const std::string &id,
         double x,
         double y,
         double vx,
         double vy,
         double s,
         double d) :
    id_(id), s_(s), d_(d) , point_(x,y), v_(sqrt(vx*vx + vy*vy)) {
  lane_ = frenet_d_to_lane(d);
}

std::string CarState::ToString() const {
  std::stringstream retval;
  retval << "id=" << id_
      << "x=" << point_.x
      << ";y=" << point_.y
      << ";v=" << v_
      << ";yaw=" << yaw_rad_
      << ";yaw_deg=" << rad_to_deg(yaw_rad_)
      << ";l=" << Lane();
  return retval.str();
}
