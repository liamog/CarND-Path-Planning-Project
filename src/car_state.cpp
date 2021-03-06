//
// Created by Liam O'Gorman on 5/28/18.
//

#include "car_state.h"
#include "utils.h"

#include <sstream>

CarState::CarState(double x, double y, double s, double d, double v,
                   double yaw_deg)
    : v_(v), point_(x, y, deg_to_rad(yaw_deg), s, d) {
  lane_ = frenet_d_to_lane(d);
}

// Constructor for other road users.
CarState::CarState(double x, double y, double s, double d, double v)
    : point_(x, y, 0.0, s, d), v_(v) {
  lane_ = frenet_d_to_lane(d);
}

int CarState::Lane() const { return frenet_d_to_lane(d()); }

std::string CarState::ToString() const {
  std::stringstream retval;
  retval << "x=" << point_.x << ";y=" << point_.y << ";v=" << v_
         << ";yaw=" << yaw_rad() << ";yaw_deg=" << rad_to_deg(yaw_rad())
         << ";l=" << Lane();
  return retval.str();
}

std::string CarState::ToCsvString() const {
  std::stringstream retval;
  retval << s() << "," << d() << "," << point_.x << "," << point_.y << "," << v_
         << "," << point_.theta << "," << rad_to_deg(point_.theta) << ","
         << Lane();
  return retval.str();
}

std::string CarState::ToCodeString() const {
  std::stringstream retval;
  retval << "CarState state(" << point_.x << "," << point_.y << "," << s()
         << "," << d() << "," << v_ << "," << point_.theta << ");";
  return retval.str();
}

// static
std::string CarState::CsvStringHeader() {
  std::stringstream retval;
  retval << "s"
         << ","
         << "d"
         << ","
         << "x"
         << ","
         << "y"
         << ","
         << "v"
         << ","
         << "yaw_rad"
         << ","
         << "yaw_deg"
         << ","
         << "lane";
  return retval.str();
}

// Return the set of possible lanes from the car's current position.
std::vector<int> CarState::GetPossibleLanes() {
  std::vector<int> possible_lanes;
  possible_lanes.push_back(Lane());
  if (Lane() + 1 <= 2) {
    possible_lanes.push_back(Lane() + 1);
  }
  if (Lane() - 1 >= 0) {
    possible_lanes.push_back(Lane() - 1);
  }
  return possible_lanes;
}
