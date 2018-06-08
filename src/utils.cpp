//
// Created by Liam O'Gorman on 5/21/18.
//
#include "utils.h"

#include "map_state.h"

#include <cmath>
#include <iostream>
#include <vector>

using namespace std;
// For converting back and forth between radians and degrees.

double deg_to_rad(double x) { return x * M_PI / 180; }

double rad_to_deg(double x) { return x * 180 / M_PI; }

// Speed conversion
double mph_to_mps(double mph) { return mph * 0.44704; }

// Lane to lateral Distance (frenet)
double lane_to_frenet_d(int lane) { return lane * 4.0 + 2.0; }

int frenet_d_to_lane(double frenet) { return frenet / 4.0; }

int ClosestWaypoint(double x, double y, const MapState &map_state) {
  double closestLen = 100000;  // large number
  int closestWaypoint = 0;

  for (int i = 0; i < map_state.map_waypoints_x().size(); i++) {
    double map_x = map_state.map_waypoints_x()[i];
    double map_y = map_state.map_waypoints_y()[i];
    double dist = Distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const MapState &map_state) {
  int closestWaypoint = ClosestWaypoint(x, y, map_state);

  double map_x = map_state.map_waypoints_x()[closestWaypoint];
  double map_y = map_state.map_waypoints_y()[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = min(2 * M_PI - angle, angle);

  if (angle > M_PI_4) {
    closestWaypoint++;
    if (closestWaypoint == map_state.map_waypoints_x().size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

Point Map2Car(const Point &car_in_map, double psi, const Point &input) {
  double s = sin(-psi);
  double c = cos(-psi);
  // Move to origin
  double x = (input.x - car_in_map.x);
  double y = (input.y - car_in_map.y);
  Point point_in_car_coords;
  // Rotate by psi
  point_in_car_coords.x = (x * c) - (y * s);
  point_in_car_coords.y = (x * s) + (y * c);
  point_in_car_coords.theta = input.theta - psi;

  return point_in_car_coords;
}

Point Car2Map(const Point &car_in_map, double psi, const Point &input) {
  double s = sin(psi);
  double c = cos(psi);

  // Rotate
  double x_rot = input.x * c - input.y * s;
  double y_rot = input.x * s + input.y * c;

  // Move relative to car pos.
  Point point_in_map_coords;
  point_in_map_coords.x = car_in_map.x + x_rot;
  point_in_map_coords.y = car_in_map.y + y_rot;
  point_in_map_coords.theta = car_in_map.theta + psi;

  return point_in_map_coords;
}

double Distance(const Point &p1, const Point &p2) {
  return Distance(p1.x, p1.y, p2.x, p2.y);
}

double Distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void UpdateFrenetForPoint(const MapState &map_state, Point *point) {
  std::vector<double> frenet = GetFrenet(point->x, point->y, point->theta, map_state);
  point->s = frenet[0];
  point->d = frenet[1];
}

void UpdateFrenet(const MapState &map_state, Path *path) {
  for (Point &p : *path) {
    UpdateFrenetForPoint(map_state, &p);
  }
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> GetFrenet(double x, double y, double theta,
                         const MapState &map_state) {
  int next_wp = NextWaypoint(x, y, theta, map_state);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = map_state.map_waypoints_x().size() - 1;
  }

  double n_x = map_state.map_waypoints_x()[next_wp] -
               map_state.map_waypoints_x()[prev_wp];
  double n_y = map_state.map_waypoints_y()[next_wp] -
               map_state.map_waypoints_y()[prev_wp];
  double x_x = x - map_state.map_waypoints_x()[prev_wp];
  double x_y = y - map_state.map_waypoints_y()[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = Distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - map_state.map_waypoints_x()[prev_wp];
  double center_y = 2000 - map_state.map_waypoints_y()[prev_wp];
  double centerToPos = Distance(center_x, center_y, x_x, x_y);
  double centerToRef = Distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += Distance(
        map_state.map_waypoints_x()[i], map_state.map_waypoints_y()[i],
        map_state.map_waypoints_x()[i + 1], map_state.map_waypoints_y()[i + 1]);
  }

  frenet_s += Distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
Point GetXY(double s, double d, const MapState &map_state) {
  int prev_wp = -1;

  while (s > map_state.map_waypoints_s()[prev_wp + 1] &&
         (prev_wp < (int)(map_state.map_waypoints_s().size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % map_state.map_waypoints_x().size();

  double heading = atan2(
      (map_state.map_waypoints_y()[wp2] - map_state.map_waypoints_y()[prev_wp]),
      (map_state.map_waypoints_x()[wp2] -
       map_state.map_waypoints_x()[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - map_state.map_waypoints_s()[prev_wp]);

  double seg_x = map_state.map_waypoints_x()[prev_wp] + seg_s * cos(heading);
  double seg_y = map_state.map_waypoints_y()[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - M_PI_2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return Point(x, y, heading, s, d);
}
