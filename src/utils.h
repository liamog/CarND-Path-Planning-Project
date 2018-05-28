//
// Created by Liam O'Gorman on 5/21/18.
//
#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>

#include "map_state.h"
#include "point.h"

double deg_to_rad(double x);
double rad_to_deg(double x);
// Speed conversion
double mph2mps(double mph);

// Lane to lateral distance (frenet)
double lane_to_frenet_d(int lane);
int frenet_d_to_lane(double d);

Point map2car(const Point &car_in_map, double psi, const Point &input);

Point car2map(const Point &car_in_map, double psi, const Point &input);

double distance(double x1, double y1, double x2, double y2);

Point getXY(double s, double d, const MapState &map_state);

std::vector<double> getFrenet(double x, double y, double theta,
                         const std::vector<double> &maps_x,
                         const std::vector<double> &maps_y);

#endif //PATH_PLANNING_UTILS_H
