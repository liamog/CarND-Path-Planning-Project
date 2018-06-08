//
// Created by Liam O'Gorman on 5/21/18.
//
#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>

#include "map_state.h"
#include "path.h"
#include "point.h"

double deg_to_rad(double x);
double rad_to_deg(double x);
// Speed conversion
double mph_to_mps(double mph);

// Lane to lateral Distance (frenet)
double lane_to_frenet_d(int lane);
int frenet_d_to_lane(double d);

Point Map2Car(const Point &car_in_map, double psi, const Point &input);

Point Car2Map(const Point &car_in_map, double psi, const Point &input);

double Distance(const Point &p1, const Point &p2);
double Distance(double x1, double y1, double x2, double y2);

Point GetXY(double s, double d, const MapState &map_state);

std::vector<double> GetFrenet(double x, double y, double theta,
                              const MapState &map_state);

void UpdateFrenet(const MapState &map_state, Path *path);


#endif  // PATH_PLANNING_UTILS_H
