//
// Created by Liam O'Gorman on 5/28/18.
//

#ifndef PATH_PLANNING_GENERATEPATH_H
#define PATH_PLANNING_GENERATEPATH_H

#include <vector>

#include "utils.h"
#include "car_state.h"
#include "map_state.h"

typedef std::vector<Point> Path;

Path MapPathToCarPath(const CarState &car,
                               const Path &map_path);

Path CarPathToMapPath(const CarState &car,
                               const Path &car_path);

std::vector<Point> PathFromVectors(const std::vector<double> &x,
                                   const std::vector<double> &y);

std::pair<std::vector<double>, std::vector<double>> VectorsFromPath(
      const Path &path);

Path PointsFromVectors(const std::vector<double> &x,
                                     const std::vector<double> &y);


// Generates a reference path (without taking the speed of the car into account)
// This is a sparse set of waypoints that should be smoothed out at a later
// stage with a spline.
Path GenerateReferencePath(const Path &prev_path,
                           const CarState &car_state,
                           const int target_lane,
                           const MapState &map_state);

Path GeneratePathByTimeSamples(const Path &ref_path_map,
                               const CarState &sdc_state,
                               double accel, double max_speed);
#endif //PATH_PLANNING_GENERATEPATH_H
