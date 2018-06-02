//
// Created by Liam O'Gorman on 5/28/18.
//

#ifndef PATH_PLANNING_GENERATEPATH_H
#define PATH_PLANNING_GENERATEPATH_H

#include <vector>

#include "car_state.h"
#include "map_state.h"
#include "path.h"
#include "utils.h"

void DumpPath(const char *name, const Path &path);

Path MapPathToCarPath(const CarState &car, const Path &map_path);

Path CarPathToMapPath(const CarState &car, const Path &car_path);

std::vector<Point> PathFromVectors(const std::vector<double> &x,
                                   const std::vector<double> &y);

std::pair<std::vector<double>, std::vector<double>> VectorsFromPath(
    const Path &path);

Path PathFromVectors(const std::vector<double> &x,
                     const std::vector<double> &y);

// Generates a reference path (without taking the speed of the car into account)
// This is a sparse set of waypoints that should be smoothed out at a later
// stage with a spline.
Path GenerateReferencePath(const Path &prev_path, const CarState &sdc_state,
                           const int target_lane, const MapState &map_state);

Path GenerateSDCPathByTimeSamples(const Path &ref_path_map,
                                  const Path &prev_path_map,
                                  const CarState &sdc_state, double accel,
                                  double max_speed, double time_step,
                                  double time_horizon);

Path GenerateOtherPathByTimeSamples(const CarState &other_state,
                                    const double time_step,
                                    const double time_horizon,
                                    const MapState &map_state);

#endif  // PATH_PLANNING_GENERATEPATH_H
