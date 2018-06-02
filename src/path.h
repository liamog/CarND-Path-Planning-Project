//
// Created by Liam O'Gorman on 6/2/18.
//

#ifndef PATH_PLANNING_PATH_H
#define PATH_PLANNING_PATH_H

#include "car_state.h"
#include "map_state.h"
#include "point.h"

typedef std::vector<Point> Path;


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

// Generate a path for the SDC from the previous path + the reference path
// + current SDC state, given a time step and time horizon a constant accel
// and a maximum speed.
Path GenerateSDCPathByTimeSamples(const Path &ref_path_map,
                                  const Path &prev_path_map,
                                  const CarState &sdc_state, double accel,
                                  double max_speed, double time_step,
                                  double time_horizon);

// Generate a path for another car on the road, given their current state,
// time step and horizon.
Path GenerateOtherPathByTimeSamples(const CarState &other_state,
                                    const double time_step,
                                    const double time_horizon,
                                    const MapState &map_state);

double PathCost(const Path &time_path, const std::vector<Path> &others);

typedef std::tuple<std::string, double, Path> Plan;

Plan GeneratePathAndCost(const std::string &name, const Path &prev_path_map,
                         const CarState &car_state,
                         const std::vector<Path> &others,
                         const MapState &map_state, int target_lane,
                         const double accel, const double max_speed,
                         double time_step, double time_horizon);

#endif  // PATH_PLANNING_PATH_H
