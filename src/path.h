//
// Created by Liam O'Gorman on 6/2/18.
//

#ifndef PATH_PLANNING_PATH_H
#define PATH_PLANNING_PATH_H\

#include "point.h"

typedef std::vector<Point> Path;

double PathCost(const Path &time_path, const std::vector<Path> &others);

#endif //PATH_PLANNING_PATH_H
