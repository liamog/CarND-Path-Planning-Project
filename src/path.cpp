//
// Created by Liam O'Gorman on 6/2/18.
//
#include "path.h"

#include "car_state.h"

#include <iostream>
#include <limits>

using namespace std;

void DumpPath(const char *name, const Path &path) {
  cout << name << endl;
  for (const Point &point : path) {
    cout << "x=" << point.x << ";y=" << point.y << "|";
  }
  cout << endl;
}

double PathCost(const Path &time_path, const std::vector<Path> &others) {
  // Incoming paths should be time sampled paths of the same size.
  double min_distance = std::numeric_limits<double>::infinity();
  for (int ii = 0; ii < time_path.size(); ii+=2) {
    for (const Path &other_path : others) {
      // Assume that the other car is following it's lane from it's current
      // position with its current speed.
      // Then compare this with our location.
      const double dist = distance(other_path[ii], time_path[ii]);
      min_distance = std::min(min_distance, dist);
    }
  }

  return 1 / min_distance;
}