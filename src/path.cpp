//
// Created by Liam O'Gorman on 6/2/18.
//
#include "path.h"

void DumpPath(const char *name, const Path &path) {
  cout << name << endl;
  for (const Point &point : path) {
    cout << "x=" << point.x << ";y=" << point.y << "|";
  }
  cout << endl;
}

double PathCost(const Path &time_path, const std::vector<CarState> &others,
                double time_step) {
  // Incoming path should be a time sampled path.

}