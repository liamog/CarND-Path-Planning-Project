//
// Created by Liam O'Gorman on 5/28/18.
//

#include "map_state.h"

#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

void MapState::LoadMapState(const char* path) {
  // Waypoint map to read from
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(path, ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;

    map_waypoints_x_.push_back(x);
    map_waypoints_y_.push_back(y);
    map_waypoints_s_.push_back(s);
    map_waypoints_dx_.push_back(d_x);
    map_waypoints_dy_.push_back(d_y);
  }
}