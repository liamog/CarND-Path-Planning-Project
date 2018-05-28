//
// Created by Liam O'Gorman on 5/28/18.
//

#ifndef PATH_PLANNING_MAP_STATE_H
#define PATH_PLANNING_MAP_STATE_H

#include <vector>

class MapState {
public:
  void LoadMapState(const char *path);

  const std::vector<double> &map_waypoints_x() const {return map_waypoints_x_ ;}
  const std::vector<double> &map_waypoints_y() const {return map_waypoints_y_ ;}
  const std::vector<double> &map_waypoints_s() const {return map_waypoints_s_ ;}
  const std::vector<double> &map_waypoints_dx() const {return map_waypoints_dx_ ;}
  const std::vector<double> &map_waypoints_dy() const {return map_waypoints_dy_ ;}

private:
  std::vector<double> map_waypoints_x_;
  std::vector<double> map_waypoints_y_;
  std::vector<double> map_waypoints_s_;
  std::vector<double> map_waypoints_dx_;
  std::vector<double> map_waypoints_dy_;
};

#endif //PATH_PLANNING_MAP_STATE_H
