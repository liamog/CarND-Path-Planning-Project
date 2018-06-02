//
// Created by Liam O'Gorman on 6/2/18.
//

//
// Created by Liam O'Gorman on 5/28/18.
//

#include <limits.h>

#include "map_state.h"
#include "path.h"
#include "utils.h"

#include "gtest/gtest.h"

TEST(GeneratePaths, GenerateFirstReferencePath) {
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");
  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path prev_path_map;
  Path drivable_path = GenerateReferencePath(prev_path_map, sdc_state,
                                             sdc_state.Lane(), map_state);
}

TEST(TimeSampledPath, GenerateSDCPathByTimeSamples) {
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");

  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path prev_path_map;
  Path drivable_path = GenerateReferencePath(prev_path_map, sdc_state,
                                             sdc_state.Lane(), map_state);

  Path ref_path =
      GenerateSDCPathByTimeSamples(drivable_path, prev_path_map, sdc_state, 5.0,
                                   mph_to_mps(49.5), 1.0 / 50.0, 1.0);
}

TEST(GeneratePaths, GenerateMultipleReferencePath) {
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");
  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path prev_path_map;
  Path ref_path = GenerateReferencePath(prev_path_map, sdc_state,
                                        sdc_state.Lane(), map_state);

  Path drivable_path =
      GenerateSDCPathByTimeSamples(ref_path, prev_path_map, sdc_state, 5.0,
                                   mph_to_mps(49.5), 1.0 / 50.0, 1.0);

  Path ref_path2 = GenerateReferencePath(drivable_path, sdc_state,
                                         sdc_state.Lane(), map_state);

  drivable_path.erase(drivable_path.begin(), drivable_path.begin() + 3);
  Path drivable_path2 =
      GenerateSDCPathByTimeSamples(ref_path2, prev_path_map, sdc_state, 5.0,
                                   mph_to_mps(49.5), 1.0 / 50.0, 1.0);
}

TEST(GeneratePaths, CalculateCostMoveOneMeterLessThanDontMove) {
  std::vector<Path> others;

  Path sdc_path1;
  sdc_path1.emplace_back(0.0, 0.0);
  sdc_path1.emplace_back(1.0, 0.0);
  double cost_move = PathCost(sdc_path1, others);

  Path sdc_path2;
  sdc_path2.emplace_back(0.0, 0.0);
  sdc_path2.emplace_back(0.0, 0.0);
  double cost_dont_move = PathCost(sdc_path2, others);

  GTEST_ASSERT_GE(cost_dont_move, cost_move);
}

TEST(GeneratePaths, CalculateCostCloserToOther) {
  Path other_path1;
  other_path1.emplace_back(5.0, 0.0);
  other_path1.emplace_back(6.0, 0.0);

  std::vector<Path> others;
  others.push_back(other_path1);

  Path sdc_path1;
  sdc_path1.emplace_back(1.0, 0.0);
  sdc_path1.emplace_back(2.0, 0.0);
  double cost_closer = PathCost(sdc_path1, others);

  Path sdc_path2;
  sdc_path2.emplace_back(0.0, 0.0);
  sdc_path2.emplace_back(1.0, 0.0);
  double cost_further = PathCost(sdc_path2, others);

  GTEST_ASSERT_GE(cost_closer, cost_further);
}