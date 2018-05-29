//
// Created by Liam O'Gorman on 5/28/18.
//

#include <limits.h>

#include "generate_path.h"
#include "map_state.h"
#include "utils.h"

#include "gtest/gtest.h"

TEST(GeneratePaths, GenerateFirstReferencePath) {
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");
  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path prev_path_map;
  Path drivable_path = GenerateReferencePath(prev_path_map, sdc_state,
                                             sdc_state.Lane(),
                                             map_state);
}

TEST(TimeSampledPath, GeneratePathByTimeSamples) {
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");

  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path prev_path_map;
  Path drivable_path = GenerateReferencePath(prev_path_map, sdc_state,
                                             sdc_state.Lane(),
                                             map_state);

  Path ref_path = GeneratePathByTimeSamples(drivable_path,
                                            prev_path_map,
                                            sdc_state,
                                            5.0,
                                            mph_to_mps(49.5));

}

TEST(GeneratePaths, GenerateMultipleReferencePath) {
  MapState map_state;
  map_state.LoadMapState("../data/highway_map.csv");
  CarState sdc_state(909.48, 1128.67, 124.8336, 6.164, 0.0, 0.0);
  Path prev_path_map;
  Path ref_path = GenerateReferencePath(prev_path_map, sdc_state,
                                             sdc_state.Lane(),
                                             map_state);

  Path drivable_path = GeneratePathByTimeSamples(ref_path,
                                                 prev_path_map,
                                                 sdc_state,
                                                 5.0,
                                                 mph_to_mps(49.5));

  Path ref_path2 = GenerateReferencePath(drivable_path,
                                         sdc_state,
                                         sdc_state.Lane(),
                                         map_state);

  drivable_path.erase(drivable_path.begin(), drivable_path.begin() + 3);
  Path drivable_path2 = GeneratePathByTimeSamples(ref_path2,
                                                  prev_path_map,
                                                  sdc_state,
                                                  5.0,
                                                  mph_to_mps(49.5));
}
