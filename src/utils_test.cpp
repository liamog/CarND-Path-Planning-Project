//
// Created by Liam O'Gorman on 5/21/18.
//
#include <limits.h>
#include "utils.h"
#include "gtest/gtest.h"

TEST(SimpleConversion, MapToCarSameOrigin) {
// This test is named "Negative", and belongs to the "FactorialTest"
// test case.
  Point car(100, 100);
  Point input(100, 100);
  double psi = 0.0;

  Point car_coords = map2car(car, psi, input);

  EXPECT_EQ(car_coords.x, 0.0);
  EXPECT_EQ(car_coords.y, 0.0);
}

TEST(SimpleConversion, MapToCarNoRotate) {
// This test is named "Negative", and belongs to the "FactorialTest"
// test case.
  Point car(100.0, 100.0);
  Point input(110.0, 110.0);
  double psi = 0.0;

  Point car_coords = map2car(car, psi, input);

  EXPECT_EQ(car_coords.x, 10.0);
  EXPECT_EQ(car_coords.y, 10.0);
}


TEST(SimpleConversion, MapToCarAndBack) {
// This test is named "Negative", and belongs to the "FactorialTest"
// test case.
  Point car(100.0, 100.0);
  Point input(110.0, 110.0);
  double psi = 45.0;

  Point car_coords = map2car(car, psi, input);
  Point back_to_map = car2map(car, psi, car_coords);

  EXPECT_EQ(back_to_map.x, 110.0);
  EXPECT_EQ(back_to_map.y, 110.0);
}