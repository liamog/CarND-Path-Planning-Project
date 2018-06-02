//
// Created by Liam O'Gorman on 5/21/18.
//
#include "utils.h"
#include <limits.h>
#include <cmath>
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
  Point car(100.0, 100.0);
  Point input(110.0, 110.0);
  double psi = 0.0;

  Point car_coords = map2car(car, psi, input);

  EXPECT_EQ(car_coords.x, 10.0);
  EXPECT_EQ(car_coords.y, 10.0);
}

TEST(SimpleConversion, MapToCarAndBack) {
  Point car(100.0, 100.0);
  Point input(110.0, 110.0);
  double psi = M_PI_2;

  Point car_coords = map2car(car, psi, input);
  Point back_to_map = car2map(car, psi, car_coords);

  EXPECT_NEAR(back_to_map.x, 110.0, 0.01);
  EXPECT_NEAR(back_to_map.y, 110.0, 0.01);
}

TEST(SimpleConversion, MapToCarAndBack2) {
  Point car(-75.0, 100.0);
  Point input(10.0, 99.0);
  double psi = -M_PI_2;

  Point car_coords = map2car(car, psi, input);
  Point back_to_map = car2map(car, psi, car_coords);

  EXPECT_NEAR(back_to_map.x, 10.0, 0.01);
  EXPECT_NEAR(back_to_map.y, 99.0, 0.01);
}