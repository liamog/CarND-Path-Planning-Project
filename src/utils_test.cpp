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
  Point car(100, 100, 0.0);
  Point input(100, 100, 0.0);
  double psi = 0.0;

  Point car_coords = Map2Car(car, psi, input);

  EXPECT_EQ(car_coords.x, 0.0);
  EXPECT_EQ(car_coords.y, 0.0);
}

TEST(SimpleConversion, MapToCarNoRotate) {
  Point car(100.0, 100.0, 0.0);
  Point input(110.0, 110.0, 0.0);
  double psi = 0.0;

  Point car_coords = Map2Car(car, psi, input);

  EXPECT_EQ(car_coords.x, 10.0);
  EXPECT_EQ(car_coords.y, 10.0);
}

TEST(SimpleConversion, MapToCarAndBack) {
  double psi = M_PI_2;
  Point car(100.0, 100.0, psi);
  Point input(110.0, 110.0, psi);

  Point car_coords = Map2Car(car, psi, input);
  Point back_to_map = Car2Map(car, psi, car_coords);

  EXPECT_NEAR(back_to_map.x, 110.0, 0.01);
  EXPECT_NEAR(back_to_map.y, 110.0, 0.01);
}

TEST(SimpleConversion, MapToCarAndBack2) {
  double psi = -M_PI_2;
  Point car(-75.0, 100.0, psi);
  Point input(10.0, 99.0, psi);

  Point car_coords = Map2Car(car, psi, input);
  Point back_to_map = Car2Map(car, psi, car_coords);

  EXPECT_NEAR(back_to_map.x, 10.0, 0.01);
  EXPECT_NEAR(back_to_map.y, 99.0, 0.01);
}