//
// Created by Liam O'Gorman on 5/28/18.
//
#include "point.h"


Point::Point() {}
Point::Point(std::vector<double> xy ) : x(xy[0]), y(xy[1]) {}
Point::Point(double x_arg, double y_arg) : x(x_arg), y(y_arg) {}
