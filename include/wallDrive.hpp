#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

//distance sensors
extern pros::Distance leftDist;
extern pros::Distance rightDist;
extern pros::Distance frontDist;
extern pros::Distance rightDistAlt;
// extern pros::Distance backDist;

// util functions
double normalize_angle(double a);
bool near_angle(double theta, double target, double tol = 1.0);
float distance_in(pros::Distance &sensor);
float sensor_confidence(float d);

// Odom correction
void correct_odom_with_sensors();

// Wall drive and point to point movement
void wall_drive_x(float x_target, int speed = 80);
void wall_drive_y(float y_target, int speed = 80);
void wall_drive_to_point(float x_target, float y_target, int speed = 80);
void wall_drive_to_point_direct(float x_target, float y_target, int speed = 80);

//angle correction
void turn_to_angle(double target_angle, int speed = 90, double tol = 1.0);

//measure correct y coordinate
void wall_snap(double known_x);

void print_odom();