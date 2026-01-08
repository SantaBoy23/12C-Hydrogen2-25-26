#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

// Distance sensors
pros::Distance leftDist(7);
pros::Distance rightDist(3);
pros::Distance frontDist(8);
// pros::Distance backDist(0);

// ------------------------------
// Utility functions
// ------------------------------
double normalize_angle(double a);
bool near_angle(double theta, double target, double tol = 1.0);
float distance_in(pros::Distance &sensor);
float sensor_confidence(float d);

// ------------------------------
// Odometry correction
// ------------------------------
void correct_odom_with_sensors();

// ------------------------------
// Wall-drive / point-to-point movement
// ------------------------------
void wall_drive_x(float x_target, int speed = 80);
void wall_drive_y(float y_target, int speed = 80);
void wall_drive_to_point(float x_target, float y_target, int speed = 80);
void wall_drive_to_point_direct(float x_target, float y_target, int speed = 80);

// ------------------------------
// Angle correction
// ------------------------------
void turn_to_angle(double target_angle, int speed = 90, double tol = 1.0);


