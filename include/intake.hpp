#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

// Declare motors, but do NOT construct them here.
extern pros::Motor intakeBottom;
extern pros::Motor intakeTop;

inline ez::Piston center('B');

void IntakeLiftDrop(bool IntakeLiftState);
void CenterLift(bool CenterState);

void BottomIntakeMove(int speed);
void TopIntakeMove(int speed);
void FullIntakeMove(int speed);
void IntakeControl();
void CenterControl();