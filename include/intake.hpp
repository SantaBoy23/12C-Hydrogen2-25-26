#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

// Declare motors, but do NOT construct them here.
inline pros::Motor intakeBottom(10);
inline pros::Motor intakeTop(6);

inline ez::Piston center('B');
inline ez::Piston bottom('G');

void IntakeLiftDrop(bool IntakeLiftState);
void CenterDrop(bool CenterState);
void BottomDrop(bool BottomState);

void BottomIntakeMove(int speed);
void TopIntakeMove(int speed);
void FullIntakeMove(int speed);
void IntakeControl();