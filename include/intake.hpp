#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline pros::Motor intakeBottom(-10);
inline pros::Motor intakeTop(-1);

inline ez::Piston hood('B');

void IntakeLiftDrop(bool IntakeLiftState);
void HoodLift(bool HoodState);

void BottomIntakeMove();
void TopIntakeMove();
void IntakeControl();
void FullIntakeMove();