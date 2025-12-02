#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline pros::Motor intakeBottom(-10);
inline pros::Motor intakeTop(-1);

void IntakeLiftDrop(bool IntakeLiftState);

void BottomIntakeMove();
void TopIntakeMove();
void IntakeControl();
void FullIntakeMove();