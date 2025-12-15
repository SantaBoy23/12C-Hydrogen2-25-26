#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline ez::Piston Antenna('C');

void AntennaRaise(bool AntennaState);

void AntennaControl();