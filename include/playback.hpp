#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

//Include all subsytems
#include "drivetrain.hpp"
#include "intake.hpp"
#include "matchLoad.hpp"
#include "antenna.hpp"
#include "park.hpp"
#include "centerDescore.hpp"

void recordSkillsDataPoint();
void startSkillsRecording();
void stopSkillsRecording();
void startSkillsPlayback();
void stopSkillsPlayback();
void playSkillsPlayback();
void playbackButtonControls();