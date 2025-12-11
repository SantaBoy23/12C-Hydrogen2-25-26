#include "main.h"

//Parallel function to change Match Load status
void MatchLoadDrop(bool MatchLoadState) {
    matchLoad.set(MatchLoadState);
}

void MatchLoadControl() {
    //If L2 is pressed, toggle Match Load state
    matchLoad.button_toggle(master.get_digital(DIGITAL_L2));
}