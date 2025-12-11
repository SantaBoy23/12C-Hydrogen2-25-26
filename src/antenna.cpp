#include "main.h"

//Parallel function to change Right Antenna status
void AntennaRaise(bool AntennaState) {
    Antenna.set(AntennaState);
}

void AntennaControl() {
    //If Right Arrow is pressed, toggle Antenna state
    Antenna.button_toggle(master.get_digital(DIGITAL_B));
}