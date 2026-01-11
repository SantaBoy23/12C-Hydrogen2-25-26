#include "main.h"

//Parallel function to change Right Antenna status
void AntennaRaise(bool AntennaState) {
    Antenna.set(AntennaState);
}

void AntennaControl() {
    //remove during skills
    // //If b button is pressed, toggle Antenna state
    // Antenna.button_toggle(master.get_digital(DIGITAL_B));

    //If b button is held, raise antenna.
    if (master.get_digital(DIGITAL_B)) {
        AntennaRaise(true);
    } 

    // Otherwise, keep antenna lowered
    else {
        AntennaRaise(false);
    } 
}