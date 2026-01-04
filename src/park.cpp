#include "main.h"

//Parallel function to change Park status
void ParkDrop(bool ParkState) {
    park.set(ParkState);
}

void ParkControl() {
    //If DOWN Arrow is pressed, toggle Park state
    // park.button_toggle(master.get_digital(DIGITAL_DOWN));
}