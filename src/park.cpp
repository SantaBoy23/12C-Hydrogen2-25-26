#include "main.h"

//Parallel function to change Park status
void ParkDrop(bool ParkState) {
    Park.set(ParkState);
}

void ParkControl() {
    //If Up Arrow is pressed, toggle Park state
    Antenna.button_toggle(master.get_digital(DIGITAL_UP));
}