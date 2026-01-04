#include "main.h"

//Parallel function to change Center Descore status
void CenterDescoreRaise(bool CenterDescoreState) {
    centerDescore.set(CenterDescoreState);
}

void CenterDescoreControl() {
    //If Up Arrow is pressed, toggle Center Descore state
    centerDescore.button_toggle(master.get_digital(DIGITAL_UP));
}