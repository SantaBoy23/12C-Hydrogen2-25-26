#include "main.h"

// Parallel function to control bottom intake speed
void BottomIntakeMove(int IntakeSpeed) {
    intakeBottom.move(IntakeSpeed);
}

// Parallel function to control top intake speed
void TopIntakeMove(int IntakeSpeed) {
    intakeTop.move(IntakeSpeed);
}

void FullIntakeMove(int IntakeSpeed) {
    intakeTop.move(IntakeSpeed);
    intakeBottom.move(-IntakeSpeed);
}

void HoodLift(bool HoodState) {
    hood.set(HoodState);
}

void HoodControl() {

}

// Function for intake driver control
void IntakeControl() {

    // If L1 is pressed, spin bottom intake forwards (top intake spins backwards to keep blocks from falling out)
    if (master.get_digital(DIGITAL_L1)) {
        BottomIntakeMove(127);
        TopIntakeMove(0);
        HoodLift(false);

    } 

    // If R1 is pressed, spin intake forwards
    else if (master.get_digital(DIGITAL_R1)) {
        BottomIntakeMove(127);      
        TopIntakeMove(127);
        HoodLift(false);
    } 

    // If R2 is pressed, spin intake backwards
    else if (master.get_digital(DIGITAL_R2)) {
        BottomIntakeMove(-127);
        TopIntakeMove(-127);
    }

    else if (master.get_digital(DIGITAL_RIGHT)) {
        BottomIntakeMove(127);
        TopIntakeMove(-127);
        HoodLift(true);
    }

    // If no button is pressed, stop intake from spinning
    else {
        BottomIntakeMove(0);
        TopIntakeMove(0);
    }
}

