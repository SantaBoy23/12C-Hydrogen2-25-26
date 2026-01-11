#include "main.h"
#include "autons.hpp"
#include "wallDrive.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

void OdomPodControl() {

  //If "LEFT" button is pressed, toggle odom pod state
  odomPod.button_toggle(master.get_digital(DIGITAL_LEFT));

}

void OdomPodLift(bool OdomPodState) {
    odomPod.set(OdomPodState);
}

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(12.5, 0.0, 12.2);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions


  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms); 
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.95); //was 0.9

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void elims_left_auto() {
  //set starting angle
  chassis.drive_angle_set(180_deg);
  chassis.odom_xyt_set(60_in, 26_in, 180_deg);


  //start intake
  intakeTop.move(-127);
  intakeBottom.move(127);

  //move to three centered blocks
  chassis.pid_odom_set({{50_in, 52_in, 155_deg}, rev, 65}, true); //was90 speed //was80 speed //was150_deg
  chassis.pid_wait_quick_chain();

  //turn so top intake faces middle goal
  chassis.pid_turn_set(45_deg, TURN_SPEED); //was-1 //was 0
  chassis.pid_wait_quick_chain();

  //move to center goal and empty all blocks
  chassis.pid_odom_set({{58_in, 52.5_in}, fwd, 127}, true); //was63,61,46
  chassis.pid_wait_until(10.5_in);
  CenterDrop(true);
  intakeTop.move(-80);
  chassis.pid_wait();
  pros::delay(1300);

  //move back towards match loader
  CenterDrop(false);
  intakeTop.move(-127);
  intakeBottom.move(127);
  chassis.pid_odom_set({{23.5_in, 17_in}, rev, 127}, true); //was26
  chassis.pid_wait();

  //turn to face match loader
  chassis.pid_turn_set(1_deg, TURN_SPEED); //was-1 //was 0
  chassis.pid_wait_quick_chain();
  MatchLoadDrop(true);
  pros::delay(25);

  //move into match loader and collect 3 blocks
  chassis.pid_odom_set({{24.5_in, 6.25_in}, rev, 100}, true); //was27,7
  chassis.pid_wait();
  pros::delay(350);

  //move forward into long goal and empty blocks
  chassis.pid_odom_set({{26.5_in, 38.0_in}, fwd, 127}, true);
  chassis.pid_wait_until({26.5_in, 30_in});
  intakeTop.move(127);
  intakeBottom.move(127);
  chassis.pid_wait();
  chassis.drive_angle_set(0_deg);
  pros::delay(700);
  MatchLoadDrop(false);
  intakeTop.move(0);
  intakeBottom.move(0);

  //pull out of long goal
  chassis.pid_odom_set({{26.5_in, 28_in}, rev, 127}, true);
  chassis.pid_wait_quick_chain();
  
  //turn towards wall
  chassis.pid_turn_set(270_deg, TURN_SPEED); //was-1 //was 0
  chassis.pid_wait_quick_chain();

  //move towards wall
  chassis.pid_odom_set({{23_in, 28.1_in}, fwd, 127}, true);
  chassis.pid_wait();

  //turn so antenna is in goal
  chassis.pid_turn_set(-5_deg, TURN_SPEED); //was-1 //was 0
  chassis.pid_wait_quick_chain();

  //push antenna to end of open area
  chassis.pid_odom_set({{18_in, 50_in}, fwd, 100}, true);
  chassis.pid_wait();

  // // Lift Odom Pod
  // OdomPodLift(true);

  // // wait then lift antenna
  // pros::delay(3150);
  // AntennaRaise(true);
}

void left_antenna_auto() {
  //set starting angle
  chassis.drive_angle_set(180_deg);
  chassis.odom_xyt_set(60_in, 26_in, 180_deg);

  //start intake
  intakeTop.move(-127);
  intakeBottom.move(127);

  //move to three centered blocks
  chassis.pid_odom_set({{48_in, 48_in, 145_deg}, rev, 75}, true); //was90 speed //was80 speed //was150_deg
  chassis.pid_wait_quick_chain();

  //turn so top intake faces middle goal
  chassis.pid_turn_set(45_deg, TURN_SPEED); //was-1 //was 0
  chassis.pid_wait_quick_chain();

  //move back towards match loader
  chassis.pid_odom_set({{27_in, 24_in}, rev, 127}, true); //was25,24
  chassis.pid_wait();

  //turn to face match loader
  chassis.pid_turn_set(1_deg, TURN_SPEED); //was-1 //was 0
  chassis.pid_wait_quick_chain();
  MatchLoadDrop(true);

  //move into match loader and collect 3 blocks
  chassis.pid_odom_set({{26.5_in, 7_in}, rev, 127}, true); //was27,7
  chassis.pid_wait();
  // pros::delay(25);

  //move forward into long goal and empty all blocks
  chassis.pid_odom_set({{26.5_in, 43_in}, fwd, 127}, true);
  chassis.pid_wait_until({26.5_in, 32_in});
  intakeTop.move(127);
  intakeBottom.move(127);
  chassis.pid_wait();
  chassis.drive_angle_set(0_deg);
  pros::delay(1300);
  MatchLoadDrop(false);

  //pull out of long goal
  chassis.pid_odom_set({{26.5_in, 33_in}, rev, 127}, true);
  chassis.pid_wait_quick_chain();
  
  //turn towards wall
  chassis.pid_turn_set(268_deg, TURN_SPEED); //was-1 //was 0
  chassis.pid_wait_quick_chain();

  //move towards wall
  chassis.pid_odom_set({{18.5_in, 33_in}, fwd, 127}, true);
  chassis.pid_wait_quick_chain();

  //turn so antenna is in goal
  chassis.pid_turn_set(-5_deg, TURN_SPEED); //was-1 //was 0
  chassis.pid_wait_quick_chain();

  //push antenna to end of open area
  chassis.pid_odom_set({{18.5_in, 62_in}, fwd, 127}, true);
  chassis.pid_wait();

  //Lift Odom Pod
  OdomPodLift(true);

  //wait then lift antenna
  pros::delay(1700);
  AntennaRaise(true);
}

void right_antenna_auto() {
  //set starting angle
  chassis.drive_angle_set(180_deg); //was-142
  chassis.odom_xyt_set(84_in, 26_in, 180_deg);

  //start intake
  intakeTop.move(-127);
  intakeBottom.move(127);

  //move to three centered blocks
  chassis.pid_odom_set({{95_in, 48_in}, rev, 50}, true); //was93
  // chassis.pid_wait_until(2_in);
  // MatchLoadDrop(true);
  chassis.pid_wait();

  //turn so top intake faces middle goal
  chassis.pid_turn_set({72_in, 72_in}, rev, 127, true);
  chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(315_deg, TURN_SPEED); 
  // chassis.pid_wait_quick_chain();

  //Move towards match loader
  chassis.pid_odom_set({{113_in, 18_in}, rev, 127}, true); //was25,24
  chassis.pid_wait_quick_chain();

  //turn to face match loader
  chassis.pid_turn_set({113_in, 9_in}, rev, 127, true);
  chassis.pid_wait();
  MatchLoadDrop(true);

  //move into match loader and collect 3 blocks
  chassis.pid_odom_set({{113_in, 6.75_in}, rev, 127}, true); //was27,7
  chassis.pid_wait();
  pros::delay(350);

  //move forward into long goal and empty all blocks
  chassis.pid_odom_set({{113_in, 39_in}, fwd, 127}, true);
  chassis.pid_wait_until({113_in, 32_in});
  intakeTop.move(127);
  intakeBottom.move(127);
  chassis.pid_wait();
  // chassis.drive_angle_set(0_deg);
  pros::delay(1600);
  MatchLoadDrop(false);

  //pull out of long goal
  chassis.pid_odom_set({{113_in, 32_in}, rev, 127}, true);
  chassis.pid_wait_quick_chain();
  
  //turn away from wall
  // chassis.pid_turn_set(268_deg, TURN_SPEED); //was-1 //was 0
  // chassis.pid_wait();
  chassis.pid_turn_set({109_in, 32_in}, rev, 127, true);
  chassis.pid_wait_quick_chain();

  //move away from wall
  chassis.pid_odom_set({{109_in, 32_in}, fwd, 127}, true);
  chassis.pid_wait_quick_chain();

  //turn so antenna is in goal
  chassis.pid_turn_set({108_in, 56_in}, rev, 127, true);
  chassis.pid_wait();
  // chassis.pid_turn_set({108_in, 62_in}, rev, 127, true);
  // chassis.pid_wait_quick_chain();

  //push antenna to end of open area
  chassis.pid_odom_set({{108_in, 56_in}, fwd, 110}, true);
  // chassis.pid_wait_quick_chain();

  // //Lift Odom Pod
  // OdomPodLift(true);

  //wait then lift antenna
  // pros::delay(1700);
  // AntennaRaise(true);
}

void sig_solo_awp (){ 
  //set starting angle
  chassis.drive_angle_set(270_deg);
  chassis.odom_xyt_set(95_in, 22_in, 270_deg);

  //start intake
  intakeTop.move(-127);
  intakeBottom.move(127);

  //move towards match loader
  chassis.pid_odom_set({{125.2_in, 22_in}, rev, 127}, true); //was90 speed //was80 speed //was150_deg
  chassis.pid_wait();
    MatchLoadDrop(true);

  //turn to face match loader
  // chassis.pid_turn_set({113_in, 10_in}, rev, 127, true);
  // chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-4_deg, TURN_SPEED); //was 0
  chassis.pid_wait_quick_chain();

  // pros::delay(5);

  //move into match loader and collect 3 blocks
  chassis.pid_odom_set({{126.25_in, 11.35_in}, rev, 100}, true); //was11.2
  chassis.pid_wait();
  pros::delay(75);

  //move straight back into long goal and deposit blocks
  chassis.pid_odom_set({{125.75_in, 39.5_in}, fwd, 127}, true);
  chassis.pid_wait_until(14_in);
  intakeTop.move(127);
  chassis.pid_wait();
  pros::delay(800); //was600

  //pull back from long goal
  MatchLoadDrop(false);
  chassis.pid_odom_set({{125.75_in, 33_in}, rev, 127}, true);
  chassis.pid_wait();

  //turn to face center with intake
  // chassis.pid_turn_set({90_in, 48_in}, rev, 127, true);
  // chassis.pid_wait_quick_chain();
  intakeTop.move(-127);
  chassis.pid_turn_set(125_deg, TURN_SPEED); //was 0
  chassis.pid_wait_quick_chain();


  //go over towards center goal and pick up blocks along the way
  chassis.pid_odom_injected_pp_set({{{96_in, 48.45_in}, rev, 80},  //was48.75
                                    {{54_in, 44.5_in}, rev, 127}}, 
                                    // {{57_in, 45_in}, fwd, 80}}, 
                                    true);
  chassis.pid_wait();
  // chassis.pid_odom_smooth_pp_set({{{96_in, 47_in}, rev, 127},
  //                                 {{52_in, 48_in}, rev, 127},
  //                                 {{58_in, 48_in}, fwd, 127}},
  //                                 true);
  // chassis.pid_wait();
  // chassis.pid_odom_set({{96_in, 48_in, 120_deg}, rev, 127}, true);
  // chassis.pid_wait();
  // chassis.pid_odom_set({{47_in, 46_in}, rev, 127}, true);
  // chassis.pid_wait();

  //turn top of intake towards center goal
  chassis.pid_turn_set(45_deg, TURN_SPEED); //was 0
  chassis.pid_wait();
  // chassis.pid_turn_set({62_in, 62_in}, fwd, 127, true);
  // chassis.pid_wait_quick_chain();

  //move to center goal and empty all blocks
  chassis.pid_odom_set({{61.5_in, 54.25_in}, fwd, 127}, true);
  chassis.pid_wait_until(4_in);
  CenterDrop(true);
  intakeTop.move(-127);
  chassis.pid_wait();
  pros::delay(600);

  //move back towards match loader
  CenterDrop(false);
  intakeTop.move(0);
  chassis.pid_odom_set({{27.0_in, 19_in}, rev, 127}, true); //was25,24
  chassis.pid_wait();
  intakeTop.move(-127);

  //turn to face match loader
  // chassis.pid_turn_set({28_in, 5_in}, rev, 127, true);
  // chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(1_deg, TURN_SPEED); //was-1 //was 0
  chassis.pid_wait_quick_chain();
  MatchLoadDrop(true);

  // //move into match loader and collect 3 blocks
  chassis.pid_odom_set({{28.35_in, 4.15_in}, rev, 70}, true); //was5.2  //was5
  chassis.pid_wait();
  pros::delay(200);

  //move forward into long goal and empty all blocks
  chassis.pid_odom_set({{26.67_in, 39_in}, fwd, 127}, true);
  chassis.pid_wait_until({26.67_in, 30_in});
  intakeTop.move(127);
  // OdomPodLift(true);
  chassis.pid_wait();

  MatchLoadDrop(false);
  // pros::delay(850);

  //Pull out
  // chassis.pid_drive_set(-6.7_in, 127);
  // chassis.pid_wait();
}

void skills_auto() {
  //part 1

  // //lift odom pod and start intake
  // chassis.odom_xyt_set(87_in, 8_in, 90_deg);
  // OdomPodLift(true);
  // BottomIntakeMove(127);
  // AntennaRaise(true);
  // TopIntakeMove(-100);
  // chassis.pid_drive_set(7_in, 127);
  // chassis.pid_wait_quick_chain();


  // //push into park zone
  // chassis.pid_drive_set(-56_in, 127);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-26_in, 90);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-18_in, 50);
  // chassis.pid_wait();

  // //drop odom pod and wall snap
  // OdomPodLift(false);
  // wall_snap(32);
  // pros::delay(500);

  //part 2
  //just for testing 
  chassis.odom_xyt_set(34_in, 8_in, 90_deg);
  BottomIntakeMove(127);
  TopIntakeMove(-100);

  //pull away from loader and go pick up blue ball from stack of 4
  chassis.pid_odom_injected_pp_set({{{46_in, 8_in}, fwd, 110},  //was48.75
                                    {{42_in, 42_in, 245_deg}, rev, 110}}, 
                                    // {{57_in, 45_in}, fwd, 80}}, 
                                    true);
  chassis.pid_wait();

  //turn so top faces center goal
  chassis.pid_turn_set({52_in, 52_in}, fwd, 110, true);
  chassis.pid_wait();

  //move to center goal and intake in all blocks
  chassis.pid_odom_set({{54_in, 50_in}, fwd, 110}, true); //was90 speed //was80 speed //was150_deg
  chassis.pid_wait_until(9_in);
  CenterDrop(true);
  intakeTop.move(-70);
  chassis.pid_wait();
  pros::delay(1500);
  intakeTop.move(-40);
  pros::delay(2000);
  intakeTop.move(-20);
  pros::delay(400);

  //lift center score then move towards match loader
  intakeTop.move(-100);
  CenterDrop(false);
  chassis.pid_odom_set({{16_in, 18_in}, rev, 110}, true); //was90 speed //was80 speed //was150_deg
  chassis.pid_wait();

  //turn to face match loader
  chassis.pid_turn_set({18_in, 25_in}, fwd, 110, true);
  chassis.pid_wait_quick_chain();

  //move back into long goal and reset odom values
  chassis.pid_odom_set({{18_in, 27_in}, fwd, 110}, true);
  chassis.pid_wait();
  chassis.odom_xyt_set(24_in, 43_in, 0_deg);
  pros::delay(25);

  //part 3 
  // //just for testing 
  // chassis.odom_xyt_set(24_in, 43_in, 0_deg);
  // BottomIntakeMove(127);
  // TopIntakeMove(-100);
  // AntennaRaise(true);

  //drop match loader and move into match loader
  MatchLoadDrop(true);
  chassis.pid_odom_set({{24.1_in, 10.5_in}, rev, 127}, true);
  chassis.pid_wait_until(20_in);
  chassis.pid_speed_max_set(50);
  chassis.pid_wait();
  pros::delay(100);

  //pull out, turn towards wall, move towards wall, and align with wall
  chassis.pid_odom_injected_pp_set({{{16_in, 20_in}, fwd, 100},  //was48.75
                                    {{10_in, 44_in, 0_deg}, fwd, 100}}, 
                                    // {{57_in, 45_in}, fwd, 80}}, 
                                    true);
  chassis.pid_wait();

  //stop intake
  BottomIntakeMove(0);
  TopIntakeMove(0);


  //lift match load and drive to other side
  MatchLoadDrop(false);
  chassis.pid_odom_set({{10.1_in, 115_in}, fwd, 127}, true);
  chassis.pid_wait();



}

void random_testing (){
  //random wall ride tests
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  // wall_drive_y(100, 127);

  // //Set starting angle
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  // // wall drive-09
  // wall_drive_to_point(0, 110.0, 127);

  // //Set starting angle
  // chassis.odom_xyt_set(0_in, 0_in, 90_deg);
  // // wall drive
  // wall_drive_to_point(110, 0, 127);

  // //turn
  // turn_to_angle(95, 110);

   // chassis.pid_swing_set(ez::RIGHT_SWING, 120, -110);
  // chassis.pid_wait();

  // chassis.pid_odom_set({{96_in, 96_in}, rev, 110}, true); //was90 speed //was80 speed //was150_deg
  // chassis.pid_wait();


  //jiggle the robot around while intaking
  // chassis.pid_swing_set(ez::RIGHT_SWING, 15, 110);
  // chassis.pid_wait();
  // chassis.pid_swing_set(ez::LEFT_SWING, -15, 110);
  // chassis.pid_wait();
  // chassis.pid_swing_set(ez::LEFT_SWING, 0, 110);
  // chassis.pid_wait();
  // drive_set(127, 90);
  // pros::delay(50);
  // drive_set(90, 127);
  // pros::delay(50);
  // drive_set(127, 90);
  // pros::delay(50);
  // drive_set(90, 127);
  // pros::delay(50);
  // drive_set(0, 0);

  // chassis.pid_turn_set(20_deg, 110);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-20_deg, 110);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(20_deg, 110);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(-20_deg, 110);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(0_deg, 110);
  // chassis.pid_wait_quick_chain();

  // //pull out of park zone
  // chassis.pid_drive_set(20_in, 127); //was-4
  // chassis.pid_wait();

  //drop odom pod


  // //turn so right faces the wall
  // chassis.pid_turn_set(270_deg, 110);
  // chassis.pid_wait();
  // pros::delay(10);

  // //run snap test
  // wall_snap(72);
  // print_odom();

  // chassis.pid_turn_set(90_deg, 110);

  // // go to 4 blocks
  // chassis.pid_odom_set({{96_in, 96_in}, rev, 110}, true); //was90 speed //was80 speed //was150_deg
  // chassis.pid_wait();

  // //turn to face center goal
  // chassis.pid_turn_set(235_deg, 110);
  // chassis.pid_wait();

  // //move to center goal and intake in all blocks
  // chassis.pid_odom_set({{82_in, 82_in}, rev, 110}, true); //was90 speed //was80 speed //was150_deg
  // chassis.pid_wait_until(4_in);
  // CenterDrop(true);
  // intakeTop.move(-110);
  // chassis.pid_wait();
  // pros::delay(1400);



// void drive_set(int left, int right);


  ////////park zone push test
  //lift odom pod and start intake
  chassis.odom_xyt_set(87_in, 8_in, 90_deg);
  OdomPodLift(true);
  BottomIntakeMove(127);
  TopIntakeMove(-127);
  chassis.pid_drive_set(7_in, 127);
  chassis.pid_wait_quick_chain();


  //push into park zone
  chassis.pid_drive_set(-56_in, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-26_in, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-18_in, 50);
  chassis.pid_wait();

  //drop odom pod and wall snap
  OdomPodLift(false);
  wall_snap(32);
  pros::delay(500);

  //pull away from loader
  chassis.pid_drive_set(3_in, 110);
  chassis.pid_wait();





}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {

  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .