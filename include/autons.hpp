#pragma once

void default_constants();

inline ez::Piston odomPod('A');

void OdomPodLift(bool OdomPodState);
void OdomPodControl();

void left_counter_auto();
void SAFE_left_counter_auto();
void push_solo_awp ();
void push_alt_solo_awp ();
void sig_solo_awp ();
void elims_left_auto();
void elims_right_auto();
void right_antenna_auto();
void left_antenna_auto();
void skills_auto();
void four_block_right();
void nine_block_left();
void nine_block_right();
void two_inch_auto();

void random_testing();
void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();


//function to move each half of the drivetrain independently
void drive_set(int left, int right);