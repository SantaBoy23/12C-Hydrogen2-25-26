#include "main.h"

ez::Drive chassis(
    {-18, -19, -20},    // Left Chassis Ports
    {11, 12, 13}, // Right Chassis Ports
    2, 3.25, 450  // IMU Port, Wheel Diameter (in), Wheel RPM
);

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
ez::tracking_wheel horiz_tracker(-17, 2, -0.25);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(7, 2, -1.25);   // This tracking wheel is parallel to the drive wheels

void initialize() {
  ez::ez_template_print();  // Print EZ-Template branding

  pros::delay(1500);  // Allow legacy ports to initialize

  // Configure intake motors
  intakeTop.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intakeBottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intakeTop.set_current_limit(2500);
  intakeBottom.set_current_limit(2500);
  
  chassis.odom_tracker_back_set(&horiz_tracker);
  chassis.odom_tracker_right_set(&vert_tracker);

  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  //chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  default_constants();  // Set the drivetrain constants from autons.cpp

  // Autonomous Selector
  ez::as::auton_selector.autons_add({
      // {"Random Testing", random_testing},

      {"Skills Auto", skills_auto},

      {"PUSH sig SOLO AWP \n\n(push + 5 + 3 + 3)", push_solo_awp},

      // {"PUSH sig SOLO AWP \n\n(push + 5 + 3 + 3)", push_solo_awp},
      {"Middle Goal Antenna Auto for LEFT Side\n\n(4 + 3 + antenna)", elims_left_auto},
      {"Antenna push auto for RIGHT side\n\n(7 + antenna)", right_antenna_auto},
      {"4 block", block_rush_right},
      {"Skills Auto", skills_auto},

      {"Sig SOLO AWP\n\n(4 + 3 + 4)", sig_solo_awp},
      {"PUSH sig SOLO AWP \n\n(push + 5 + 3 + 3)", push_solo_awp},

      {"Middle Goal Antenna Auto for RIGHT Side\n\n(4 + 3 + antenna)", elims_right_auto},
      {"Antenna push auto for LEFT side\n\n(7 + antenna)", left_antenna_auto},

      {"Random Testing", random_testing},
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

//Simplifies printing tracker values to the brain screen
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

// EZ-Template screen task
void ez_screen_task() {
  while (true) {
    // Only runs this when not connected to a comp switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task ezScreenTask(ez_screen_task);  // Run EZ-Template screen task

void ez_template_extras() {
  // Only runs this when not connected to a competition switch
  if (!pros::competition::is_connected()) {

    // Enable / Disable PID Tuner
    // When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X)){
      chassis.pid_tuner_toggle();
      chassis.pid_tuner_full_enable(true);
    }

    // Trigger the selected autonomous routine using B and DOWN
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

// Driver Control
void opcontrol() {



  // pros::delay(500);  // Allow ports to initialize

  // AntennaRaise(true); //add for skills
  // OdomPodLift(true);

  chassis.drive_brake_set(MOTOR_BRAKE_COAST); // Switch motor brakes to coast

  while (true) {
    ez_template_extras(); // Built in EZ-Template extras

    chassis.opcontrol_tank();  // Tank control
    IntakeControl();
    OdomPodControl();
    AntennaControl();
    MatchLoadControl();
    ParkControl();
    CenterDescoreControl();

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
