#include "main.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Direct motor control for playback - separate from chassis
pros::Motor left_motor1(-18);
pros::Motor left_motor2(-19);
pros::Motor left_motor3(-20);
pros::Motor right_motor1(11);
pros::Motor right_motor2(12);
pros::Motor right_motor3(13);

// Direct inertial sensor access (port 2)
pros::Imu inertial_sensor(2);

struct SkillsDataPoint {
    double timestamp;
    double imu_heading;
    double imu_pitch;
    double imu_roll;
    double gyro_rate;
    double odom_x;
    double odom_y;
    int left_drive;
    int right_drive;
    int intake_top;
    int intake_bottom;
    bool center_piston;
    bool match_load_piston;
    bool odom_pod_piston;
    int distance_front;
    int distance_left;
    int distance_right;
};

std::vector<SkillsDataPoint> recordedSkillsData;
bool recordingSkills = false;
double skillsRecordingStartTime = 0;
bool playingBackSkills = false;
int playbackSkillsIndex = 0;
double playbackSkillsStartTime = 0;

const int SAMPLE_RATE_MS = 10; // INCREASED to 10ms (100Hz) to reduce noise/jitter
double lastRecordTime = 0;

// ===== SMOOTHER PID TUNING PARAMETERS =====

// Heading PID (Reduced kP, Increased kD for smoothness)
const double HEADING_kP = 0.5;    // Was 2.0 - Lower = less jerky
const double HEADING_kI = 0.00;   // Keep I low/zero to prevent oscillation
const double HEADING_kD = 4.5;    // Increased D to dampen/smooth movements
const double MAX_HEADING_CORRECTION = 27.5; // Reduced max correction

// Distance PID (Reduced significantly)
const double DISTANCE_kP = 1.0;   // Was 3.0 - Lower = smoother speed changes
const double DISTANCE_kI = 0.00;
const double DISTANCE_kD = 2.5;
const double MAX_DISTANCE_CORRECTION = 25.0; // Reduced max correction

// Distance sensor PID (Reduced to be subtle hints)
const double FRONT_DIST_kP = 0.5;
const double SIDE_DIST_kP = 0.0;
const double MAX_DIST_SENSOR_CORRECTION = 10.0;
const int DIST_SENSOR_THRESHOLD = 400; // Only look at walls closer than 400mm

// DEADBANDS (Ignore small errors to stop shivering)
const double HEADING_DEADBAND = 4.0; // Ignore errors < 1 degree
const double DISTANCE_DEADBAND = 1.0; // Ignore errors < 1 inch

// PID state variables
double heading_integral = 0;
double heading_prev_error = 0;
double distance_integral = 0;
double distance_prev_error = 0;
double front_dist_integral = 0;
double front_dist_prev_error = 0;
double left_dist_integral = 0;
double left_dist_prev_error = 0;
double right_dist_integral = 0;
double right_dist_prev_error = 0;

const double MAX_INTEGRAL = 30.0;

ez::Drive chassis(
    {-18, -19, -20},
    {11, 12, 13},
    2, 3.25, 450
);

ez::tracking_wheel horiz_tracker(14, 2, 1.25);
ez::tracking_wheel vert_tracker(-7, 2, 1.5);

// Helper function to get average voltage from left motors
int getLeftMotorVoltage() {
    return (left_motor1.get_voltage() + left_motor2.get_voltage() + left_motor3.get_voltage()) / 3;
}

// Helper function to get average voltage from right motors
int getRightMotorVoltage() {
    return (right_motor1.get_voltage() + right_motor2.get_voltage() + right_motor3.get_voltage()) / 3;
}

// Helper function to set all left motors
void setLeftMotors(int value) {
    left_motor1.move(value);
    left_motor2.move(value);
    left_motor3.move(value);
}

// Helper function to set all right motors
void setRightMotors(int value) {
    right_motor1.move(value);
    right_motor2.move(value);
    right_motor3.move(value);
}

// PID calculation helper
double calculatePID(double error, double &integral, double &prev_error, 
                    double kP, double kI, double kD, double dt) {
    // Proportional
    double P = error * kP;
    
    // Integral with anti-windup
    integral += error * dt;
    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;
    double I = integral * kI;
    
    // Derivative
    double derivative = (error - prev_error) / dt;
    double D = derivative * kD;
    prev_error = error;
    
    return P + I + D;
}

// Reset all PID state
void resetPIDState() {
    heading_integral = 0; heading_prev_error = 0;
    distance_integral = 0; distance_prev_error = 0;
    front_dist_integral = 0; front_dist_prev_error = 0;
    left_dist_integral = 0; left_dist_prev_error = 0;
    right_dist_integral = 0; right_dist_prev_error = 0;
}

std::string skillsDataPointToCSV(const SkillsDataPoint& point) {
    std::stringstream ss;
    ss << point.timestamp << "," 
       << point.imu_heading << "," 
       << point.imu_pitch << "," 
       << point.imu_roll << ","
       << point.gyro_rate << ","
       << point.odom_x << "," 
       << point.odom_y << ","
       << point.left_drive << "," 
       << point.right_drive << "," 
       << point.intake_top << "," 
       << point.intake_bottom << ","
       << point.center_piston << "," 
       << point.match_load_piston << "," 
       << point.odom_pod_piston << ","
       << point.distance_front << ","
       << point.distance_left << ","
       << point.distance_right;
    return ss.str();
}

SkillsDataPoint skillsCSVLineToDataPoint(const std::string& line) {
    SkillsDataPoint point = {};
    std::stringstream ss(line);
    std::string token;
    
    std::getline(ss, token, ','); point.timestamp = std::stod(token);
    std::getline(ss, token, ','); point.imu_heading = std::stod(token);
    std::getline(ss, token, ','); point.imu_pitch = std::stod(token);
    std::getline(ss, token, ','); point.imu_roll = std::stod(token);
    std::getline(ss, token, ','); point.gyro_rate = std::stod(token);
    std::getline(ss, token, ','); point.odom_x = std::stod(token);
    std::getline(ss, token, ','); point.odom_y = std::stod(token);
    std::getline(ss, token, ','); point.left_drive = std::stoi(token);
    std::getline(ss, token, ','); point.right_drive = std::stoi(token);
    std::getline(ss, token, ','); point.intake_top = std::stoi(token);
    std::getline(ss, token, ','); point.intake_bottom = std::stoi(token);
    std::getline(ss, token, ','); point.center_piston = std::stoi(token);
    std::getline(ss, token, ','); point.match_load_piston = std::stoi(token);
    std::getline(ss, token, ','); point.odom_pod_piston = std::stoi(token);
    std::getline(ss, token, ','); point.distance_front = std::stoi(token);
    std::getline(ss, token, ','); point.distance_left = std::stoi(token);
    std::getline(ss, token, ','); point.distance_right = std::stoi(token);
    
    return point;
}

void recordSkillsDataPoint() {
    if (!recordingSkills) return;
    
    double currentTime = pros::millis();
    if (currentTime - lastRecordTime < SAMPLE_RATE_MS) return;
    lastRecordTime = currentTime;
    
    SkillsDataPoint point;
    point.timestamp = currentTime - skillsRecordingStartTime;
    
    point.imu_heading = inertial_sensor.get_heading();
    point.imu_pitch = inertial_sensor.get_pitch();
    point.imu_roll = inertial_sensor.get_roll();
    point.gyro_rate = inertial_sensor.get_gyro_rate().z;
    
    point.odom_x = chassis.odom_x_get();
    point.odom_y = chassis.odom_y_get();
    
    point.left_drive = getLeftMotorVoltage() / 94;
    point.right_drive = getRightMotorVoltage() / 94;
    
    point.intake_top = intakeTop.get_target_velocity();
    point.intake_bottom = intakeBottom.get_target_velocity();
    point.center_piston = center.get();
    point.match_load_piston = matchLoad.get();
    point.odom_pod_piston = odomPod.get();
    
    point.distance_front = frontDist.get();
    point.distance_left = leftDist.get();
    point.distance_right = rightDist.get();
    
    recordedSkillsData.push_back(point);
}

bool saveSkillsRecording(const char* filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    file << "timestamp,imu_heading,imu_pitch,imu_roll,gyro_rate,odom_x,odom_y,left_drive,right_drive,intake_top,intake_bottom,center_piston,match_load_piston,odom_pod_piston,distance_front,distance_left,distance_right\n";
    for (const auto& point : recordedSkillsData) {
        file << skillsDataPointToCSV(point) << "\n";
    }
    file.close();
    printf("Saved %zu points\n", recordedSkillsData.size());
    return true;
}

bool loadSkillsRecording(const char* filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    
    recordedSkillsData.clear();
    std::string line;
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        recordedSkillsData.push_back(skillsCSVLineToDataPoint(line));
    }
    file.close();
    printf("Loaded %zu points\n", recordedSkillsData.size());
    return true;
}

void startSkillsRecording() {
    recordingSkills = true;
    skillsRecordingStartTime = pros::millis();
    lastRecordTime = 0;
    recordedSkillsData.clear();
    recordedSkillsData.reserve(15000);
    inertial_sensor.set_heading(0);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    printf("Recording started\n");
}

void stopSkillsRecording() {
    recordingSkills = false;
    printf("Recording stopped\n");
}

void startSkillsPlayback() {
    playingBackSkills = true;
    playbackSkillsIndex = 0;
    playbackSkillsStartTime = pros::millis();
    inertial_sensor.set_heading(0);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    resetPIDState();
    printf("Playback started\n");
}

void stopSkillsPlayback() {
    playingBackSkills = false;
    setLeftMotors(0);
    setRightMotors(0);
    intakeTop.move(0);
    intakeBottom.move(0);
    printf("Playback stopped\n");
}

void updateSkillsPlayback() {
    if (!playingBackSkills || recordedSkillsData.empty()) return;
    
    double elapsedTime = pros::millis() - playbackSkillsStartTime;
    
    while (playbackSkillsIndex < recordedSkillsData.size() - 1 &&
           recordedSkillsData[playbackSkillsIndex + 1].timestamp <= elapsedTime) {
        playbackSkillsIndex++;
    }
    
    if (playbackSkillsIndex >= recordedSkillsData.size() - 1) {
        stopSkillsPlayback();
        return;
    }
    
    const SkillsDataPoint& target = recordedSkillsData[playbackSkillsIndex];
    double dt = SAMPLE_RATE_MS / 1000.0;
    
    // ===== HEADING PID (With Deadband) =====
    double current_heading = inertial_sensor.get_heading();
    double heading_error = target.imu_heading - current_heading;
    while (heading_error > 180) heading_error -= 360;
    while (heading_error < -180) heading_error += 360;
    
    if (std::abs(heading_error) < HEADING_DEADBAND) heading_error = 0;

    double heading_correction = calculatePID(heading_error, heading_integral, heading_prev_error,
                                              HEADING_kP, HEADING_kI, HEADING_kD, dt);
    
    if (heading_correction > MAX_HEADING_CORRECTION) heading_correction = MAX_HEADING_CORRECTION;
    if (heading_correction < -MAX_HEADING_CORRECTION) heading_correction = -MAX_HEADING_CORRECTION;
    
    // ===== DISTANCE PID (With Deadband) =====
    double current_x = chassis.odom_x_get();
    double current_y = chassis.odom_y_get();
    double error_x = target.odom_x - current_x;
    double error_y = target.odom_y - current_y;
    double distance_error = std::sqrt(error_x * error_x + error_y * error_y);
    
    // Direction calculation
    double heading_rad = current_heading * M_PI / 180.0;
    double dot = error_x * std::cos(heading_rad) + error_y * std::sin(heading_rad);
    if (dot < 0) distance_error = -distance_error;
    
    if (std::abs(distance_error) < DISTANCE_DEADBAND) distance_error = 0;

    double distance_correction = calculatePID(distance_error, distance_integral, distance_prev_error,
                                               DISTANCE_kP, DISTANCE_kI, DISTANCE_kD, dt);
    
    if (distance_correction > MAX_DISTANCE_CORRECTION) distance_correction = MAX_DISTANCE_CORRECTION;
    if (distance_correction < -MAX_DISTANCE_CORRECTION) distance_correction = -MAX_DISTANCE_CORRECTION;
    
    // ===== DISTANCE SENSOR PID =====
    double dist_speed_corr = 0;
    double dist_head_corr = 0;
    
    int current_front_dist = frontDist.get();
    if (current_front_dist < DIST_SENSOR_THRESHOLD && target.distance_front < DIST_SENSOR_THRESHOLD) {
        double error = target.distance_front - current_front_dist;
        if (std::abs(error) > 20) {
             dist_speed_corr = calculatePID(error/10.0, front_dist_integral, front_dist_prev_error, 
                                            FRONT_DIST_kP, 0, 0.1, dt);
        }
    }
    
    if (dist_speed_corr > MAX_DIST_SENSOR_CORRECTION) dist_speed_corr = MAX_DIST_SENSOR_CORRECTION;
    if (dist_speed_corr < -MAX_DIST_SENSOR_CORRECTION) dist_speed_corr = -MAX_DIST_SENSOR_CORRECTION;

    // ===== APPLY =====
    double total_heading_correction = heading_correction + dist_head_corr;
    double total_speed_correction = distance_correction + dist_speed_corr;
    
    int left_output = target.left_drive + total_heading_correction + total_speed_correction;
    int right_output = target.right_drive - total_heading_correction + total_speed_correction;
    
    if (left_output > 127) left_output = 127;
    if (left_output < -127) left_output = -127;
    if (right_output > 127) right_output = 127;
    if (right_output < -127) right_output = -127;
    
    setLeftMotors(left_output);
    setRightMotors(right_output);
    
    intakeTop.move(target.intake_top);
    intakeBottom.move(target.intake_bottom);
    center.set(target.center_piston);
    matchLoad.set(target.match_load_piston);
    odomPod.set(target.odom_pod_piston);
}

void initialize() {
  // SD Card Fix: Delay to allow SD card to mount
  pros::delay(2000); 

  ez::ez_template_print();
  pros::delay(500);

  intakeTop.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intakeBottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intakeTop.set_current_limit(2500);
  intakeBottom.set_current_limit(2500);
  
  chassis.odom_tracker_back_set(&horiz_tracker);
  chassis.odom_tracker_right_set(&vert_tracker);

  chassis.opcontrol_curve_buttons_toggle(true);
  chassis.opcontrol_drive_activebrake_set(0.0);

  default_constants();

  ez::as::auton_selector.autons_add({
    
      {"Right Antenna Auto", right_antenna_auto},
      {"Generated skills auto", generated_skills_auto},
      {"Middle Goal Antenna Auto for LEFT Side", elims_left_auto},
      {"Sig SOLO AWP", sig_solo_awp},
      {"Right Antenna Auto", right_antenna_auto},
      {"Left Antenna Auto", left_antenna_auto},
      {"Skills Auto", skills_auto},
      {"Random Testing", random_testing},
      {"Drive Example", drive_example},
      {"Turn Example", turn_example},
      {"Drive and Turn", drive_and_turn},
      {"Wait Until Change Speed", wait_until_change_speed},
      {"Swing Example", swing_example},
      {"Motion Chaining", motion_chaining},
      {"Combining Movements", combining_movements},
      {"Interfered Example", interfered_example},
      {"Odom Drive Example", odom_drive_example},
      {"Pure Pursuit", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until", odom_pure_pursuit_wait_until_example},
      {"Boomerang", odom_boomerang_example},
      {"Boomerang Pure Pursuit", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets", measure_offsets},
  });

  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  ez::as::auton_selector.selected_auton_call();
}

void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());
  }
  ez::screen_print(tracker_value + tracker_width, line);
}

void ez_screen_task() {
  while (true) {
    if (!pros::competition::is_connected()) {
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        if (ez::as::page_blank_is_on(0)) {
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()), 1);
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task ezScreenTask(ez_screen_task);

void ez_template_extras() {
}

void opcontrol() {

  //raise antenna for skills
  // AntennaRaise(true);
  OdomPodLift(true);
  pros::delay(500);
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  bool recordingMode = false;
  bool playbackMode = false;

  while (true) {
    ez_template_extras();

    if (master.get_digital_new_press(DIGITAL_X)) {
      if (!recordingMode && !playbackMode) {
        recordingMode = true;
        startSkillsRecording();
        printf("Press Y to stop and save\n");
      }
    }
    
    if (master.get_digital_new_press(DIGITAL_Y)) {
      if (recordingMode) {
        recordingMode = false;
        stopSkillsRecording();
        saveSkillsRecording("/usd/skills_recording.csv");
      }
    }

    if (master.get_digital_new_press(DIGITAL_A)) {
      if (!playbackMode && !recordingMode) {
        if (loadSkillsRecording("/usd/skills_recording.csv")) {
          playbackMode = true;
          startSkillsPlayback();
          printf("Press DOWN to stop playback\n");
        }
      }
    }

    if (!playbackMode) {
      chassis.opcontrol_tank();
      IntakeControl();
      OdomPodControl();
      AntennaControl();
      MatchLoadControl();
      CenterControl();
      ParkControl();
      CenterDescoreControl();

      if (recordingMode) {
        recordSkillsDataPoint();
      }
    } 
    else {
      updateSkillsPlayback();
      
      if (master.get_digital_new_press(DIGITAL_DOWN)) {
        playbackMode = false;
        stopSkillsPlayback();
        printf("Playback stopped\n");
      }
      
      if (!playingBackSkills) {
        playbackMode = false;
      }
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}