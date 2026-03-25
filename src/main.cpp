#include "main.h"
#include <fstream>
#include <sstream>
#include <vector>

pros::Controller master(pros::E_CONTROLLER_MASTER);

//individual data points from sensors, motors, pistons
struct SkillsDataPoint {
    double timestamp;
    double rotation;
    double odom_x;
    double odom_y;
    int left_drive;
    int right_drive;
    int intake_top;
    int intake_bottom;
    bool center_piston;
    bool match_load_piston;
    bool odom_pod_piston;
    bool odom_reset;
};

//define values for functiosn 
std::vector<SkillsDataPoint> recordedSkillsData;
bool recordingSkills = false;
double skillsRecordingStartTime = 0;
bool playingBackSkills = false;
int playbackSkillsIndex = 0;
double playbackSkillsStartTime = 0;

//how often its samppled
const int SAMPLE_RATE_MS = 3; //was5 ms //was3 ms

//normal motor declarations
ez::Drive chassis(
    {-18, -19, -20},
    {11, 12, 13},
    1, 3.25, 450
);

//odom wheel declarations
ez::tracking_wheel horiz_tracker(-17, 2, -0.25);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(7, 2, -1.25);   // This tracking wheel is parallel to the drive wheels

//turns the data points listed above into a CSV file that can be reread later on
std::string skillsDataPointToCSV(const SkillsDataPoint& point) {
    std::stringstream ss;
    ss << point.timestamp << "," << point.rotation << "," << point.odom_x << "," << point.odom_y << ","
       << point.left_drive << "," << point.right_drive << "," << point.intake_top << "," << point.intake_bottom << ","
       << point.center_piston << "," << point.match_load_piston << "," << point.odom_pod_piston << "," << point.odom_reset;
    return ss.str();
}

//formatting for the CSV file
SkillsDataPoint skillsCSVLineToDataPoint(const std::string& line) {
    SkillsDataPoint point = {};
    std::stringstream ss(line);
    std::string token;
    
    std::getline(ss, token, ','); point.timestamp = std::stod(token);
    std::getline(ss, token, ','); point.rotation = std::stod(token);
    std::getline(ss, token, ','); point.odom_x = std::stod(token);
    std::getline(ss, token, ','); point.odom_y = std::stod(token);
    std::getline(ss, token, ','); point.left_drive = std::stoi(token);
    std::getline(ss, token, ','); point.right_drive = std::stoi(token);
    std::getline(ss, token, ','); point.intake_top = std::stoi(token);
    std::getline(ss, token, ','); point.intake_bottom = std::stoi(token);
    std::getline(ss, token, ','); point.center_piston = std::stoi(token);
    std::getline(ss, token, ','); point.match_load_piston = std::stoi(token);
    std::getline(ss, token, ','); point.odom_pod_piston = std::stoi(token);
    std::getline(ss, token, ','); point.odom_reset = std::stoi(token);
    
    return point;
}

double lastRecordTime = 0;

//function to record inputs from sensors, pistons, motors, etc
void recordSkillsDataPoint() {
    if (!recordingSkills) return;
    
    double currentTime = pros::millis();
    
    //Records only when the sample rate time has past
    if (currentTime - lastRecordTime < SAMPLE_RATE_MS) return;
    lastRecordTime = currentTime;
    
    SkillsDataPoint point;
    point.timestamp = currentTime - skillsRecordingStartTime;
    point.rotation = chassis.odom_theta_get();
    point.odom_x = chassis.odom_x_get();
    point.odom_y = chassis.odom_y_get();
    point.left_drive = master.get_analog(ANALOG_LEFT_Y);
    point.right_drive = master.get_analog(ANALOG_RIGHT_Y);
    point.intake_top = intakeTop.get_target_velocity();
    point.intake_bottom = intakeBottom.get_target_velocity();
    point.center_piston = center.get();
    point.match_load_piston = matchLoad.get();
    point.odom_pod_piston = odomPod.get();
    point.odom_reset = 1;
    
    recordedSkillsData.push_back(point);
}

//function to save the recorded values into the previously setup CSV file
bool saveSkillsRecording(const char* filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    file << "timestamp,rotation,odom_x,odom_y,left_drive,right_drive,intake_top,intake_bottom,center_piston,match_load_piston,odom_pod_piston, odom_reset\n";
    for (const auto& point : recordedSkillsData) {
        file << skillsDataPointToCSV(point) << "\n";
    }
    file.close();
    printf("Saved %zu points at %d Hz\n", recordedSkillsData.size(), 1000/SAMPLE_RATE_MS);
    return true;
}

//calls the CSV file in the SD card
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

// FUnctyion to start recording playback
void startSkillsRecording() {
    recordingSkills = true;
    skillsRecordingStartTime = pros::millis();
    lastRecordTime = 0;
    recordedSkillsData.clear();
    recordedSkillsData.reserve(15000);  // Pre-allocate for 60 sec at 200 Hz
    printf("Recording started at %d Hz\n", 1000/SAMPLE_RATE_MS);
}

// function to stop recording playback
void stopSkillsRecording() {
    recordingSkills = false;
    printf("Recording stopped - %zu points\n", recordedSkillsData.size());
}

//function to start playing playback
void startSkillsPlayback() {
    playingBackSkills = true;
    playbackSkillsIndex = 0;
    playbackSkillsStartTime = pros::millis();
    printf("Playback started - %zu points\n", recordedSkillsData.size());
}

//function to stop playing playback
void stopSkillsPlayback() {
    playingBackSkills = false;
    chassis.drive_set(0, 0);
    intakeTop.move(0);
    intakeBottom.move(0);
    printf("Playback stopped\n");
}

//plays the values in the CSV file
void playSkillsPlayback() {
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
    
    //motor and piston values
    chassis.drive_set(target.left_drive, target.right_drive);
    intakeTop.move(target.intake_top);
    intakeBottom.move(target.intake_bottom);
    center.set(target.center_piston);
    matchLoad.set(target.match_load_piston);
    odomPod.set(target.odom_pod_piston);
}

//normal initialize function
void initialize() {
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
  chassis.opcontrol_curve_default_set(0.0);

  default_constants();

  // Autonomous Selector
  ez::as::auton_selector.autons_add({
      {"Playback auto testing", generated_skills_auto},
      // {"Skills Auto", skills_auto},
      
      {"Sig SOLO AWP\n\n(4 + 3 + 4)", sig_solo_awp},
      {"Antenna push auto for RIGHT side\n\n(7 + antenna)", right_antenna_auto},
      {"Skills Auto", skills_auto},


      // {"PUSH sig SOLO AWP \n\n(push + 5 + 3 + 3)", push_solo_awp},
      {"Middle Goal Antenna Auto for LEFT Side\n\n(4 + 3 + antenna)", elims_left_auto},
      {"Antenna push auto for RIGHT side\n\n(7 + antenna)", right_antenna_auto},
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

  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

void disabled() {}

void competition_initialize() {}

//auto function
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

//driver control with built in playback buttons for starting, stopping, etc
void opcontrol() {
  // OdomPodLift(true);
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

    if (master.get_digital_new_press(DIGITAL_B)) {
      if (recordingMode) {
        
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
      playSkillsPlayback();
      
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