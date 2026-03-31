#include "main.h"
#include "playback.hpp"

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

//how often its sampled
const int SAMPLE_RATE_MS = 3; //was5 ms //was3 ms

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

void playbackButtonControls(){
    bool recordingMode = false;
    bool playbackMode = false;

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
