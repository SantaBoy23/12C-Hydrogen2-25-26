// #include "main.h"
// #include "playback.hpp"
// #include "autons.hpp"
// #include "drivetrain.hpp"

// struct SkillsDataPoint {
//     double timestamp;
//     double rotation;
//     double odom_x;
//     double odom_y;
//     int left_drive;
//     int right_drive;
//     int intake_top;
//     int intake_bottom;
//     bool center_piston;
//     bool match_load_piston;
//     bool odom_pod_piston;
// };

// std::vector<SkillsDataPoint> recordedSkillsData;
// bool recordingSkills = false;
// double skillsRecordingStartTime = 0;
// bool playingBackSkills = false;
// int playbackSkillsIndex = 0;
// double playbackSkillsStartTime = 0;



// std::string skillsDataPointToCSV(const SkillsDataPoint& point) {
//     std::stringstream ss;
//     ss << point.timestamp << "," << point.rotation << "," << point.odom_x << "," << point.odom_y << ","
//        << point.left_drive << "," << point.right_drive << "," << point.intake_top << "," << point.intake_bottom << ","
//        << point.center_piston << "," << point.match_load_piston << "," << point.odom_pod_piston;
//     return ss.str();
// }

// SkillsDataPoint skillsCSVLineToDataPoint(const std::string& line) {
//     SkillsDataPoint point = {};
//     std::stringstream ss(line);
//     std::string token;
    
//     std::getline(ss, token, ','); point.timestamp = std::stod(token);
//     std::getline(ss, token, ','); point.rotation = std::stod(token);
//     std::getline(ss, token, ','); point.odom_x = std::stod(token);
//     std::getline(ss, token, ','); point.odom_y = std::stod(token);
//     std::getline(ss, token, ','); point.left_drive = std::stoi(token);
//     std::getline(ss, token, ','); point.right_drive = std::stoi(token);
//     std::getline(ss, token, ','); point.intake_top = std::stoi(token);
//     std::getline(ss, token, ','); point.intake_bottom = std::stoi(token);
//     std::getline(ss, token, ','); point.center_piston = std::stoi(token);
//     std::getline(ss, token, ','); point.match_load_piston = std::stoi(token);
//     std::getline(ss, token, ','); point.odom_pod_piston = std::stoi(token);
    
//     return point;
// }

// void recordSkillsDataPoint() {
//     if (!recordingSkills) return;
    
//     SkillsDataPoint point;
//     point.timestamp = pros::millis() - skillsRecordingStartTime;
//     point.rotation = chassis.odom_theta_get();
//     point.odom_x = chassis.odom_x_get();
//     point.odom_y = chassis.odom_y_get();
//     point.left_drive = master.get_analog(ANALOG_LEFT_Y);
//     point.right_drive = master.get_analog(ANALOG_RIGHT_Y);
//     point.intake_top = intakeTop.get_target_velocity();
//     point.intake_bottom = intakeBottom.get_target_velocity();
//     point.center_piston = center.get();
//     point.match_load_piston = matchLoad.get();
//     point.odom_pod_piston = odomPod.get();
    
//     recordedSkillsData.push_back(point);
// }

// bool saveSkillsRecording(const char* filename) {
//     std::ofstream file(filename);
//     if (!file.is_open()) return false;
    
//     file << "timestamp,rotation,odom_x,odom_y,left_drive,right_drive,intake_top,intake_bottom,center_piston,match_load_piston,odom_pod_piston\n";
//     for (const auto& point : recordedSkillsData) {
//         file << skillsDataPointToCSV(point) << "\n";
//     }
//     file.close();
//     printf("Saved %zu points\n", recordedSkillsData.size());
//     return true;
// }

// bool loadSkillsRecording(const char* filename) {
//     std::ifstream file(filename);
//     if (!file.is_open()) return false;
    
//     recordedSkillsData.clear();
//     std::string line;
//     std::getline(file, line);
    
//     while (std::getline(file, line)) {
//         if (line.empty()) continue;
//         recordedSkillsData.push_back(skillsCSVLineToDataPoint(line));
//     }
//     file.close();
//     printf("Loaded %zu points\n", recordedSkillsData.size());
//     return true;
// }

// void startSkillsRecording() {
//     recordingSkills = true;
//     skillsRecordingStartTime = pros::millis();
//     recordedSkillsData.clear();
//     printf("Recording started\n");
// }

// void stopSkillsRecording() {
//     recordingSkills = false;
//     printf("Recording stopped\n");
// }

// void startSkillsPlayback() {
//     playingBackSkills = true;
//     playbackSkillsIndex = 0;
//     playbackSkillsStartTime = pros::millis();
//     printf("Playback started\n");
// }

// void stopSkillsPlayback() {
//     playingBackSkills = false;
//     printf("Playback stopped\n");
// }


// void updateSkillsPlayback() {
//     if (!playingBackSkills || recordedSkillsData.empty()) return;
    
//     double elapsedTime = pros::millis() - playbackSkillsStartTime;
    
//     while (playbackSkillsIndex < recordedSkillsData.size() - 1 &&
//            recordedSkillsData[playbackSkillsIndex + 1].timestamp <= elapsedTime) {
//         playbackSkillsIndex++;
//     }
    
//     if (playbackSkillsIndex >= recordedSkillsData.size() - 1) {
//         stopSkillsPlayback();
//         return;
//     }
    
//     const SkillsDataPoint& target = recordedSkillsData[playbackSkillsIndex];
    
//     // Get current position
//     double current_x = chassis.odom_x_get();
//     double current_y = chassis.odom_y_get();
//     double current_theta = chassis.odom_theta_get();
    
//     // Calculate error
//     double error_x = target.odom_x - current_x;
//     double error_y = target.odom_y - current_y;
//     double error_dist = std::sqrt(error_x * error_x + error_y * error_y);
    
//     // Small correction: if drifting more than 2 inches, apply gentle correction
//     const double DRIFT_THRESHOLD = 2.0;  // inches
//     const double CORRECTION_GAIN = 0.15; // gentle correction (0-1)
    
//     int left_output = target.left_drive;
//     int right_output = target.right_drive;
    
//     if (error_dist > DRIFT_THRESHOLD) {
//         // Calculate correction needed
//         double correction = error_dist * CORRECTION_GAIN;
        
//         if (error_x > 0) {
//             // Drifted left, push right
//             right_output += correction;
//             left_output -= correction;
//         } else {
//             // Drifted right, push left
//             left_output += correction;
//             right_output -= correction;
//         }
        
//         // Clamp to valid range
//         left_output = std::max(-127, std::min(127, left_output));
//         right_output = std::max(-127, std::min(127, right_output));
        
//         printf("Drift detected: %.2f in - Correcting\n", error_dist);
//     }
    
//     // Apply corrected motor commands
//     chassis.drive_set(left_output, right_output);
//     intakeTop.move(target.intake_top);
//     intakeBottom.move(target.intake_bottom);
//     center.set(target.center_piston);
//     matchLoad.set(target.match_load_piston);
//     odomPod.set(target.odom_pod_piston);
// }
