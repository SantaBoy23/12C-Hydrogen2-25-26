// #include "main.h"
// #include "wallDrive.hpp"
// #include "autons.hpp"
// #include "drivetrain.hpp"
// #include "distanceIntegration.hpp"


// // Distance sensor definitions
// pros::Distance leftDist(9);
// pros::Distance rightDist(4);
// pros::Distance frontDist(8);
// pros::Distance rightDistAlt(3);


// // Sensor offsets
// constexpr float RIGHT_SENSOR_OFFSET = 1.25;
// constexpr float RIGHT_SENSOR_ALT_OFFSET = 5.5;
// constexpr float LEFT_SENSOR_OFFSET  = 5.25;
// constexpr float FRONT_SENSOR_OFFSET = 2.75;

// // Distance between front right and back right sensors
// constexpr float RIGHT_SENSOR_SPACING = 5.35;

// //orintation/location/position of bot
// double position = 0;
// double measured_x = 0;
// double measured_y = 0;
// double estimated_x = 0;
// double estimated_y = 0;

// // Field wall positions
// constexpr float WALL_X_LEFT   = 0.0;
// constexpr float WALL_X_RIGHT  = 140.0;
// constexpr float WALL_Y_BACK   = 140.0;
// constexpr float WALL_Y_FRONT  = 0.0;


// //Distance sensor confidence weighting
// float sensor_confidence(float d) {
//     constexpr float MIN_D = 2.0f;      // invalid below
//     constexpr float MAX_D = 40.0f;     // invalid above
//     constexpr float FULL_CONF = 12.0f; // full confidence when close

//     if (d < MIN_D || d > MAX_D) return 0.0f;
//     if (d <= FULL_CONF) return 1.0f;

//     return (MAX_D - d) / (MAX_D - FULL_CONF);
// }

// //Read sensors
// float d_right = distance_in(rightDist);
// float d_left  = distance_in(leftDist);
// float d_front = distance_in(frontDist);

// //Confidence weighted odom correction
// void correct_odom_with_sensors() {

//     //Read sensors
//     float d_right = distance_in(rightDist);
//     float d_left  = distance_in(leftDist);
//     float d_front = distance_in(frontDist);

//     //current odom values
//     double estimated_x = chassis.odom_x_get();
//     double estimated_y = chassis.odom_y_get();
//     double theta = chassis.odom_theta_get();

//     // Robot X correction
//     float conf_r = sensor_confidence(d_right);
//     float conf_l = sensor_confidence(d_left);

//     if (conf_r > conf_l && conf_r > 0.0f && -10 <= theta <= 10) {
//         double measured_x = WALL_X_RIGHT - (d_right + RIGHT_SENSOR_OFFSET);

//         double position = 1;
//     }
//     else if (conf_r > conf_l && conf_r > 0.0f && 170 <= theta <= 190) {
//         double measured_x = WALL_X_LEFT + (d_right + RIGHT_SENSOR_OFFSET);

//         double position = 1;
//     }
//     else if (conf_r > conf_l && conf_r > 0.0f && 260 <= theta <= 280) {
//         double measured_x = WALL_Y_BACK - (d_right + RIGHT_SENSOR_OFFSET);
        

//         double position = 1;
//     }
//     else if (conf_r > conf_l && conf_r > 0.0f && 80 <= theta <= 100) {
//         double measured_x = WALL_Y_FRONT + (d_right + RIGHT_SENSOR_OFFSET);
        

//         double position = 1;
//     }
//     else if (conf_l > 0.0f && -10 <= theta <= 10) {
//         double measured_x = WALL_X_LEFT + (d_left + LEFT_SENSOR_OFFSET);
        

//         double position = 2;
//     }
//     else if (conf_l > 0.0f && 170 <= theta <= 190) {
//         double measured_x = WALL_X_RIGHT - (d_left + LEFT_SENSOR_OFFSET);
        

//         double position = 2;
//     }
//     else if (conf_l > 0.0f && 260 <= theta <= 280) {
//         double measured_x = WALL_Y_FRONT + (d_left + LEFT_SENSOR_OFFSET);
        

//         double position = 2;
//     }
//     else if (conf_l > 0.0f && 80 <= theta <= 100) {
//         double measured_x = WALL_Y_BACK - (d_left + LEFT_SENSOR_OFFSET);
        

//         double position = 2;
//     }

//     // Robot Y correction
//     float conf_f = sensor_confidence(d_front);

//     if (conf_f > 0.0f && -10 <= theta <= 10) {
//         double measured_y = WALL_Y_BACK - (d_front + FRONT_SENSOR_OFFSET);
        
//     }
//     else if (conf_f > 0.0f && 80 <= theta <= 100) {
//         double measured_y = WALL_X_RIGHT - (d_front + FRONT_SENSOR_OFFSET);
        
//     }
//     else if (conf_f > 0.0f && 170 <= theta <= 190) {
//         double measured_y = WALL_Y_FRONT + (d_front + FRONT_SENSOR_OFFSET);
        
//     }
//     else if (conf_f > 0.0f && 260 <= theta <= 280) {
//         double measured_y = WALL_X_LEFT + (d_front + FRONT_SENSOR_OFFSET);
        
//     }
// }

// void weightSensorReadings () {
//     double new_x = 1;
//     correct_odom_with_sensors();

//     double alpha = 0.1; // trust factor (0 = ignore sensor, 1 = fully trust)

//     if (position = 1){
//        double new_x = (1 - alpha) * estimated_x + alpha * measured_x;
//     }
//     else if (position = 2){
//         double new_y = (1 - alpha) * estimated_y + alpha * measured_y;
//     }

//     chassis.odom_x_set(new_x);
// }