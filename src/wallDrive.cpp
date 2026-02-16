#include "main.h"
#include "wallDrive.hpp"
#include "autons.hpp"
#include "drivetrain.hpp"


// Distance sensor definitions
pros::Distance leftDist(9);
pros::Distance rightDist(4);
pros::Distance frontDist(8);
pros::Distance rightDistAlt(3);
// pros::Distance backDist(0);


// Sensor offsets
constexpr float RIGHT_SENSOR_OFFSET = 1.25;
constexpr float RIGHT_SENSOR_ALT_OFFSET = 5.5;
constexpr float LEFT_SENSOR_OFFSET  = 5.25;
constexpr float FRONT_SENSOR_OFFSET = 2.75;
// constexpr float BACK_SENSOR_OFFSET  = 6.5;

// Distance between front right and back right sensors
constexpr float RIGHT_SENSOR_SPACING = 5.35;


// Field wall positions
constexpr float WALL_X_LEFT   = 0.0;
constexpr float WALL_X_RIGHT  = 140.0;
constexpr float WALL_Y_BACK   = 140.0;
constexpr float WALL_Y_FRONT  = 0.0;

double normalize_angle(double a) {
    while (a > 180) a -= 360;
    while (a < -180) a += 360;
    return a;
}

bool near_angle(double theta, double target, double tol) {
    return fabs(normalize_angle(theta - target)) < tol;
}

// Convert distance sensor input to inches
float distance_in(pros::Distance &sensor) {
    return sensor.get() / 25.4f;
}


//Distance sensor confidence weighting
float sensor_confidence(float d) {
    constexpr float MIN_D = 2.0f;      // invalid below
    constexpr float MAX_D = 40.0f;     // invalid above
    constexpr float FULL_CONF = 12.0f; // full confidence when close

    if (d < MIN_D || d > MAX_D) return 0.0f;
    if (d <= FULL_CONF) return 1.0f;

    return (MAX_D - d) / (MAX_D - FULL_CONF);
}


//Confidence weighted odom correction
void correct_odom_with_sensors() {

    //Read sensors
    float d_right = distance_in(rightDist);
    float d_left  = distance_in(leftDist);
    float d_front = distance_in(frontDist);

    //current odom values
    double x = chassis.odom_x_get();
    double y = chassis.odom_y_get();
    double theta = chassis.odom_theta_get();

    //Xcorrection
    float conf_r = sensor_confidence(d_right);
    float conf_l = sensor_confidence(d_left);

    if (conf_r > conf_l && conf_r > 0.0f) {
        double x_meas = WALL_X_RIGHT - (d_right + RIGHT_SENSOR_OFFSET);
        x += conf_r * (x_meas - x);
    }
    else if (conf_l > 0.0f) {
        double x_meas = WALL_X_LEFT + d_left + LEFT_SENSOR_OFFSET;
        x += conf_l * (x_meas - x);
    }

    //Y correction
    float conf_f = sensor_confidence(d_front);

    if (conf_f > 0.0f) {
        double y_meas = WALL_Y_FRONT - (d_front + FRONT_SENSOR_OFFSET);
        y += conf_f * (y_meas - y);
    }

    // Apply blended odom update
    chassis.odom_xyt_set(x, y, theta);
}


//move along the wall on the x axis
void wall_drive_x(float x_target, double known_y, int speed) {
    double current_x = chassis.odom_x_get();
    double distance_to_drive = x_target - current_x;

    while (fabs(distance_to_drive) > 0.5) {
        correct_odom_with_sensors();

        current_x = chassis.odom_x_get();
        distance_to_drive = x_target - current_x;

        // chassis.pid_odom_set(distance_to_drive, speed);
        // chassis.pid_wait();
        chassis.pid_odom_set(x_target, known_y, speed);
        chassis.pid_wait();
    }
}


//move along the wall on the y axis
void wall_drive_y(double known_x, float y_target, int speed) {
    double current_y = chassis.odom_y_get();
    double distance_to_drive = y_target - current_y;

    while (fabs(distance_to_drive) > 0.5) {
        correct_odom_with_sensors();

        current_y = chassis.odom_y_get();
        distance_to_drive = y_target - current_y;

        // chassis.pid_odom_set(distance_to_drive, speed);
        // chassis.pid_wait();

        chassis.pid_odom_set(known_x, y_target, speed);
        chassis.pid_wait();
    }
}

// void wall_drive_to_point(float x_target, float y_target, int speed) {

//     //Move along x axis
//     double current_x = chassis.odom_x_get();
//     double dx = x_target - current_x;

//     if (fabs(dx) > 0.5) {
//         double heading_x = (dx > 0) ? 0 : 180;
//         chassis.pid_turn_set(heading_x, 90);
//         chassis.pid_wait();

//         wall_drive_x(x_target, y_target, speed);
//     }

//     // //move along y axis
//     double current_y = chassis.odom_y_get();
//     double dy = y_target - current_y;

//     // if (fabs(dy) > 0.5) {
//     //     double heading_y = (dy > 0) ? 90 : -90;
//     //     chassis.pid_turn_set(heading_y, 90);
//     //     chassis.pid_wait();

//     //     wall_drive_y(y_target, speed);
//     // }

//     // keep moving until reach desired point
//     if (fabs(dx) > 0.5) {
//         double heading_y = (dy > 0) ? 90 : -90;
//         chassis.pid_turn_set(heading_y, 0);
//         chassis.pid_wait();

//         wall_drive_y(x_target, y_target, speed);
//     }
//     else if (current_y > WALL_Y_BACK - y_target + FRONT_SENSOR_OFFSET){
//         wall_drive_y(x_target, y_target, speed);
//     }
// }

void wall_drive_to_point(double x_target, double y_target, int speed) {

    chassis.pid_odom_set(x_target, y_target, speed);

    while (true) {
        correct_odom_with_sensors();

        double dx = x_target - chassis.odom_x_get();
        double dy = y_target - chassis.odom_y_get();

        if (sqrt(dx*dx + dy*dy) < 0.75) break;
        pros::delay(10);
    }
}


void wall_drive_to_point_fwd_y_back(double x_target, double y_target, int speed, double y_tol) {
    // Face target
    double dx = x_target - chassis.odom_x_get();
    double dy = y_target - chassis.odom_y_get();
    // double heading = atan2(dy, dx) * 180.0 / M_PI;

    chassis.pid_turn_set(0_deg, 90);
    chassis.pid_wait();

    chassis.pid_odom_set(x_target, y_target, speed);

    uint32_t start = pros::millis();

    while (pros::millis() - start < 3000) { // safety timeout
        correct_odom_with_sensors();

        float d = distance_in(frontDist);
        if (sensor_confidence(d) > 0.0f) {
            double y_meas = WALL_Y_BACK - (d + FRONT_SENSOR_OFFSET);

            if (fabs(y_meas - y_target) < y_tol) {
                chassis.pid_odom_set(chassis.odom_x_get(), chassis.odom_y_get(), speed);
                chassis.pid_wait_quick();
                return;
            }
        }

        pros::delay(10);
    }

    chassis.pid_wait(); // fallback
}


void wall_drive_to_point_fwd_y_front(double x_target, double y_target, int speed, double y_tol) {
    // Face target
    double dx = x_target - chassis.odom_x_get();
    double dy = y_target - chassis.odom_y_get();
    // double heading = atan2(dy, dx) * 180.0 / M_PI;
    chassis.pid_turn_set(180_deg, 90);
    chassis.pid_wait();

    chassis.pid_odom_set(x_target, y_target, speed);

    uint32_t start = pros::millis();

    while (pros::millis() - start < 3000) { // safety timeout
        correct_odom_with_sensors();

        float d = distance_in(frontDist);
        if (sensor_confidence(d) > 0.0f) {
            double y_meas = WALL_Y_FRONT + (d + FRONT_SENSOR_OFFSET);

            if (fabs(y_meas - y_target) < y_tol) {
                chassis.pid_odom_set(chassis.odom_x_get(), chassis.odom_y_get(), speed);
                chassis.pid_wait_quick();
                return;
            }
        }

        pros::delay(10);
    }

    chassis.pid_wait(); // fallback
}


//move directly to a point using distance sensors 
void wall_drive_to_point_direct(float x_target, float y_target, int speed) {
    double x = chassis.odom_x_get();
    double y = chassis.odom_y_get();

    double dx = x_target - x;
    double dy = y_target - y;

    double dist = sqrt(dx * dx + dy * dy);
    if (dist < 0.5) return;

    double heading = atan2(dy, dx) * 180.0 / M_PI;

    chassis.pid_turn_set(heading, speed);
    chassis.pid_wait();

    chassis.pid_odom_set(dist, speed);
    chassis.pid_wait();

    correct_odom_with_sensors();
}


//turn to an angle using distance sensor input
void turn_to_angle(double target_angle, int speed, double tol) {
    chassis.pid_turn_set(target_angle, speed);

    while (fabs(normalize_angle(chassis.odom_theta_get() - target_angle)) > tol) {
        correct_odom_with_sensors();
        pros::delay(10);
    }
}


//function to find y coordinate using 2 right side distance sensors
void wall_snap(double known_x) {
    float d_front = distance_in(rightDist);
    float d_back  = distance_in(rightDistAlt);

    float conf_f = sensor_confidence(d_front);
    float conf_b = sensor_confidence(d_back);
    if (conf_f == 0.0f || conf_b == 0.0f) return;

    double y = WALL_Y_BACK - (((d_front + d_back) / 2.0) + ((RIGHT_SENSOR_OFFSET + RIGHT_SENSOR_ALT_OFFSET) / 2));

    double delta = fabs(d_front - d_back);

    double horizontal = sqrt(
        RIGHT_SENSOR_SPACING * RIGHT_SENSOR_SPACING
        - delta * delta
    );

    double t = atan2(delta, horizontal);

    chassis.odom_xyt_set(known_x, y, t);
}
 

 //prints x, y, t values to brain screen (used for checking reliability)
void print_odom() {
    pros::lcd::print(0, "X: %.2f", chassis.odom_x_get());
    pros::lcd::print(1, "Y: %.2f", chassis.odom_y_get());
    pros::lcd::print(2, "T: %.2f", chassis.odom_theta_get());
}
