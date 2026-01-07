#include "main.h"
#include "wallDrive.hpp"
#include "autons.hpp"   // distance sensors
#include "drivetrain.hpp"    // chassis


// Sensor offsets from robot center (in inches)
constexpr float RIGHT_SENSOR_OFFSET = -1.5;
constexpr float LEFT_SENSOR_OFFSET  = -1.5;
constexpr float FRONT_SENSOR_OFFSET = 0;
// constexpr float BACK_SENSOR_OFFSET  = 6.5;

// Field wall positions (in inches)
constexpr float WALL_X_LEFT   = 0.0;
constexpr float WALL_X_RIGHT  = 140.0; // adjust to your field width
constexpr float WALL_Y_BACK   = 0.0;
constexpr float WALL_Y_FRONT  = 140.0; // adjust to your field length

static double normalize_angle(double a) {
    while (a > 180) a -= 360;
    while (a < -180) a += 360;
    return a;
}

static bool near_angle(double theta, double target, double tol) {
    return fabs(normalize_angle(theta - target)) < tol;
}

// Convert V5 Distance sensor reading (mm) to inches
inline float distance_in(pros::Distance &sensor) {
    return sensor.get() / 25.4f; // mm → in
}




// ------------------------------
// Distance sensor confidence weighting
// ------------------------------
static float sensor_confidence(float d) {
    constexpr float MIN_D = 2.0f;     // invalid below
    constexpr float MAX_D = 40.0f;    // invalid above
    constexpr float FULL_CONF = 12.0f; // full confidence when close

    if (d < MIN_D || d > MAX_D) return 0.0f;
    if (d <= FULL_CONF) return 1.0f;

    return (MAX_D - d) / (MAX_D - FULL_CONF);
}

// ------------------------------
// Confidence-weighted odometry correction (X + Y)
// ------------------------------
void correct_odom_with_sensors() {

    // Read sensors (inches)
    float d_right = distance_in(rightDist);
    float d_left  = distance_in(leftDist);
    float d_front = distance_in(frontDist);

    // Current odometry
    double x = chassis.odom_x_get();
    double y = chassis.odom_y_get();
    double theta = chassis.odom_theta_get();

    // ---------------- X correction ----------------
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

    // ---------------- Y correction ----------------
    float conf_f = sensor_confidence(d_front);

    if (conf_f > 0.0f) {
        double y_meas = WALL_Y_FRONT - (d_front + FRONT_SENSOR_OFFSET);
        y += conf_f * (y_meas - y);
    }

    // Apply blended odometry update
    chassis.odom_xyt_set(x, y, theta);
}


void wall_drive_x(float x_target, int speed = 80) {
    double current_x = chassis.odom_x_get();

    // compute relative distance to target
    double distance_to_drive = x_target - current_x;

    while (fabs(distance_to_drive) > 0.5) {
        correct_odom_with_sensors(); // correct odom using sensors

        // recompute distance after correction
        current_x = chassis.odom_x_get();
        distance_to_drive = x_target - current_x;

        // drive the remaining distance
        chassis.pid_odom_set(distance_to_drive, speed);
        chassis.pid_wait();
    }
}

void wall_drive_y(float y_target, int speed = 80) {
    double current_y = chassis.odom_y_get();
    double distance_to_drive = y_target - current_y;

    while (fabs(distance_to_drive) > 0.5) {
        correct_odom_with_sensors(); // correct odom using sensors

        // recompute distance after correction
        current_y = chassis.odom_y_get();
        distance_to_drive = y_target - current_y;

        // drive the remaining distance
        chassis.pid_odom_set(distance_to_drive, speed);
        chassis.pid_wait();
    }
}


void wall_drive_to_point(float x_target, float y_target, int speed) {

    // --- Move in X ---
    double current_x = chassis.odom_x_get();
    double dx = x_target - current_x;

    if (fabs(dx) > 0.5) {
        double heading_x = (dx > 0) ? 0 : 180;
        chassis.pid_turn_set(heading_x, 90);
        chassis.pid_wait();

        wall_drive_x(x_target, speed);
    }

    // --- Move in Y ---
    double current_y = chassis.odom_y_get();
    double dy = y_target - current_y;

    if (fabs(dy) > 0.5) {
        double heading_y = (dy > 0) ? 90 : -90;
        chassis.pid_turn_set(heading_y, 90);
        chassis.pid_wait();

        wall_drive_y(y_target, speed);
    }
}

void turn_to_angle(double target_angle, int speed = 90, double tol = 1.0) {
    // Start PID turn
    chassis.pid_turn_set(target_angle, speed);

    // Continue correcting odometry while turning
    while (fabs(normalize_angle(chassis.odom_theta_get() - target_angle)) > tol) {
        correct_odom_with_sensors(); // continuous correction
        pros::delay(10);
    }
}
