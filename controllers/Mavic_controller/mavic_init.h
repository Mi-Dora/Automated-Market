#ifndef MAVIC_INIT
#define MAVIC_INIT
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <windows.h>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/camera_recognition_object.h>
#include <webots/supervisor.h>

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define M_PI 3.1415926535
#define GRIPPER_MOTOR_MAX_SPEED 0.1
#define BEER_BOTTLE 1
#define JAM_JAR 2
#define CAN 3
#define WATER_BOTTLE 4
#define HONEY_JAR 5
#define CEREAL_BOX 6
#define BISCUIT_BOX 7

extern WbDeviceTag camera;
extern WbDeviceTag front_left_led;
extern WbDeviceTag front_right_led;
extern WbDeviceTag imu;
extern WbDeviceTag gps;
extern WbDeviceTag compass;
extern WbDeviceTag gyro;
extern WbDeviceTag camera_motor[3];

extern const double k_vertical_thrust;  // with this thrust, the drone lifts.
extern const double k_vertical_offset;   // Vertical offset where the robot actually targets to stabilize itself.
extern const double k_vertical_p;        // P constant of the vertical PID.
extern const double k_roll_p;           // P constant of the roll PID.
extern const double k_pitch_p;          // P constant of the pitch PID.
extern int time_step;

extern double roll;
extern double pitch;
extern double red_x_F;
extern double altitude_y;
extern double green_z;
extern double roll_acceleration;
extern double pitch_acceleration;
extern double yaw_acceleration;
extern double initial_x;
extern double initial_z;
extern double new_target_position_x;
extern double new_target_position_y;
extern double new_target_position_z;
extern const double* north;
extern double angle;

extern double initializelist[7][4];
void device_init();
void update_stage(const double time, int h);
#endif