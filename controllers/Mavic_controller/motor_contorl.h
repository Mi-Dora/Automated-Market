#ifndef MOTOR_CONTROL
#define MOTOR_CONTROL
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

extern WbDeviceTag motors[4];

void four_motor_init();
void four_motor_contorl(double roll, double pitch, double height_difference, double roll_acceleration,
    double roll_disturbance, double pitch_acceleration, double pitch_disturbance, double yaw_disturbance);
#endif