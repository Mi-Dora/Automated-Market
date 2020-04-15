#include "mavic_init.h"
#include "motor_contorl.h"
#include "process_control.h"

WbDeviceTag motors[4];

void four_motor_init()
{
    WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
    WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
    WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
    WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
    motors[0] = front_left_motor;
    motors[1] = front_right_motor;
    motors[2] = rear_left_motor;
    motors[3] = rear_right_motor;
    int m;
    for (m = 0; m < 4; ++m) {
        wb_motor_set_position(motors[m], INFINITY);
        wb_motor_set_velocity(motors[m], 1.0);
    }
}

void four_motor_contorl(double roll, double pitch, double height_difference, double roll_acceleration, 
    double roll_disturbance, double pitch_acceleration, double pitch_disturbance, double yaw_disturbance)
{
    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance;
    const double yaw_input = yaw_disturbance;
    const double clamped_difference_altitude = CLAMP(height_difference + k_vertical_offset, -1.0, 1.0);
    const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);
    // Actuate the motors taking into consideration all the computed inputs.
    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    wb_motor_set_velocity(motors[0], front_left_motor_input);
    wb_motor_set_velocity(motors[1], -front_right_motor_input);
    wb_motor_set_velocity(motors[2], -rear_left_motor_input);
    wb_motor_set_velocity(motors[3], rear_right_motor_input);
}