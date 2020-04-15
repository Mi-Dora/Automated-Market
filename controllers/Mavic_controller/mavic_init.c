#include "mavic_init.h"

WbDeviceTag camera;
WbDeviceTag front_left_led;
WbDeviceTag front_right_led;
WbDeviceTag imu;
WbDeviceTag gps;
WbDeviceTag compass;
WbDeviceTag gyro;
WbDeviceTag camera_motor[3];

const double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
const double k_vertical_offset = 0.01;   // Vertical offset where the robot actually targets to stabilize itself.
const double k_vertical_p = 5.0;        // P constant of the vertical PID.
const double k_roll_p = 50.0;           // P constant of the roll PID.
const double k_pitch_p = 30.0;          // P constant of the pitch PID.
int time_step = 0;
double roll;
double pitch;
double red_x_F;
double altitude_y;
double green_z;
double roll_acceleration;
double pitch_acceleration;
double yaw_acceleration;
double initial_x;
double initial_z;
double new_target_position_x;
double new_target_position_y;
double new_target_position_z;
const double* north;
double angle;

double initializelist[7][4] = { {0.0,0.0,0.143,0.031},{0.0,0.0,0.115,0.045},
    {0.0,0.0,0.122,0.0318},{0.0,0.0,0.31,0.044},{0.0,0.0,0.115,0.045},
    {0.08,0.3,0.2,0.0},{0.583,4.5,1.0,0.0} };

void device_init()
{
    int timestep = (int)wb_robot_get_basic_time_step();
    // Get and enable devices.
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, timestep);
    wb_camera_recognition_enable(camera, timestep);
    front_left_led = wb_robot_get_device("front left led");
    front_right_led = wb_robot_get_device("front right led");
    imu = wb_robot_get_device("inertial unit");
    wb_inertial_unit_enable(imu, timestep);
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, timestep);
    compass = wb_robot_get_device("compass");
    wb_compass_enable(compass, timestep);
    gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(gyro, timestep);
    wb_keyboard_enable(timestep);
    WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
    WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
    WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw");
    camera_motor[0] = camera_roll_motor;
    camera_motor[1] = camera_pitch_motor;
    camera_motor[2] = camera_yaw_motor;
}

void update_stage(const double time, int h)
{
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);
    roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0] + M_PI / 2.0;
    pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    red_x_F = wb_gps_get_values(gps)[0];
    altitude_y = wb_gps_get_values(gps)[1];
    green_z = wb_gps_get_values(gps)[2];

    if (!h)
    {
        initial_x = red_x_F;
        initial_z = green_z;
    }
    roll_acceleration = wb_gyro_get_values(gyro)[0];
    pitch_acceleration = wb_gyro_get_values(gyro)[1];
    yaw_acceleration = wb_gyro_get_values(gyro)[2];
}