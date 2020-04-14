/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Implement the functions defined in gripper.h
 */

#include "gripper.h"

#include <webots/motor.h>
#include <webots/robot.h>

#include "tiny_math.h"

#define LIFT 0
#define LEFT 1
#define RIGHT 2

#define MIN_POS 0.0
#define MAX_POS 0.05
#define OFFSET_WHEN_LOCKED 0.021
#define GRIPPER_MOTOR_MAX_SPEED 10
#define MAX_POS_LIFT 0.05
#define MIN_POS_LIFT -0.05
#define MAX_V_LIFT 0.1

static WbDeviceTag fingers[3];

void gripper_init() {
    fingers[LIFT] = wb_robot_get_device("lift motor");
    fingers[LEFT] = wb_robot_get_device("left finger motor");
    fingers[RIGHT] = wb_robot_get_device("right finger motor");
    wb_motor_set_velocity(fingers[LIFT], MAX_V_LIFT);
    wb_motor_set_velocity(fingers[LEFT], GRIPPER_MOTOR_MAX_SPEED);
    wb_motor_set_velocity(fingers[RIGHT], GRIPPER_MOTOR_MAX_SPEED);
}

void gripper_grip() {
    wb_motor_set_position(fingers[LEFT], MIN_POS);
    wb_motor_set_position(fingers[RIGHT], MIN_POS);
}

void gripper_release() {
    wb_motor_set_position(fingers[LEFT], MAX_POS);
    wb_motor_set_position(fingers[RIGHT], MAX_POS);
}

void gripper_set_gap(double gap) {
    double v = bound(0.5 * (gap - OFFSET_WHEN_LOCKED), MIN_POS, MAX_POS);
    wb_motor_set_position(fingers[LEFT], v);
    wb_motor_set_position(fingers[RIGHT], v);
}

void gripper_set_height(double height) {
    wb_motor_set_position(fingers[LIFT], height);
}