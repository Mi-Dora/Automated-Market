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
 * Description:   Implement the functions defined in base.h
 */

#define _CRT_SECURE_NO_WARNINGS
#include "base.h"


#define SPEED 4.0
#define DISTANCE_TOLERANCE 0.001
#define ANGLE_TOLERANCE 0.001

// stimulus coefficients
#define K1 3.0
#define K2 1.0
#define K3 1.0





static void base_set_wheel_velocity(WbDeviceTag t, double velocity) {
  wb_motor_set_position(t, INFINITY);
  wb_motor_set_velocity(t, velocity);
}

static void base_set_wheel_speeds_helper(double speeds[4]) {
  int i;
  for (i = 0; i < 4; i++)
    base_set_wheel_velocity(wheels[i], speeds[i]);
}

void base_init() {
  int i;
  char wheel_name[16];
  for (i = 0; i < 4; i++) {
    sprintf(wheel_name, "wheel%d", (i + 1));
    wheels[i] = wb_robot_get_device(wheel_name);
  }
}

void base_reset() {
  static double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  base_set_wheel_speeds_helper(speeds);
}

void base_forwards() {
  static double speeds[4] = {SPEED, SPEED, SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_backwards() {
  static double speeds[4] = {-SPEED, -SPEED, -SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_left() {
  static double speeds[4] = {-SPEED, SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_right() {
  static double speeds[4] = {SPEED, -SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_strafe_left() {
  static double speeds[4] = {SPEED, -SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_strafe_right() {
  static double speeds[4] = {-SPEED, SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_goto_init(double time_step) {
  gps = wb_robot_get_device("gps");
  compass = wb_robot_get_device("compass");
  if (gps)
    wb_gps_enable(gps, time_step);
  if (compass)
    wb_compass_enable(compass, time_step);
  if (!gps || !compass)
    fprintf(stderr, "cannot use goto feature without GPS and Compass");

  goto_data.v_target.u = 0.0;
  goto_data.v_target.v = 0.0;
  goto_data.alpha = 0.0;
  goto_data.reached = false;
}

void base_goto_set_target(double x, double z, double alpha) {
  if (!gps || !compass)
    fprintf(stderr, "base_goto_set_target: cannot use goto feature without GPS and Compass");

  goto_data.v_target.u = x;
  goto_data.v_target.v = z;
  goto_data.alpha = alpha;
  goto_data.reached = false;
}

void base_goto_run() {
  if (!gps || !compass)
    fprintf(stderr, "base_goto_set_target: cannot use goto feature without GPS and Compass");

  // get sensors
  const double *gps_raw_values = wb_gps_get_values(gps);
  const double *compass_raw_values = wb_compass_get_values(compass);

  // compute 2d vectors
  Vector2 v_gps = {gps_raw_values[0], gps_raw_values[2]};
  Vector2 v_front = {compass_raw_values[0], compass_raw_values[1]};
  Vector2 v_right = {-v_front.v, v_front.u};
  Vector2 v_north = {1.0, 0.0};

  // compute distance
  Vector2 v_dir;
  vector2_minus(&v_dir, &goto_data.v_target, &v_gps);
  double distance = vector2_norm(&v_dir);

  // compute absolute angle & delta with the delta with the target angle
  double theta = vector2_angle(&v_front, &v_north);
  double delta_angle = theta - goto_data.alpha;

  // compute the direction vector relatively to the robot coordinates
  // using an a matrix of homogenous coordinates
  Matrix33 transform;
  matrix33_set_identity(&transform);
  transform.a.u = v_front.u;
  transform.a.v = v_right.u;
  transform.b.u = v_front.v;
  transform.b.v = v_right.v;
  transform.c.u = -v_front.u * v_gps.u - v_front.v * v_gps.v;
  transform.c.v = -v_right.u * v_gps.u - v_right.v * v_gps.v;
  Vector3 v_target_tmp = {goto_data.v_target.u, goto_data.v_target.v, 1.0};
  Vector3 v_target_rel;
  matrix33_mult_vector3(&v_target_rel, &transform, &v_target_tmp);

  // compute the speeds
  double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  // -> first stimulus: delta_angle
  speeds[0] = -delta_angle / M_PI * K1;
  speeds[1] = delta_angle / M_PI * K1;
  speeds[2] = -delta_angle / M_PI * K1;
  speeds[3] = delta_angle / M_PI * K1;

  // -> second stimulus: u coord of the relative target vector
  speeds[0] += v_target_rel.u * K2;
  speeds[1] += v_target_rel.u * K2;
  speeds[2] += v_target_rel.u * K2;
  speeds[3] += v_target_rel.u * K2;

  // -> third stimulus: v coord of the relative target vector
  speeds[0] += -v_target_rel.v * K3;
  speeds[1] += v_target_rel.v * K3;
  speeds[2] += v_target_rel.v * K3;
  speeds[3] += -v_target_rel.v * K3;

  // apply the speeds
  int i;
  for (i = 0; i < 4; i++) {
    speeds[i] /= (K1 + K2 + K2);  // number of stimuli (-1 <= speeds <= 1)
    speeds[i] *= SPEED;           // map to speed (-SPEED <= speeds <= SPEED)

    // added an arbitrary factor increasing the convergence speed
    speeds[i] *= 30.0;
    speeds[i] = bound(speeds[i], -SPEED, SPEED);
  }
  base_set_wheel_speeds_helper(speeds);

  // check if the taget is reached
  if (distance < DISTANCE_TOLERANCE && delta_angle < ANGLE_TOLERANCE && delta_angle > -ANGLE_TOLERANCE)
    goto_data.reached = true;
}

bool base_goto_reached() {
  return goto_data.reached;
}
