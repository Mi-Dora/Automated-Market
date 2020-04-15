/*
Created on Saturday Apr 5 2020
Author : yu du
Email : yuduseu@gmail.com
Last edit date :
*/

#pragma once

#ifndef NAVIGATE_H
#define NAVIGATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>

#include "base.h"
#include "tiny_math.h"
#include "basic.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void go_to(double x, double z, double a);
int get_partition(double x, double z);
double distance_clockwise(double rbt_x, double rbt_z, double obj_x, double obj_z);
double distance_anticlockwise(double rbt_x, double rbt_z, double obj_x, double obj_z);
int go_to_grasp(double x, double z); // param should be the position of the goods
int go_to_shelf(double x, double z); // param should be the position of the the shelf grid
Vector3 differentiation(double ix, double iz, double ia, double ox, double oz, double oa); // a = alpha
void CP_planning(double ix, double iz, double ia, double ox, double oz, double oa);
void curve_turning(int tf_pt);
void go_to_point(double ox, double oz);
void go_to_translation(double ox, double oz, int status);
void approach(double depth, double height, int status);
void backup(double distance);
void turning(double alpha);



#ifdef __cplusplus
}
#endif

#endif

