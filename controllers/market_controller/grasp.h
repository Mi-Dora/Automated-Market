/*
Created on Saturday Apr 5 2020
Author : yu du
Email : yuduseu@gmail.com
Last edit date :
*/

#pragma once

#ifndef GRASP_H
#define GRASP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include "arm.h"
#include "base.h"
#include "gripper.h"
#include "tiny_math.h"
#include "basic.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>



double* get_obj_position(int objectID);

const double* get_obj_size(int objectID);

void grasp_ik(double x, double w, double h);

void grasp_prepare(double height, double width);

void grasp_hold(double width);

void grip(double width);

void gripper_lift();

void gripper_down();

bool put_to_back(double w, double h, int shelfid);

bool grasp_shelf(double h, double y);

bool back_to_hold(int shelfid);

bool put_bottle_back(double w, double h, int shelfid);


#ifdef __cplusplus
}
#endif

#endif