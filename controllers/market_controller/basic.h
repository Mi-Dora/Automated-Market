/*
Created on Saturday Apr 5 2020
Author : yu du
Email : yuduseu@gmail.com
Last edit date :
*/

#pragma once

#ifndef BASIC_H
#define BASIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 16
#define TIME_STEP_CAM 16

#define DIS_ARM_OBJ 0.2
#define DIS_APPROACH 0.3 
#define HEIGHT_THRESH 0.181
#define H_SHELF_PLATE 0.01



void step();
void passive_wait(double sec);

#ifdef __cplusplus
}
#endif

#endif