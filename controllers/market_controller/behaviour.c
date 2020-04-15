/*
Created on Saturday Apr 13 2020
Author : yu du
Email : yuduseu@gmail.com
Last edit date :
*/


/*top layer of youbot control*/
#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include "arm.h"
#include "base.h"
#include "gripper.h"
#include "tiny_math.h"
#include "navigate.h"
#include "grasp.h"
#include "basic.h"
#include "behaviour.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// defined for status
#define PLATEFORM 1
#define SHELF -1


/*grasp only one good at one time*/
bool grasp_and_place(double* i_pos, double* size, double* o_pos) {
	/*Input:
		i_pos: coordinate of goods position (x, z) (input position)
		size: height, width, depth of the goods of grasp
		o_pos: coordinate of goods position (x, y, z) £¨output position£©
	*/
	double ix = i_pos[0];
	double iz = i_pos[1];
	double h = size[0];
	double w = size[1];
	double d = size[2]; // depth
	double ox = o_pos[0];
	double oy = o_pos[1];
	double oz = o_pos[2];

	grasp_hold(-0.1);
    go_to_translation(ix, iz, PLATEFORM);
	grasp_prepare(h, w);
	passive_wait(3.0);
    approach(d, h, PLATEFORM);
	grasp_hold(w);
	
	go_to_translation(ox, oz, SHELF);
	//set place posture
	if (!grasp_shelf(h, oy))
		return false;

	approach(d, h, SHELF);
	gripper_release();
	passive_wait(1.0);
	backup(0.2);
	return true;
}

bool grasp2_and_place(double* i_pos, double* size, double* o_pos) {
	double ix = i_pos[0];
	double iz = i_pos[1];
	double h = size[0];
	double w = size[1];
	double d = size[2]; // depth
	double ox = o_pos[0];
	double oy = o_pos[1];
	double oz = o_pos[2];

	grasp_hold(-0.1);
	go_to_translation(ix, iz, PLATEFORM);
	grasp_prepare(h, w);
	passive_wait(3.0);
	approach(d, h, PLATEFORM);
	//grasp_hold(w);
	put_to_back(w, h, 3);

	go_to_translation(ox, oz, SHELF);
	//set place posture
	if (!grasp_shelf(h, oy))
		return false;

	approach(d, h, SHELF);
	gripper_release();
	passive_wait(1.0);
	backup(0.2);
	return true;
}