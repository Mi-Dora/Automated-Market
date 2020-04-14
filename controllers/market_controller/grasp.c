/*
Created on Saturday Apr 5 2020
Author : yu du
Email : yuduseu@gmail.com
Last edit date : 
*/

#include "grasp.h"

#define BACK_HEIGHT 0.1
#define INIT_GRIPPER_HEIGHT 0.182

static double distance_arm0_platform = DIS_ARM_OBJ;

static bool storage[3] = { false, false, false };
static int shelf[3] = { -1, -1, -1 };
static double storage_size[3][2]; // w, h
static bool grasp = false;

double* get_obj_position(int objectID) {
    double nodePos[2] = {0, 0};
    WbNodeRef node = wb_supervisor_node_get_from_id(objectID);
    const double* nodePosition = wb_supervisor_node_get_position(node);
    nodePos[0] = nodePosition[0];
    nodePos[1] = nodePosition[2];
    return nodePos;
}

const double* get_obj_size(int objectID) {
    WbNodeRef node = wb_supervisor_node_get_from_id(objectID);
    WbFieldRef Field = wb_supervisor_node_get_field(node, "size");
    const double* fieldOfsize = wb_supervisor_field_get_sf_vec3f(Field);
    return fieldOfsize;
}

void grasp_ik(double x, double w, double h) {
    static double h_per_step = 0.002;
    static double platform_height = 0.0;
    static double offset = 0.01;  // security margin

    double y = offset + platform_height + h;
    double z = 0;
    z *= 0.9;  // This fix a small offset that I cannot explain


    // prepare
    //arm_set_sub_arm_rotation(ARM5, M_PI_2);
    arm_ik(x, h + 0.04, z);
    passive_wait(5.0);
    gripper_release();
    passive_wait(1.0);

    // move the arm down
    for (double i = h + 0.04; i > y; i -= h_per_step) {
        arm_ik(x, i, z);
        step();
    }
    
    gripper_set_gap(w - 0.01);
    passive_wait(1.0);
    // move the arm up
    for (double i = y; i < h + 0.04; i += h_per_step) {
        arm_ik(x, i, z);
        step();
    }
    arm_set_orientation(ARM_FRONT);
}


void grasp_prepare(double height, double width) {
    if (height >= HEIGHT_THRESH) {
        arm_set_height(ARM_GRASP_HORIZONTAL);
        gripper_set_height(0.0);
    }
    else {
        arm_set_height(ARM_GRASP_HORIZONTAL);
        gripper_down();
    }
    arm_set_orientation(ARM_FRONT);
    gripper_release();
    grasp = false;
}

void grasp_hold(double width) {
    if (width > 0) {
        gripper_set_gap(width - 0.01);
        passive_wait(1.0);
        grasp = true;
    }
    arm_set_height(ARM_HOLD_STEP1);
    passive_wait(1.0);
    arm_set_height(ARM_HOLD_STEP2);
    arm_set_orientation(ARM_FRONT);
    gripper_set_height(0.0);
    passive_wait(0.5);
}

void grip(double width) {
    if (width > 0) {
        gripper_set_gap(width - 0.01);
        passive_wait(1.0);
        grasp = true;
    }
}

void gripper_lift() {
    gripper_set_height(0.1);
}

void gripper_down() {
    gripper_set_height(-0.1);
}

bool put_to_back(double w, double h, int shelfid) {
    grasp_hold(w);
    double offset = 0.0;
    if (h > HEIGHT_THRESH) {
        if (storage[0] || storage[1] || storage[2]) {
            printf("Back plate is full! Keep holding!\n");
            return false;
        }
        offset = BACK_HEIGHT - (h - INIT_GRIPPER_HEIGHT);
        gripper_set_height(offset);
        arm_set_orientation(ARM_FRONT);
        arm_set_height(ARM_BACK_PLATE);
        passive_wait(4.0);
        gripper_release();
        passive_wait(1.0);
        arm_set_height(ARM_SHRINK1);
        passive_wait(1.5);
        arm_set_height(ARM_SHRINK2);
        passive_wait(1.5);
        arm_set_height(ARM_GRASP_HORIZONTAL);
        gripper_set_height(0.0);
        storage[1] = true;
        storage_size[1][0] = w;
        storage_size[1][1] = h;
        shelf[1] = shelfid;
    }
    else {
        if (storage[1] || (storage[0] && storage[2])) {
            printf("Back plate is full! Keep holding!\n");
            return false;
        }
        offset = BACK_HEIGHT - (h - (INIT_GRIPPER_HEIGHT - 0.1));
        gripper_set_height(offset);
        if (!storage[0]) {
            arm_set_orientation(ARM_FRONT_LEFT);
            storage[0] = true;
            storage_size[0][0] = w;
            storage_size[0][1] = h;
            shelf[0] = shelfid;
        }
        else {
            arm_set_orientation(ARM_FRONT_RIGHT);
            storage[2] = true;
            storage_size[2][0] = w;
            storage_size[2][1] = h;
            shelf[2] = shelfid;
        }
        arm_set_height(ARM_BACK_PLATE);
        passive_wait(4.0);
        gripper_release();
        passive_wait(1.0);
        arm_set_height(ARM_GRASP_HORIZONTAL);
        gripper_set_height(0.0);
    }
    grasp = false;
    return true;
}


// need to be redefined for multi backup
bool back_to_hold(int shelfid) {
    double offset = 0.0;
    if (storage[1]) {
        offset = BACK_HEIGHT - (storage_size[1][1] - INIT_GRIPPER_HEIGHT);
        gripper_release();
        gripper_set_height(offset);
        arm_set_height(ARM_SHRINK2);
        passive_wait(4.0);
        arm_set_height(ARM_SHRINK1);
        passive_wait(1.0);
        arm_set_height(ARM_BACK_PLATE);
        passive_wait(2.5);
        gripper_set_gap(storage_size[1][0] - 0.01);
        passive_wait(1.0);
        storage[1] = false;
    }
    else {
        double w = 0.0;
        if (storage[0]) {
            arm_set_orientation(ARM_FRONT_LEFT);
            offset = BACK_HEIGHT - (storage_size[0][1] - (INIT_GRIPPER_HEIGHT - 0.1));
            w = storage_size[0][0];
            storage[0] = false;
        }
        else if (storage[2]) {
            arm_set_orientation(ARM_FRONT_RIGHT);
            offset = BACK_HEIGHT - (storage_size[2][1] - (INIT_GRIPPER_HEIGHT - 0.1));
            w = storage_size[2][0];
            storage[2] = false;
        }
        else {
            printf("No such goods on back plate!\n");
            return false;
        }
        gripper_set_height(offset);
        arm_set_height(ARM_BACK_PLATE);
        passive_wait(6.0);
        gripper_set_gap(w - 0.01);
        passive_wait(1.0);
    }
    arm_set_height(ARM_HOLD_STEP2);
    gripper_set_height(0.0);
    arm_set_orientation(ARM_FRONT);
    return true;
}


bool grasp_shelf(double h, double y) {
    arm_set_orientation(ARM_FRONT);
    double gap_grriper_shelf = 0.0;
    if (y >= 0.4) {
        gap_grriper_shelf = 0.1;
        arm_set_height(ARM_SHELF_HIGH);
        passive_wait(1.0);
    }
    else if (y >= 0.2) {
        gap_grriper_shelf = 0.12;
        arm_set_height(ARM_SHELF_MIDDLE);
        passive_wait(2.0);
    }
    else {
        gap_grriper_shelf = INIT_GRIPPER_HEIGHT-0.05;
        arm_set_height(ARM_SHELF_LOW_STEP1);
        passive_wait(2.0);
        arm_set_height(ARM_SHELF_LOW_STEP2);
        passive_wait(2.0);

    }
    double offset = 0.0;
    if (h > HEIGHT_THRESH)
        offset = -(gap_grriper_shelf - INIT_GRIPPER_HEIGHT);
    else
        offset = -(gap_grriper_shelf - (INIT_GRIPPER_HEIGHT - 0.1));
    gripper_set_height(offset);
}




