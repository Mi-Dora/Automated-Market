/*
Created on Saturday Apr 5 2020
Author : yu du
Email : yuduseu@gmail.com
Last edit date :
*/

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


typedef struct {
    Vector3 i_pos;
    Vector2 size;
    Vector3 o_pos;
} Command;

#define EMIT_CHANNEL 1
#define RECEIVE_CHANNEL 2

static WbDeviceTag cam[2], receiver, emitter;
unsigned short width, height;
static double distance_arm0_platform = DIS_ARM_OBJ;



int main(int argc, char** argv) {
    wb_robot_init();
    base_init();
    base_goto_init(TIME_STEP);
    arm_init();
    gripper_init();

    cam[0] = wb_robot_get_device("camera1");
    wb_camera_enable(cam[0], TIME_STEP_CAM);
    width = wb_camera_get_width(cam[0]);
    height = wb_camera_get_height(cam[0]);
    wb_camera_recognition_enable(cam[0], TIME_STEP_CAM);


    receiver = wb_robot_get_device("receiver");
    emitter = wb_robot_get_device("emitter");
    wb_receiver_enable(receiver, TIME_STEP);

    /* if the cannel is not the right one, change it */
    const int channel = wb_emitter_get_channel(emitter);
    if (channel != EMIT_CHANNEL) {
        wb_emitter_set_channel(emitter, EMIT_CHANNEL);
    }
    wb_receiver_set_channel(receiver, RECEIVE_CHANNEL);

    passive_wait(2.0);


    
    
    int have_done = 0;
    while (true) {
        step();
        /* is there at least one packet in the receiver's queue ? */
        if (wb_receiver_get_queue_length(receiver) > 0) {
            /* read current packet's data */
            const char* head = wb_receiver_get_data(receiver);
            void* p = wb_receiver_next_packet(receiver);
            Command* command = (Command*)p;
            if (grasp_and_place(&command->i_pos, &command->size, &command->o_pos)) {
                have_done++;
                char message[128];
                sprintf(message, "%d", have_done);
                wb_emitter_send(emitter, message, strlen(message) + 1);
            }
        }

        //double h = 0.122;
        //double w = 0.0318;
        //double d = 0.0318;
        //double x = -0.343;
        //double z = 0.55;
        //double ox = 1.875;
        //double oy = 0.01;
        //double oz = 0.125;
        //Vector2 i_pos;
        //Vector3 size;
        //Vector3 o_pos;
        //vector2_set_values(&i_pos, x, z);
        //vector3_set_values(&size, h, w, d);
        //vector3_set_values(&o_pos, ox, oy, oz);
        //grasp_and_place(&i_pos, &size, &o_pos);

        //passive_wait(10.0);



    }

    wb_robot_cleanup();

    //return 0;
}
