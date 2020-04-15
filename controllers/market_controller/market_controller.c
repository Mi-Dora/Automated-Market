/*
Created on Saturday Apr 5 2020
Author : yu du
Email : yuduseu@gmail.com
Last edit date :
*/
#define _CRT_SECURE_NO_WARNINGS
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
#include <io.h>


typedef struct {
    double i_pos[3];
    double size[3];
    double o_pos[3];
} Command;

#define EMIT_CHANNEL 1
#define RECEIVE_CHANNEL 2

static WbDeviceTag cam[2], receiver, emitter;
unsigned short width, height;
static double distance_arm0_platform = DIS_ARM_OBJ;

void struct_read_file(int n, Command* tk)
{
    int count = 0;
    char c;
    char filename[100];
    char buf[100];
    snprintf(filename, sizeof(filename), "../%d.txt", n);
    FILE* fp;
    while (!(fp = fopen(filename, "r")))
    {
        if (count == 0) {

            printf("Waiting for task %d\n", n);
            count++;
        }
        step();
    }
    printf("%s found.\n", filename);
    fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf", 
        &tk->i_pos[0], &tk->i_pos[1], &tk->size[0], &tk->size[1], &tk->size[2], &tk->o_pos[0], &tk->o_pos[1], &tk->o_pos[2]);
    printf("%f %f %f %f %f %f %f %f\n", 
        tk->i_pos[0], tk->i_pos[1], tk->size[0], tk->size[1], tk->size[2], tk->o_pos[0], tk->o_pos[1], tk->o_pos[2]);

    fclose(fp);

}

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



    int print = 0;
    int wait = 1;
    while (true) {
        step();
        int i = 1;
        Command* buf = (Command*)malloc(sizeof(Command));

        for (int i = 1; i <= 4; i++) {
            struct_read_file(i, buf);
            printf("ix = %f\n", buf->i_pos[0]);
            if (grasp_and_place(buf->i_pos, buf->size, buf->o_pos)) {
                wait++;
            }
            else {
                perror("Grasp failed!\n");
                i--;
            }
           
        }
        free(buf);




    }

    wb_robot_cleanup();

    return 0;
}




/* is there at least one packet in the receiver's queue ? */
//if (wb_receiver_get_queue_length(receiver) > 0) {
//    /* read current packet's data */
//    const char* head = wb_receiver_get_data(receiver);
//    void* p = wb_receiver_next_packet(receiver);
//    Command* command = (Command*)p;
//    if (grasp_and_place(&command->i_pos, &command->size, &command->o_pos)) {
//        wait++;
//        char message[128];
//        sprintf(message, "%d", wait);
//        wb_emitter_send(emitter, message, strlen(message) + 1);
//    }
//}

//char message[128];
//sprintf(message, "%d", wait);
//wb_emitter_send(emitter, message, strlen(message) + 1);