#pragma once
#ifndef PROCESS_CONTROL
#define PROCESS_CONTROL
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/camera_recognition_object.h>
#include <webots/supervisor.h>

typedef struct ID
{
    int name; //category
    int objectID;//key
    double id_gps_position[3];
}IDlist;

typedef struct Pnode
{
    IDlist* data;
    struct Pnode* next;
}Node;

typedef struct hash_table
{
    int size;
    int length;
    struct Pnode* head;
}Hash_table;

int namejudge(char* name);
double get_bearing_in_degrees(WbDeviceTag tag);
Hash_table* Create_Table();
Node* lookup(Hash_table* h, int objectID, int name);
void Insert(Hash_table* h, IDlist k);
void destroy_table(Hash_table* h);
void print_Table(Hash_table* h);

#endif