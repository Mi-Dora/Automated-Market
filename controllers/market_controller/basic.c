/*
Created on Saturday Apr 5 2020
Author : yu du
Email : yuduseu@gmail.com
Last edit date :
*/

#include "basic.h"

void step() {
    if (wb_robot_step(TIME_STEP) == -1) {
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
    }
}


void passive_wait(double sec) {
    double start_time = wb_robot_get_time();
    do {
        step();
    } while (start_time + sec > wb_robot_get_time());
}

