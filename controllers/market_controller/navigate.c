/*
Created on Saturday Apr 5 2020
Author : yu du
Email : yuduseu@gmail.com
Last edit date :
*/

#include "navigate.h"
#define LEFT 1
#define RIGHT -1

double PID(double err);
double Kp = 0.5;
double Ki = 0.01;
double Kd = 0.01;
double last_err = 0.0;
double llast_err = 0.0;

// may be revised
static double tf_pts[4][2] = {{ 1.1,  1.1},   // between 0 1
                                        {-1.1,  1.1},   // between 1 2
                                        {-1.2, -1.1},   // between 2 3
                                        { 1.1, -1.1} }; // between 3 4
#define FK_OFFSET 0.375
static double fake_tf_pts[4][2][2] = { { { 1.0,  1.0 - FK_OFFSET}, { 1.0 - FK_OFFSET,  1.0} },  // between 0 1
                                       { {-1.0 + FK_OFFSET,  1.0}, {-1.0,  1.0 - FK_OFFSET} },   // between 1 2
                                       { {-1.0, -1.0 + FK_OFFSET}, {-1.0 + FK_OFFSET, -1.0} },  // between 2 3
                                       { { 1.0 - FK_OFFSET, -1.0}, { 1.0, -1.0 + FK_OFFSET} } }; // between 3 4


//static double ori_clockwise[4] = { M_PI_2, 0, -M_PI_2, M_PI }; // orientation for grasp
//static double ori_anticlockwise[4] = { M_PI, M_PI_2, 0, -M_PI_2 }; // for shelf placement, use the opposite

static double ori_clockwise[4] = { M_PI, M_PI_2, 0, -M_PI_2 }; // orientation for grasp
static double ori_clockwise_last[4] = { -M_PI_2, M_PI, M_PI_2, 0 }; // orientation for last status
static double ori_anticlockwise[4] = { M_PI_2, 0, -M_PI_2, M_PI }; // for shelf placement, use the opposite
static double ori_anticlockwise_last[4] = { 0, -M_PI_2, M_PI, M_PI_2 };
static double dis_p2p = 2.0; 
static double distance_arm0_platform = DIS_ARM_OBJ;
static double distance_arm0_robot_center = 0.189;
static double num_interval = 2.0;
//static double offset = distance_arm0_platform + distance_arm0_robot_center;


void go_to(double x, double z, double a) {
    base_goto_set_target(x, z, a);
    while (!base_goto_reached()) {
        base_goto_run();
        step();
        step();
    }
    base_reset();
}

static double get_alpha() {
    const double* compass_raw_values = wb_compass_get_values(compass);

    // compute 2d vectors
    Vector2 v_front = { compass_raw_values[0], compass_raw_values[1] };
    Vector2 v_right = { -v_front.v, v_front.u };
    Vector2 v_north = { 1.0, 0.0 };

    // compute absolute angle & delta with the delta with the target angle
    double theta = vector2_angle(&v_front, &v_north);
    // clockwise increase
    return theta; // should be in (-pi, pi)
}



int get_partition(double x, double z) {
    if(x >= fabs(z)) {
        return 0;
    }
    else if (z > fabs(x)) {
        return 1;
    }
    else if (fabs(z) <= -x) {
        return 2;
    }
    else if (-z > fabs(x)) {
        return 3;
    }
    else {
        return -1;
    }
}


double distance_clockwise(double rbt_x, double rbt_z, double obj_x, double obj_z) {
    int obj_partition = get_partition(obj_x, obj_z);
    int rbt_partition = get_partition(rbt_x, rbt_z);
    double dis = 0.0;
    if (obj_partition == rbt_partition) {
        dis = sqrt(pow(rbt_x - obj_x, 2)
                    + pow(rbt_z - obj_z, 2));
        return dis;
    }
    
    dis = sqrt(pow(rbt_x - tf_pts[rbt_partition][0], 2)
                + pow(rbt_z - tf_pts[rbt_partition][1], 2));
    if (obj_partition < rbt_partition)
        obj_partition += 4;
    dis += sqrt(pow(obj_x - tf_pts[(obj_partition - 1)%4][0], 2)
        + pow(obj_z - tf_pts[(obj_partition - 1)%4][1], 2));
    for (int i = rbt_partition; i < obj_partition - 1; i++) {
        dis += dis_p2p;
    }
    return dis;
}


double distance_anticlockwise(double rbt_x, double rbt_z, double obj_x, double obj_z) {
    // for anti-clockwise from robot to object
    // it is clockwise from object to robot 
    double dis = distance_clockwise(obj_x, obj_z, rbt_x, rbt_z);
    return dis;
}


int go_to_grasp(double ox, double oz) {
    double offset = distance_arm0_platform + distance_arm0_robot_center;
    const double* gps_raw_values = wb_gps_get_values(gps);
    //const double* compass_raw_values = wb_compass_get_values(compass);
    double rx = gps_raw_values[0];
    double rz = gps_raw_values[2];
    printf("ox=%f\toz=%f\n", ox, oz);
    int rbt_partition = get_partition(rx, rz);
    int obj_partition = get_partition(ox, oz);
    if (rbt_partition == -1) {
        perror("Invalid robot partition!\n");
        return -1;
    }
    if (obj_partition == -1) {
        perror("Invalid object partition!\n");
        return -1;
    }
    printf("Robot partition=%d, Object partition=%d\n", rbt_partition, obj_partition);
    double alpha = 0.0;
    if (obj_partition == 0) {
        alpha = M_PI;
        ox += offset;
    }
    else if (obj_partition == 1) {
        alpha = M_PI_2;
        oz += offset;
    }
    else if (obj_partition == 2) {
        alpha = 0;
        ox -= offset;
    }
    else if (obj_partition == 3) {
        alpha = -M_PI_2;
        oz -= offset;
    }
    if (rbt_partition == obj_partition) {
        go_to(ox, oz, alpha);
        return 0;
    }
    double dis_clockwise = distance_clockwise(rx, rz, ox, oz);
    double dis_anticlockwise = distance_anticlockwise(rx, rz, ox, oz);
    printf("Distance clockwise=%f, Distance anticlockwise=%f\n", dis_clockwise, dis_anticlockwise);
    bool clockwise = true;
    if (dis_anticlockwise < dis_clockwise)
        clockwise = false;

    if (clockwise) {
        if (obj_partition < rbt_partition)
            obj_partition += 4;
        for (int i = rbt_partition; i < obj_partition; i++) {
            go_to(tf_pts[i % 4][0], tf_pts[i % 4][1], ori_clockwise[(i - 1) % 4]);
            go_to(tf_pts[i % 4][0], tf_pts[i % 4][1], ori_clockwise[i % 4]);
        }
    }
    else {
        if (rbt_partition < obj_partition)
            rbt_partition += 4;
        for (int i = rbt_partition; i > obj_partition; i--) {
            go_to(tf_pts[(i - 1) % 4][0], tf_pts[(i - 1) % 4][1], ori_anticlockwise[i % 4]);
            go_to(tf_pts[(i - 1) % 4][0], tf_pts[(i - 1) % 4][1], ori_anticlockwise[(i - 1) % 4]);
        }
    }
    go_to(ox, oz, alpha);
    return 0;

}



int go_to_shelf(double ox, double oz) {
    double offset = distance_arm0_platform + distance_arm0_robot_center;
    const double* gps_raw_values = wb_gps_get_values(gps);
    //const double* compass_raw_values = wb_compass_get_values(compass);
    double rx = gps_raw_values[0];
    double rz = gps_raw_values[2];
    int rbt_partition = get_partition(rx, rz);
    int obj_partition = get_partition(ox, oz);
    if (rbt_partition == -1) {
        perror("Invalid robot partition!\n");
        return -1;
    }
    if (obj_partition == -1) {
        perror("Invalid object partition!\n");
        return -1;
    }
    printf("Robot partition=%d, Object partition=%d\n", rbt_partition, obj_partition);
    double alpha = 0.0;
    if (obj_partition == 0) {
        alpha = -M_PI;
        ox -= offset;
    }
    else if (obj_partition == 1) {
        alpha = -M_PI_2;
        oz -= offset;
    }
    else if (obj_partition == 2) {
        alpha = 0;
        ox += offset;
    }
    else if (obj_partition == 3) {
        alpha = M_PI_2;
        oz += offset;
    }
    if (rbt_partition == obj_partition) {
        go_to(ox, oz, alpha);
        return 0;
    }
    double dis_clockwise = distance_clockwise(rx, rz, ox, oz);
    double dis_anticlockwise = distance_anticlockwise(rx, rz, ox, oz);
    bool clockwise = true;
    if (dis_anticlockwise < dis_clockwise)
        clockwise = false;

    if (clockwise) {
        if (obj_partition < rbt_partition)
            obj_partition += 4;
        for (int i = rbt_partition; i < obj_partition; i++) {
            go_to(tf_pts[i % 4][0], tf_pts[i % 4][1], ori_clockwise[(i - 1) % 4]);
            go_to(tf_pts[i % 4][0], tf_pts[i % 4][1], ori_clockwise[i % 4]);
        }
    }
    else {
        if (rbt_partition < obj_partition)
            rbt_partition += 4;
        for (int i = rbt_partition; i > obj_partition; i--) {
            go_to(tf_pts[(i - 1) % 4][0], tf_pts[(i - 1) % 4][1], ori_anticlockwise[i % 4]);
            go_to(tf_pts[(i - 1) % 4][0], tf_pts[(i - 1) % 4][1], ori_anticlockwise[(i - 1) % 4]);
        }
    }
    go_to(ox, oz, alpha);
    return 0;
}


Vector3 differentiation(double ix, double iz, double ia, double ox, double oz, double oa) {
    Vector3 d;
    d.u = (ox - ix) / num_interval;
    d.v = (oz - iz) / num_interval;
    d.w = (oa - ia) / num_interval;
    return d;
}


void CP_planning(double ix, double iz, double ia, double ox, double oz, double oa) {
    Vector3 d = differentiation(ix, iz, ia, ox, oz, oa);
    for (int i = 1; i <= num_interval; i++) {
        go_to(ix + i * d.u, iz + i * d.v, ia + i * d.w);
    }

}

void curve_turning(int tf_pt) {

    go_to(fake_tf_pts[tf_pt][0][0], fake_tf_pts[tf_pt][0][1], ori_clockwise_last[tf_pt]);
    CP_planning(fake_tf_pts[tf_pt][0][0], fake_tf_pts[tf_pt][0][1], ori_clockwise_last[tf_pt],
        fake_tf_pts[tf_pt][1][0], fake_tf_pts[tf_pt][1][1], ori_clockwise[tf_pt]);
}

void go_to_point(double ox, double oz) {
    double offset = distance_arm0_platform + distance_arm0_robot_center;
    const double* gps_raw_values = wb_gps_get_values(gps);
    //const double* compass_raw_values = wb_compass_get_values(compass);
    double rx = gps_raw_values[0];
    double rz = gps_raw_values[2];
    printf("ox=%f\toz=%f\n", ox, oz);
    int rbt_partition = get_partition(rx, rz);
    int obj_partition = get_partition(ox, oz);
    if (rbt_partition == -1) {
        perror("Invalid robot partition!\n");
        return -1;
    }
    if (obj_partition == -1) {
        perror("Invalid object partition!\n");
        return -1;
    }
    printf("Robot partition=%d, Object partition=%d\n", rbt_partition, obj_partition);
    double alpha = 0.0;
    if (obj_partition == 0) {
        alpha = -M_PI;
        ox += offset;
    }
    else if (obj_partition == 1) {
        alpha = M_PI_2;
        oz += offset;
    }
    else if (obj_partition == 2) {
        alpha = 0;
        ox -= offset;
    }
    else if (obj_partition == 3) {
        alpha = -M_PI_2;
        oz -= offset;
    }
    if (rbt_partition == obj_partition) {
        go_to(ox, oz, alpha);
        return 0;
    }
    double dis_clockwise = distance_clockwise(rx, rz, ox, oz);
    double dis_anticlockwise = distance_anticlockwise(rx, rz, ox, oz);
    printf("Distance clockwise=%f, Distance anticlockwise=%f\n", dis_clockwise, dis_anticlockwise);
    bool clockwise = true;
    if (dis_anticlockwise < dis_clockwise)
        clockwise = false;

    if (clockwise) {
        if (obj_partition < rbt_partition)
            obj_partition += 4;
        for (int i = rbt_partition; i < obj_partition; i++) {
            curve_turning(i % 4);
        }
    }
    else {
        if (rbt_partition < obj_partition)
            rbt_partition += 4;
        for (int i = rbt_partition; i > obj_partition; i--) {
            curve_turning((i-1) % 4);
        }
    }
    gps_raw_values = wb_gps_get_values(gps);
    double ix = gps_raw_values[0];
    double iz = gps_raw_values[2];
    double cur_alpha = get_alpha();
    CP_planning(ix, iz, cur_alpha, ox, oz, alpha);
    return 0;
}

void go_to_translation(double ox, double oz, int status) {

    double offset = distance_arm0_platform + distance_arm0_robot_center;
    const double* gps_raw_values = wb_gps_get_values(gps);
    double cur_alpha = get_alpha();
    double rx = gps_raw_values[0];
    double rz = gps_raw_values[2];
    printf("ox=%f\toz=%f\n", ox, oz);
    int rbt_partition = get_partition(rx, rz);
    int obj_partition = get_partition(ox, oz);
    if (rbt_partition == -1) {
        perror("Invalid robot partition!\n");
        return -1;
    }
    if (obj_partition == -1) {
        perror("Invalid object partition!\n");
        return -1;
    }
    if (status == -1) {
        offset += 0.2;
    }
    printf("Robot partition=%d, Object partition=%d\n", rbt_partition, obj_partition);
    double alpha = 0.0;
    if (obj_partition == 0) {
        if (status == 1)
            alpha = -M_PI;
        else
            alpha = 0;
        ox += status * (offset + DIS_APPROACH);


    }
    else if (obj_partition == 1) {
        alpha = status * M_PI_2;
        oz += status * (offset + DIS_APPROACH);

    }
    else if (obj_partition == 2) {
        if (status == 1)
            alpha = 0;
        else
            alpha = M_PI;
        ox -= status * (offset + DIS_APPROACH);

    }
    else if (obj_partition == 3) {
        alpha = -status * M_PI_2;
        oz -= status * (offset + DIS_APPROACH);

    }
    if (rbt_partition == obj_partition) {
        go_to(ox, oz, alpha);
        return 0;
    }
    double dis_clockwise = distance_clockwise(rx, rz, ox, oz);
    double dis_anticlockwise = distance_anticlockwise(rx, rz, ox, oz);
    printf("Distance clockwise=%f, Distance anticlockwise=%f\n", dis_clockwise, dis_anticlockwise);
    bool clockwise = true;
    if (dis_anticlockwise < dis_clockwise)
        clockwise = false;

    if (clockwise) {
        if (obj_partition < rbt_partition)
            obj_partition += 4;
        for (int i = rbt_partition; i < obj_partition; i++) {
            go_to(tf_pts[i % 4][0], tf_pts[i % 4][1], cur_alpha);
        }
    }
    else {
        if (rbt_partition < obj_partition)
            rbt_partition += 4;
        for (int i = rbt_partition; i > obj_partition; i--) {
            go_to(tf_pts[(i - 1) % 4][0], tf_pts[(i - 1) % 4][1], cur_alpha);
        }
    }
    gps_raw_values = wb_gps_get_values(gps);
    double ix = gps_raw_values[0];
    double iz = gps_raw_values[2];
    go_to(ox, oz, cur_alpha);
    //turning(alpha);
    //passive_wait(1.0);
    go_to(ox, oz, alpha);
    // approach from the radial direction
    //go_to(target_x, target_z, alpha);
}



void approach(double depth, double height, int status) {
    // depth: depth of the object (length in direction forward the robot)
    // height: height of the object
    double gap = 0.02;
    double dis_center_gripper = 0.45; // should be defined upon status
    double offset = 0.0;
        - dis_center_gripper - gap - depth / 2;
    if (depth > 0.1) {
        offset = DIS_APPROACH + distance_arm0_platform + distance_arm0_robot_center
            - dis_center_gripper - gap - depth/2;
    }
    else {
        offset = DIS_APPROACH + distance_arm0_platform + distance_arm0_robot_center
            - dis_center_gripper - 0.075;
    }

    const double* gps_raw_values = wb_gps_get_values(gps);
    double cur_alpha = get_alpha();
    double rx = gps_raw_values[0];
    double rz = gps_raw_values[2];
    double ox = rx;
    double oz = rz;
    int rbt_partition = get_partition(rx, rz);
    if (rbt_partition == -1) {
        perror("Invalid robot partition!\n");
        return -1;
    }
    if (status == -1) {
        offset += 0.18;
        if (depth <= 0.1)
            offset -= 0.04;
    }
    double alpha = 0.0;
    if (rbt_partition == 0) {
        if (status == 1)
            alpha = status * M_PI;
        else
            alpha = 0;
        ox -= status * offset;

    }
    else if (rbt_partition == 1) {
        alpha = status * M_PI_2;
        oz -= status * offset;
    }
    else if (rbt_partition == 2) {
        if (status == 1)
            alpha = 0;
        else
            alpha = M_PI;
        ox += status * offset;
    }
    else if (rbt_partition == 3) {
        alpha = -status * M_PI_2;
        oz += status * offset;
    }
    printf("rx = %f\t rz = %f\n", rx, rz);
    printf("ox = %f\t oz = %f\n", ox, oz);
    go_to(ox, oz, cur_alpha);
}

void backup(double distance) {

    const double* gps_raw_values = wb_gps_get_values(gps);

    double rx = gps_raw_values[0];
    double rz = gps_raw_values[2];
    double ix = rx;
    double iz = rz;

    base_backwards();
    double dis;
    do {
        step();
        gps_raw_values = wb_gps_get_values(gps);
        rx = gps_raw_values[0];
        rz = gps_raw_values[2];
        dis = sqrt(pow(ix - rx, 2) + pow(iz - rz, 2));
    } while (fabs(dis-distance) > 0.005);
    base_reset();
    

}

void turning(double alpha) {
    if (alpha < 0) {
        alpha += 2 * M_PI;
    }
    double cur_alpha;
    double delta;
    int dir;
    double k = 0.0;
    do {
        step();
        cur_alpha = get_alpha();
        if (cur_alpha < 0) {
            cur_alpha += 2 * M_PI;
        }
        delta = alpha - cur_alpha;
        // delta > 0 turn right

        //if (delta > M_PI) {
        //    delta = delta - 2 * M_PI;
        //}
        //k -= PID(delta);
        //k = bound(k, -0.5, 0.5);
        //base_set_turn_speed(k);


        if (fabs(delta) > M_PI) {
            if (delta > 0) {
                dir = LEFT;
                //delta = delta - 2 * M_PI;
            }
            else {
                dir = RIGHT;
                //delta = 2 * M_PI + delta;
            }

            //delta = 2 * M_PI - fabs(delta);
        }
        else {
            if (delta > 0) {
                dir = RIGHT;
            }
            else {
                dir = LEFT;
            }
        }

        if (dir == LEFT) {
            base_set_turn_speed(0.5);
        }
        else {
            base_set_turn_speed(-0.5);
        }
    } while (fabs(delta) > 0.005);
    base_reset();
}


double PID(double err) {
    double delta = Kp * (err - last_err) + Ki * err + Kd * (err - 2 * last_err + llast_err);
    return delta;
}