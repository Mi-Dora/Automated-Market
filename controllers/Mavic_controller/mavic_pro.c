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
#include <webots/receiver.h>
#include <webots/emitter.h>


#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))
#define M_PI 3.1415926535
#define FILE_BUFFER_LENGTH 100000

/*---------------------------------------category---------------------------------------------------------------------*/
#define BEER_BOTTLE 1
#define JAM_JAR 2
#define CAN 3
#define WATER_BOTTLE 4
#define HONEY_JAR 5
#define CEREAL_BOX 6
#define BISCUIT_BOX 7

#define EMIT_CHANNEL 8
#define RECEIVE_CHANNEL 9

typedef struct trans
{
    double i_pos[2];
    double i_size[3];   //height, width, depth
    double o_pos[3];
}Trans2Youbot;

typedef struct Vector0 {
    double u;
    double v;
} Vector2;

typedef struct Vector {
    double u;
    double v;
    double w;
} Vector3;

/*----------------------------------------the hash table--------------------------------------------------------------*/
typedef struct ID
{
    int name; //category
    int objectID;//key
    double color[3];
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


/*-----------------------------------------device used---------------------------------------------------------------*/
static WbDeviceTag camera;
static WbDeviceTag front_left_led;
static WbDeviceTag front_right_led;
static WbDeviceTag imu;
static WbDeviceTag gps;
static WbDeviceTag compass;
static WbDeviceTag gyro;
static WbDeviceTag camera_motor[3];
static WbDeviceTag motors[4];
static WbDeviceTag emitter;
static WbDeviceTag receiver;


/*-----------------------------------------------parameter used------------------------------------------------------*/
static const double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
static const double k_vertical_offset = 0.01;   // Vertical offset where the robot actually targets to stabilize itself.
static const double k_vertical_p = 6.0;        // P constant of the vertical PID.
static const double k_roll_p = 50.0;           // P constant of the roll PID.
static const double k_pitch_p = 30.0;          // P constant of the pitch PID.

static int time_step = 0;

static double roll;
static double pitch;

static double red_x_F;
static double altitude_y;
static double green_z;

static double roll_acceleration;
static double pitch_acceleration;
static double yaw_acceleration;

static double initial_x;
static double initial_z;

static double new_target_position_x;
static double new_target_position_y;
static double new_target_position_z;

static const double* north;
static double angle;
static bool finish_A = false;
static bool finish_B = false;
static bool finish_C = false;
static bool finish_D = false;
static int run;
//length, width, height, radius
//to
//height, width, depth
/*--------------------------------------------------size list--------------------------------------------------------*/
static double initializelist[7][3] = { {0.143,0.031,0.031},{0.115,0.045,0.045},
    {0.122,0.0318,0.0318},{0.31,0.044,0.044},{0.115,0.045,0.045},
    {0.3,0.08,0.2},{0.18,0.14,0.08} };

/*--------------------------------------------------shelf list------------------------------------------------------*/
//1. region(4)
//2. width(8)
//3. height(2)--4 conditions
static bool shelf[4][8][2] = { 0 };

/*--------------------------------------------------bool set function---------------------------------------------------*/
void shelf_add(const double* gps_position)
{
    if (gps_position[0] > fabs(gps_position[2]))  //R0
    {
        if (fabs(gps_position[0]) > 1.25)         //on the shelf
        {
            for (int cell = 0; cell < 8; ++cell)
            {
                double cell_position_min = -1.0 + 0.25 * cell;
                double cell_position_max = -1.0 + 0.25 * (cell + 1.0);
                if (fabs(gps_position[2]) > cell_position_min && fabs(gps_position[2]) < cell_position_max)
                {
                    if (gps_position[1] > 0 && gps_position[1] <= 0.4)
                    {
                        shelf[0][cell][0] = true;
                    }
                    if (gps_position[1] > 0.4 && gps_position[1] < 0.6)
                    {
                        shelf[0][cell][1] = true;
                    }
                }
            }
        }
    }
    if (gps_position[2] > fabs(gps_position[0]))  //R1
    {
        if (fabs(gps_position[2]) > 1.25)         //on the shelf
        {
            for (int cell = 0; cell < 8; ++cell)
            {
                double cell_position_min = -1.0 + 0.25 * cell;
                double cell_position_max = -1.0 + 0.25 * (cell + 1.0);
                if (fabs(gps_position[0]) > cell_position_min && fabs(gps_position[0]) < cell_position_max)
                {
                    if (gps_position[1] > 0 && gps_position[1] <= 0.2)
                    {
                        shelf[1][cell][0] = true;
                    }
                    if (gps_position[1] > 0.2 && gps_position[1] < 0.6)
                    {
                        shelf[1][cell][1] = true;
                    }
                }
            }
        }
    }
    if (-gps_position[0] > fabs(gps_position[2])) //R2
    {
        if (fabs(gps_position[0]) > 1.25)         //on the shelf
        {
            for (int cell = 0; cell < 8; ++cell)
            {
                double cell_position_min = -1.0 + 0.25 * cell;
                double cell_position_max = -1.0 + 0.25 * (cell + 1.0);
                if (fabs(gps_position[2]) > cell_position_min && fabs(gps_position[2]) < cell_position_max)
                {
                    if (gps_position[1] > 0 && gps_position[1] <= 0.4)
                    {
                        shelf[2][cell][0] = true;
                    }
                    if (gps_position[1] > 0.4 && gps_position[1] < 0.8)
                    {
                        shelf[2][cell][1] = true;
                    }
                }
            }
        }
    }
    if (-gps_position[2] > fabs(gps_position[0]))  //R3
    {
        if (fabs(gps_position[2]) > 1.25)         //on the shelf
        {
            for (int cell = 0; cell < 8; ++cell)
            {
                double cell_position_min = -1.0 + 0.25 * cell;
                double cell_position_max = -1.0 + 0.25 * (cell + 1.0);
                if (fabs(gps_position[0]) > cell_position_min && fabs(gps_position[0]) < cell_position_max)
                {
                    if (gps_position[1] > 0 && gps_position[1] <= 0.2)
                    {
                        shelf[3][cell][0] = true;
                    }
                    if (gps_position[1] > 0.2 && gps_position[1] < 0.6)
                    {
                        shelf[3][cell][1] = true;
                    }
                }
            }
        }
    }
}


/*-----------------------------------------------bool search function-----------------------------------------------*/

bool shelf_search(int x, int y, int z)
{
    bool in_shelf = shelf[x][y][z];
    return in_shelf;
}

/*bool to gps position function*/

void bool2cor(Vector3* pos, int x, int y, int z)
{
    double x1 = 0.0;
    double y1 = 0.0;
    double z1 = 0.0;
    if (z == 0)
    {
        if (x == 0)
        {
            z1 = 2.0 - 0.25 / 2.0;
            x1 = -0.875 + y * 0.25;
            y1 = 0.2;
        }
        if (x == 2)
        {
            z1 = -2.0 + 0.25 / 2.0;
            x1 = -0.875 + y * 0.25;
            y1 = 0.2;
        }
        if (x == 1)
        {
            x1 = 2.0 - 0.25 / 2.0;
            z1 = -0.875 + y * 0.25;
            y1 = 0.1;
        }
        if (x == 3)
        {
            x1 = -2.0 + 0.25 / 2.0;
            z1 = -0.875 + y * 0.25;
            y1 = 0.1;
        }
    }
    if (z == 1)
    {
        if (x == 0)
        {
            z1 = 2.0 - 0.25 / 2.0;
            y1 = 0.5;
            x1 = -2.0 + 0.25 / 2.0;
        }
        if (x == 2)
        {
            z1 = -2.0 + 0.25 / 2.0;
            y1 = 0.6;
            x1 = -2.0 + 0.25 / 2.0;
        }
        if (x == 1)
        {
            x1 = 2.0 - 0.25 / 2.0;
            y1 = 0.4;
            z1 = -0.875 + y * 0.25;
        }
        if (x == 3)
        {
            x1 = -2.0 + 0.25 / 2.0;
            y1 = 0.4;
            z1 = -0.875 + y * 0.25;
        }
    }
    pos->u = x1;
    pos->v = y1;
    pos->w = z1;
}

/*-----------------------------------------------print bool shelf array---------------------------------------------*/
void output_shelf(bool shelf_array[][8][2])
{
    int x, y, z;
    FILE* shelf_w;
    shelf_w = fopen("shelf.txt", "w");
    if (shelf_w == NULL)
    {
        printf("fail to open the file!\n");
        return;
    }
    for (x = 0; x < 4; ++x)
    {
        for (y = 0; y < 8; ++y)
        {
            for (z = 0; z < 2; ++z)
            {
                fprintf(shelf_w, "%d ", shelf_array[x][y][z]);
            }
            fprintf(shelf_w, "\n");
        }
        fprintf(shelf_w, "\n");
    }
    fclose(shelf_w);
}

/*-----------------------------------------------decide region------------------------------------------------------*/
int region(double x, double z)
{
    int num_region = 0;
    if (x > fabs(z))
    {
        num_region = 0;
    }
    if (z > fabs(x))
    {
        num_region = 1;
    }
    if (-x > fabs(z))
    {
        num_region = 2;
    }
    if (-z > fabs(x))
    {
        num_region = 3;
    }
    return num_region;
}

bool region_find(Vector2* p, double x, double z, int r)
{
    bool judge = false;
    int re = region(x, z);
    if ((re == 0 && r == 2) || (re == 1 && r == 1) || (re == 2 && r == 0) || (re == 3 && r == 3))
    {
        p->u = x;
        p->v = z;
        judge = true;
    }
    return judge;
}

void lookup4trans(Hash_table* h, Trans2Youbot* tk, int order, int order_height)
{
    Vector3* pos1 = (Vector3*)malloc(sizeof(Vector3));
    bool judgement_existence = true;
    if (order == 2)             //red bisuit box(7) + red can(3)
    {
        bool region_judge = false;
        Vector2* pos0 = (Vector2*)malloc(sizeof(Vector2));
        Node* p = (Node*)malloc(sizeof(Node));
        p->data = (IDlist*)malloc(sizeof(IDlist));
        if (order_height == 1)
            p = h->head[6].next;   //biscuit box
        if (order_height == 0)
            p = h->head[2].next;   //red can

        while (p && !region_judge)
        {
            printf("WHY1\n");
            //printf("%d \n", p->data->name);
            double i_position_x = p->data->id_gps_position[0];
            double i_position_z = p->data->id_gps_position[2];
            region_judge = region_find(pos0, i_position_x, i_position_z, order);
            p = p->next;
        }
        if (order_height == 1)
            p = h->head[6].next;   //biscuit box
        if (order_height == 0)
            p = h->head[2].next;   //red can
        while (p && !finish_A)
        {
            printf("WHY2\n");
            double i_position_x = p->data->id_gps_position[0];
            double i_position_z = p->data->id_gps_position[2];
            double color_tag = p->data->color[0];
            if (fabs(i_position_x) <= 1.25 && fabs(i_position_z) <= 1.25 && (color_tag))      //wait for picking and it is red
            {
                printf("WHY3\n");
                for (int i1 = 0; i1 < 8; ++i1)        //judge the shelf
                {
                    judgement_existence = shelf_search(2, i1, order_height);
                    if (!judgement_existence && !finish_A)       //empty
                    {
                        printf("WHY4\n");
                        bool2cor(pos1, 2, i1, order_height);//o_pos
                        tk->o_pos[0] = pos1->u;
                        tk->o_pos[1] = pos1->v;
                        tk->o_pos[2] = pos1->w;
                        for (int k2 = 0; k2 < 3; ++k2)//size
                        {
                            if (order_height)
                            {
                                printf("WHY5\n");
                                tk->i_size[k2] = initializelist[6][k2];
                            }
                            if (!order_height)
                            {
                                printf("WHY5\n");
                                tk->i_size[k2] = initializelist[2][k2];
                            }
                        }
                        if (region_judge)
                        {
                            tk->i_pos[0] = pos0->u;
                            tk->i_pos[1] = pos0->v;
                            finish_A = true;
                        }
                        else
                        {
                            tk->i_pos[0] = i_position_x;
                            tk->i_pos[1] = i_position_z;
                            finish_A = true;
                        }

                    }
                }
            }
            p = p->next;
        }

        //   free(p->data);
         //  p->data = NULL;
        //   free(p);
        //   p = NULL;
        free(pos0);
        pos0 = NULL;
        //   free(pos1);
        //   pos1 = NULL;
    }
    if (order == 1)            // red cereal box(6) + beer bottle(1)
    {
        bool region_judge = false;
        Vector2* pos0 = (Vector2*)malloc(sizeof(Vector2));
        Node* p = (Node*)malloc(sizeof(Node));
        p->data = (IDlist*)malloc(sizeof(IDlist));
        if (order_height == 1)
            p = h->head[5].next;   //red cereal box
        if (order_height == 0)
            p = h->head[0].next;   //beer bottle
        while (p && !region_judge)
        {
            double i_position_x = p->data->id_gps_position[0];
            double i_position_z = p->data->id_gps_position[2];
            region_judge = region_find(pos0, i_position_x, i_position_z, order);
            p = p->next;
        }
        if (order_height == 1)
            p = h->head[5].next;   //red cereal box
        if (order_height == 0)
            p = h->head[0].next;   //beer bottle
        while (p && !finish_B)
        {
            double i_position_x = p->data->id_gps_position[0];
            double i_position_z = p->data->id_gps_position[2];
            double color_tag_1 = p->data->color[0] + (double)order_height;
            if (fabs(i_position_x) <= 1.25 && fabs(i_position_z) <= 1.25 && ((int)color_tag_1 == 2 || order_height == 0))      //wait for picking and it is red
            {
                for (int i1 = 0; i1 < 8; ++i1)        //judge the shelf
                {
                    judgement_existence = shelf_search(1, i1, order_height);
                    if (!judgement_existence && !finish_B)       //empty
                    {
                        bool2cor(pos1, 1, i1, order_height);//o_pos
                        tk->o_pos[0] = pos1->u;
                        tk->o_pos[1] = pos1->v;
                        tk->o_pos[2] = pos1->w;
                        for (int k2 = 0; k2 < 3; ++k2)//size
                        {
                            if (order_height)
                            {
                                tk->i_size[k2] = initializelist[5][k2];
                            }
                            if (!order_height)
                            {
                                tk->i_size[k2] = initializelist[0][k2];
                            }
                        }
                        if (region_judge)
                        {
                            tk->i_pos[0] = pos0->u;
                            tk->i_pos[1] = pos0->v;
                            finish_B = true;
                        }
                        else
                        {
                            tk->i_pos[0] = i_position_x;
                            tk->i_pos[1] = i_position_z;
                            finish_B = true;
                        }


                    }
                }
            }
            p = p->next;
        }
        //   free(p->data);
        //   p->data = NULL;
        //   free(p);
        //   p = NULL;
        free(pos0);
        pos0 = NULL;
        //  free(pos1);
       //   pos1 = NULL;
    }
    if (order == 0)            // blue cereal box(6) + green can(3)
    {
        bool region_judge = false;
        // Vector2* pos0 = NULL;
        Vector2* pos0 = (Vector2*)malloc(sizeof(Vector2));
        Node* p = (Node*)malloc(sizeof(Node));
        p->data = (IDlist*)malloc(sizeof(IDlist));
        if (order_height == 1)
            p = h->head[5].next;   //cereal box
        if (order_height == 0)
            p = h->head[2].next;   //can
        while (p && !region_judge)
        {
            //  printf("%d \n", p->data->name);
            double i_position_x = p->data->id_gps_position[0];
            double i_position_z = p->data->id_gps_position[2];
            printf("The value of x: %f\n", i_position_x);
            printf("The value of z: %f\n", i_position_z);
            region_judge = region_find(pos0, i_position_x, i_position_z, order);
            p = p->next;
        }
        if (order_height == 1)
            p = h->head[5].next;   //cereal box
        if (order_height == 0)
            p = h->head[2].next;   //can
        while (p && !finish_C)
        {
            printf("T'M HERE!\n");
            double i_position_x = p->data->id_gps_position[0];
            double i_position_z = p->data->id_gps_position[2];
            double color_tag_2 = p->data->color[0];
            if (fabs(i_position_x) <= 1.25 && fabs(i_position_z) <= 1.25 &&
                (((int)color_tag_2 == 0 && order_height == 1) || ((int)color_tag_2 == 0 && order_height == 0)))      //wait for picking and it is red
            {
                printf("WHY?_0\n");
                for (int i1 = 0; i1 < 8; ++i1)        //judge the shelf
                {
                    judgement_existence = shelf_search(2, i1, order_height);
                    printf("The value of judgement_existence is: %d\n", judgement_existence);
                    if (!judgement_existence && !finish_C)       //empty
                    {
                        bool2cor(pos1, 2, i1, order_height);//o_pos
                        tk->o_pos[0] = pos1->u;
                        tk->o_pos[1] = pos1->v;
                        tk->o_pos[2] = pos1->w;
                        for (int k2 = 0; k2 < 3; ++k2)//size
                        {
                            if (order_height)
                            {
                                tk->i_size[k2] = initializelist[5][k2];
                            }
                            if (!order_height)
                            {
                                tk->i_size[k2] = initializelist[2][k2];
                            }
                        }
                        //  printf("The size is: %f\n", tk->i_size[1]);
                        if (region_judge)
                        {
                            // printf("The value of region_judge is: %d\n", region_judge);
                            tk->i_pos[0] = pos0->u;
                            tk->i_pos[1] = pos0->v;
                            finish_C = true;
                        }
                        else
                        {
                            tk->i_pos[0] = i_position_x;
                            tk->i_pos[1] = i_position_z;
                            finish_C = true;
                        }

                    }
                }
            }
            p = p->next;
        }
        //printf("Final: %f\n", tk.i_pos[1]);
        //printf("Final: %f\n", tk.i_size[1]);
        //printf("Final: %f\n", tk.o_pos[1]);
     //   free(p->data);
     //   p->data = NULL;
    //    free(p);
    //    p = NULL;
        free(pos0);
        pos0 = NULL;
        //   free(pos1);
       //    pos1 = NULL;
    }
    if (order == 3)            // jam jar(2) + honey jar(5)   +   water bottle(4)
    {
        bool region_judge = false;
        Vector2* pos0 = (Vector2*)malloc(sizeof(Vector2));
        Node* p = (Node*)malloc(sizeof(Node));
        p->data = (IDlist*)malloc(sizeof(IDlist));
        bool jam = true;
        if (order_height == 1)
        {
            p = h->head[1].next;   //jam jar
            while (p && !region_judge)
            {
                double i_position_x = p->data->id_gps_position[0];
                double i_position_z = p->data->id_gps_position[2];
                region_judge = region_find(pos0, i_position_x, i_position_z, order);
                p = p->next;
            }
            p = h->head[1].next;
            while (p && !finish_D)
            {
                double i_position_x = p->data->id_gps_position[0];
                double i_position_z = p->data->id_gps_position[2];
                if (i_position_x <= 1.25 && i_position_z <= 1.25)      //wait for picking and it is red
                {
                    for (int i1 = 0; i1 < 8; ++i1)        //judge the shelf
                    {
                        judgement_existence = shelf_search(3, i1, order_height);
                        if (!judgement_existence && !finish_D)       //empty
                        {
                            bool2cor(pos1, 3, i1, order_height);//o_pos
                            tk->o_pos[0] = pos1->u;
                            tk->o_pos[1] = pos1->v;
                            tk->o_pos[2] = pos1->w;
                            for (int k2 = 0; k2 < 3; ++k2)//size
                            {
                                tk->i_size[k2] = initializelist[1][k2];
                            }
                            if (region_judge)
                            {
                                tk->i_pos[0] = pos0->u;
                                tk->i_pos[1] = pos0->v;
                                finish_D = true;
                            }
                            else
                            {
                                tk->i_pos[0] = i_position_x;
                                tk->i_pos[1] = i_position_z;
                                finish_D = true;
                            }

                        }
                    }
                }
                p = p->next;
            }
            if (!finish_D)
            {
                p = h->head[4].next;   //honey jar
                while (p && !region_judge)
                {
                    double i_position_x = p->data->id_gps_position[0];
                    double i_position_z = p->data->id_gps_position[2];
                    region_judge = region_find(pos0, i_position_x, i_position_z, order);
                    p = p->next;
                }
                p = h->head[4].next;
                while (p && !finish_D)
                {
                    double i_position_x = p->data->id_gps_position[0];
                    double i_position_z = p->data->id_gps_position[2];
                    if (i_position_x <= 1.25 && i_position_z <= 1.25)      //wait for picking and it is red
                    {
                        for (int i1 = 0; i1 < 8; ++i1)        //judge the shelf
                        {
                            judgement_existence = shelf_search(3, i1, order_height);
                            if (!judgement_existence && !finish_D)       //empty
                            {
                                bool2cor(pos1, 3, i1, order_height);//o_pos
                                tk->o_pos[0] = pos1->u;
                                tk->o_pos[1] = pos1->v;
                                tk->o_pos[2] = pos1->w;
                                for (int k2 = 0; k2 < 3; ++k2)//size
                                {
                                    tk->i_size[k2] = initializelist[4][k2];
                                }
                                if (region_judge)
                                {
                                    tk->i_pos[0] = pos0->u;
                                    tk->i_pos[1] = pos0->v;
                                    finish_D = true;
                                }
                                else
                                {
                                    tk->i_pos[0] = i_position_x;
                                    tk->i_pos[1] = i_position_z;
                                    finish_D = true;
                                }

                            }
                        }
                    }
                    p = p->next;
                }
            }

            //    free(p->data);
            //    p->data = NULL;
            //    free(p);
            //    p = NULL;
            //    free(pos0);
            //    pos0 = NULL;
             //   free(pos1);
             //   pos1 = NULL;
        }
        if (order_height == 0)
        {
            p = h->head[3].next;   //water bottle
            while (p && !region_judge)
            {
                double i_position_x = p->data->id_gps_position[0];
                double i_position_z = p->data->id_gps_position[2];
                region_judge = region_find(pos0, i_position_x, i_position_z, order);
                p = p->next;
            }
            p = h->head[3].next;
            while (p && !finish_D)
            {
                double i_position_x = p->data->id_gps_position[0];
                double i_position_z = p->data->id_gps_position[2];
                double color_tag_2 = p->data->color[0];
                if (i_position_x <= 1.25 && i_position_z <= 1.25)      //wait for picking and it is red
                {
                    for (int i1 = 0; i1 < 8; ++i1)        //judge the shelf
                    {
                        judgement_existence = shelf_search(2, i1, order_height);
                        if (!judgement_existence && !finish_D)       //empty
                        {
                            bool2cor(pos1, 2, i1, order_height);//o_pos
                            tk->o_pos[0] = pos1->u;
                            tk->o_pos[1] = pos1->v;
                            tk->o_pos[2] = pos1->w;
                            for (int k2 = 0; k2 < 3; ++k2)//size
                            {
                                tk->i_size[k2] = initializelist[3][k2];
                            }
                            if (region_judge)
                            {
                                tk->i_pos[0] = pos0->u;
                                tk->i_pos[1] = pos0->v;
                                finish_D = true;
                            }
                            else
                            {
                                tk->i_pos[0] = i_position_x;
                                tk->i_pos[1] = i_position_z;
                                finish_D = true;
                            }
                        }
                    }
                }
                p = p->next;
            }

            //   free(p->data);
            //   p->data = NULL;
            //   free(p);
            //   p = NULL;
            //   free(pos0);
           //    pos0 = NULL;
            //   free(pos1);
            //   pos1 = NULL;
        }
        free(pos0);
        pos0 = NULL;
    }
    free(pos1);
    pos1 = NULL;
    // printf("remain: %f\n", tk->i_size[1]);
}

/*-----------------------------------------------hash table funcion-------------------------------------------------*/
Hash_table* Create_Table()
{
    Hash_table* h = (Hash_table*)malloc(sizeof(Hash_table));
    if (h)
    {
        h->size = 7;
        h->head = (Node*)malloc((h->size) * sizeof(Node));
        if (h->head)
        {
            h->length = 0;
            int i = 0;
            for (i = 0; i < h->size; ++i)
            {
                h->head[i].next = NULL;
            }
        }
        else
            printf("Not enough space\n");
    }
    else
        printf("Not enough space!\n");
    return h;
}

Node* lookup(Hash_table* h, int objectID, int name)
{
    Node* p = h->head[name - 1].next;
    while (p && (p->data)->objectID != objectID)
        p = p->next;
    return p;
}

void Insert(Hash_table* h, IDlist k)
{
    Node* p = lookup(h, k.objectID, k.name);
    if (p == NULL)
    {
        Node* q = (Node*)malloc(sizeof(Node));
        q->data = (IDlist*)malloc(sizeof(IDlist));
        if (q->data != NULL)
        {
            (q->data)->name = k.name;
            (q->data)->objectID = k.objectID;
            for (int i = 0; i < 3; ++i)
            {
                (q->data)->id_gps_position[i] = k.id_gps_position[i];
            }
            for (int i1 = 0; i1 < 3; ++i1)
            {
                (q->data)->color[i1] = k.color[i1];
            }
            q->next = h->head[k.name - 1].next;
            h->head[k.name - 1].next = q;
            h->length += 1;
            return;
        }
        else
        {
            printf("No space for your operation!\n");
            return;
        }

    }
    else
    {
        // printf("The keys exist! \n");
        return;
    }

}

void destroy_table(Hash_table* h)
{
    int i;
    Node* p, * q;
    for (i = 0; i < h->size; ++i)
    {
        p = h->head[i].next;
        while (p)
        {
            q = p->next;
            free(p);
            p = q;
        }
        free(h->head);
        free(h);
    }
}

void print_Table(Hash_table* h)
{
    int i = 0;
    FILE* w;
    w = fopen("output.txt", "w");
    if (w == NULL)
    {
        printf("fail to open the file!\n");
        return;
    }
    for (i = 0; i < h->size; i++)
    {
        Node* p = h->head[i].next;
        while (p)
        {
            fprintf(w, "%d %d %f %f %f  \n", p->data->name, p->data->objectID,
                p->data->id_gps_position[0], p->data->id_gps_position[1], p->data->id_gps_position[2]);
            fprintf(w, "%f %f %f   \n", p->data->color[0], p->data->color[1], p->data->color[2]);
            p = p->next;
        }
        fprintf(w, "\n");
    }
    fclose(w);
}


/*--------------------------------------compass function-------------------------------------------------------------*/
double get_bearing_in_degrees(WbDeviceTag tag) {
    const double* north = wb_compass_get_values(tag);
    double rad = atan2(north[0], north[2]);
    double bearing = rad / M_PI * 180.0;
    // if (bearing < 0.0)
    //     bearing = bearing + 360.0;
    return bearing;
}


/*--------------------------------------init all the device----------------------------------------------------------*/
void device_init()
{
    int timestep = (int)wb_robot_get_basic_time_step();
    // Get and enable devices.
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, timestep);
    wb_camera_recognition_enable(camera, timestep);
    front_left_led = wb_robot_get_device("front left led");
    front_right_led = wb_robot_get_device("front right led");
    imu = wb_robot_get_device("inertial unit");
    wb_inertial_unit_enable(imu, timestep);
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, timestep);
    compass = wb_robot_get_device("compass");
    wb_compass_enable(compass, timestep);
    gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(gyro, timestep);
    emitter = wb_robot_get_device("emitter");
    wb_keyboard_enable(timestep);
    WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
    WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
    WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw");
    camera_motor[0] = camera_roll_motor;
    camera_motor[1] = camera_pitch_motor;
    camera_motor[2] = camera_yaw_motor;
}


void four_motor_init()
{
    WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
    WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
    WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
    WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
    motors[0] = front_left_motor;
    motors[1] = front_right_motor;
    motors[2] = rear_left_motor;
    motors[3] = rear_right_motor;
    int m;
    for (m = 0; m < 4; ++m) {
        wb_motor_set_position(motors[m], INFINITY);
        wb_motor_set_velocity(motors[m], 1.0);
    }
}


/*---------------------------------------------control the motor----------------------------------------------------*/
void four_motor_contorl(double roll, double pitch, double height_difference, double roll_acceleration,
    double roll_disturbance, double pitch_acceleration, double pitch_disturbance, double yaw_disturbance)
{
    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance;
    const double yaw_input = yaw_disturbance;
    const double clamped_difference_altitude = CLAMP(height_difference + k_vertical_offset, -0.8, 0.8);
    const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);
    // Actuate the motors taking into consideration all the computed inputs.
    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    wb_motor_set_velocity(motors[0], front_left_motor_input);
    wb_motor_set_velocity(motors[1], -front_right_motor_input);
    wb_motor_set_velocity(motors[2], -rear_left_motor_input);
    wb_motor_set_velocity(motors[3], rear_right_motor_input);
}


/*---------------------------------------------update the mavic fly phase--------------------------------------------*/
void update_stage(const double time, int h)
{
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);
    roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0] + M_PI / 2.0;
    pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    red_x_F = wb_gps_get_values(gps)[0];
    altitude_y = wb_gps_get_values(gps)[1];
    green_z = wb_gps_get_values(gps)[2];

    if (!h)
    {
        initial_x = red_x_F;
        initial_z = green_z;
    }
    roll_acceleration = wb_gyro_get_values(gyro)[0];
    pitch_acceleration = wb_gyro_get_values(gyro)[1];
    yaw_acceleration = wb_gyro_get_values(gyro)[2];
}


/*--------------------------------------------judge the object category----------------------------------------------*/
int namejudge(char* name) {
    int nameFlag = 0;
    char str_1[] = "beer bottle";
    char str_2[] = "jam jar";
    char str_3[] = "can";
    char str_4[] = "water bottle";
    char str_5[] = "honey jar";
    char str_6[] = "cereal box";
    char str_7[] = "biscuit box";
    if (!strcmp(name, str_1))
        nameFlag = BEER_BOTTLE;
    else if (!strcmp(name, str_2))
        nameFlag = JAM_JAR;
    else if (!strcmp(name, str_3))
        nameFlag = CAN;
    else if (!strcmp(name, str_4))
        nameFlag = WATER_BOTTLE;
    else if (!strcmp(name, str_5))
        nameFlag = HONEY_JAR;
    else if (!strcmp(name, str_6))
        nameFlag = CEREAL_BOX;
    else if (!strcmp(name, str_7))
        nameFlag = BISCUIT_BOX;
    return nameFlag;
}


/*--------------------------------------------------------------------------------------------------------------------*/
int main(int argc, char** argv)
{
    // Variables.
    wb_robot_init();
    int timestep = (int)wb_robot_get_basic_time_step();
    device_init();
    four_motor_init();

    double target_altitude = 2.00;  // The target altitude. Can be changed by the user.
    char* name = NULL;
    double roll_disturbance = 0.0;
    double pitch_disturbance = 0.0;
    double yaw_disturbance = 0.0;
    double difX = 0.0;
    double difZ = 0.0;
    double nodePose[3] = { 0.0,0.0,0.0 };

    int h = 0;
    long step = 0;
    long step_contract = 0;

    //some phase control bool constant
    bool left = false;
    bool right = false;
    bool go_contract = false;
    bool turn = false;
    bool contract_search = false;
    bool storage_tag = false;
    bool f1 = false;
    bool f2 = false;
    bool f3 = false;
    bool f4 = false;
    int if_finish_tag = 0;
    /*--------------------------------------------------------------------------------------------------------------*/



    /*----------------------------------------------scanf------------------------------------------------------------*/


    //define the objects array given to the youbot
    Trans2Youbot ob_list[4];
    //for (int d = 0; d < 4; ++d)
    //{
    //    ob_list[d] = (Trans2Youbot*)malloc(sizeof(Trans2Youbot));
    //}

    //create the hash table used
    Hash_table* idtable = Create_Table();

    // Main loop
    while (wb_robot_step(timestep) != -1)
    {

        const double time = wb_robot_get_time();  // in seconds.
        //update the fly phase
        update_stage(time, h);
        //call the camera
        int num = wb_camera_recognition_get_number_of_objects(camera);

        /*----------------------if see the object---------------------------------------------*/
        if (num)
        {
            const WbCameraRecognitionObject* object = wb_camera_recognition_get_objects(camera);
            for (int i = 0; i < num; ++i)
            {
                int objectID = object[i].id;
                name = object[i].model;
                int number_of_colors = object[i].number_of_colors;
                double* colors = object[i].colors;
                int nameflag = namejudge(name);
                WbNodeRef node = wb_supervisor_node_get_from_id(objectID);
                const double* nodePosition = wb_supervisor_node_get_position(node);
                //shelf_add function
                shelf_add(nodePosition);
                IDlist idlist;
                idlist.name = nameflag;
                idlist.objectID = objectID;
                for (int r = 0; r < 3; ++r)
                    idlist.id_gps_position[r] = nodePosition[r];
                for (int r1 = 0; r1 < 3; ++r1)
                {
                    idlist.color[r1] = colors[r1];
                }
                //insert into the hash table
                Insert(idtable, idlist);
                storage_tag = true;
            }
        }
        /*-------------------------------------------------------------------------------------*/
        if (if_finish_tag == 1 && !f1)
        {
            lookup4trans(idtable, &ob_list[0], 2, 0);
            //  printf("1: %f\n", ob_list[if_finish_tag - 1].i_pos[0]);
            //  printf("1: %f\n", ob_list[if_finish_tag - 1].i_pos[1]);
            //  printf("1: %f\n", ob_list[if_finish_tag - 1].o_pos[2]);
            //  printf("1: %f\n", ob_list[if_finish_tag - 1].o_pos[1]);
            f1 = true;
            //  Trans2Youbot we = { {0.2,0.2},{0.2,0.1,0.05},{-0.2,0.1,0.3} };
            wb_emitter_send(emitter, &ob_list[if_finish_tag - 1], sizeof(Trans2Youbot));
        }
        if (if_finish_tag == 2 && !f2)
        {
            lookup4trans(idtable, &ob_list[if_finish_tag - 1], 1, 1);
            f2 = true;
            wb_emitter_send(emitter, &ob_list[if_finish_tag - 1], sizeof(Trans2Youbot));
        }
        if (if_finish_tag == 3 && !f3)
        {
            lookup4trans(idtable, &ob_list[if_finish_tag - 1], 0, 0);
            f3 = true;
            wb_emitter_send(emitter, &ob_list[if_finish_tag - 1], sizeof(Trans2Youbot));
        }
        if (if_finish_tag == 4 && !f4)
        {
            lookup4trans(idtable, &ob_list[if_finish_tag - 1], 3, 1);
            f4 = true;
            wb_emitter_send(emitter, &ob_list[if_finish_tag - 1], sizeof(Trans2Youbot));
        }
        //set necessary parameters to help controlling
        //control the drone
        double gps_update_x = wb_gps_get_values(gps)[0];
        double gps_update_y = wb_gps_get_values(gps)[1];
        double gps_update_z = wb_gps_get_values(gps)[2];

        difX = initial_x - gps_update_x;
        difZ = initial_z - gps_update_z;
        altitude_y = gps_update_y;

        //use in the initial position
        if (!go_contract)
        {
            // printf("WHY 0?\n");
            pitch_disturbance = (difX) > (0.001) ? (-0.125) : (difX) < (-0.001) ? (0.125) : (0);
            roll_disturbance = (difZ) > (0.001) ? (0.0625) : (difZ) < (-0.001) ? (-0.0625) : (0);
        }

        //use in the diagonal process
        else
        {
            if ((fabs(difX) > 0.3) || (fabs(difZ) > 0.3) && (!turn))
            {
                //  printf("WHY 1?\n");
                pitch_disturbance = (difX) > (0.5) ? (-1.5) : (difX) < (-0.5) ? (1.5) : (0);
                roll_disturbance = (difZ) > (0.001) ? (0.0625) : (difZ) < (-0.001) ? (-0.0625) : (0);
            }
            if ((fabs(difX) < 0.3) && (fabs(difX) > 0.1) && (!turn))
            {
                // printf("WHY 2\n?");
                pitch_disturbance = (difX) > (0.001) ? (-0.125) : (difX) < (-0.001) ? (0.125) : (0);
                roll_disturbance = (difZ) > (0.001) ? (0.0625) : (difZ) < (-0.001) ? (-0.0625) : (0);
            }

            //turn 180 degree
            if ((fabs(difX) < 0.2 || fabs(difZ) < 0.2))
            {
                turn = true;
                const double* north = wb_compass_get_values(compass);
                // printf("North0: %f\n", north[0]);
                // printf("North2: %f\n", north[1]);
                double rad = atan2(north[0], north[1]);
                angle = rad / M_PI * 180.0;
                printf("The angle now is: %d\n", (int)angle);
                if ((int)angle < 128)
                {
                    yaw_disturbance = -0.1;
                    roll_disturbance = 0.0;
                    pitch_disturbance = 0.0;
                }
                else if ((int)angle > 132)
                {
                    yaw_disturbance = 0.1;
                    roll_disturbance = 0.0;
                    pitch_disturbance = 0.0;
                }
                else
                {
                    yaw_disturbance = 0.0;
                    roll_disturbance = 0.0;
                    pitch_disturbance = 0.0;
                    contract_search = true;
                }
                // step = 0;
                if (!contract_search)
                    wb_motor_set_position(camera_motor[1], 0.4);
            }
        }

        double height_difference = target_altitude - altitude_y;

        //start control the camera
        if (fabs(height_difference) < 0.70)
        {

            if (!go_contract)
            {
                int elapsed_time = 0;
                if ((!right) && (!left) && (step < 51))
                    wb_motor_set_position(camera_motor[2], -(step + 1.0) / 100.0);
                if ((!right) && (!left) && (step == 51))                  //finish the first process - right
                {
                    left = true;
                    step = 0;
                    if_finish_tag = 1;

                }
                if ((!right) && left && (step < 101))
                    wb_motor_set_position(camera_motor[2], (-51 + (step + 1.0)) / 100.0);
                if ((!right) && (step == 101))                           //finish the first process - left
                {
                    right = true;
                    step = 0;
                    if_finish_tag = 2;
                }
                if (right && left && (step < 51))
                    wb_motor_set_position(camera_motor[2], (51 - (step + 1.0)) / 100.0);
                if (right && left && (step == 51))
                {
                    initial_x = -initial_x;
                    initial_z = -initial_z;
                    go_contract = true;
                }
                step++;
            }
            else
            {
                if (contract_search)
                {
                    if (step_contract == 0)
                    {
                        left = false;
                        right = false;
                        wb_motor_set_position(camera_motor[1], 0.15);
                    }
                    int elapsed_time = 0;
                    if ((!right) && (!left) && (step_contract < 51))
                        wb_motor_set_position(camera_motor[2], -(step_contract + 1.0) / 100.0);
                    if ((!right) && (!left) && (step_contract == 51))          //finish the second process-left
                    {
                        left = true;
                        step_contract = 0;
                        if_finish_tag = 3;
                    }
                    if ((!right) && left && (step_contract < 101))
                        wb_motor_set_position(camera_motor[2], (-51 + (step_contract + 1.0)) / 100.0);
                    if ((!right) && (step_contract == 101))                   //finish the second process-right
                    {
                        right = true;
                        step_contract = 0;
                        if_finish_tag = 4;
                    }
                    if (right && left && (step_contract < 51))
                        wb_motor_set_position(camera_motor[2], (51 - (step_contract + 1.0)) / 100.0);
                    step_contract++;
                }
            }
            // step++;
        }

        //control the drone 
        four_motor_contorl(roll, pitch, height_difference, roll_acceleration, roll_disturbance,
            pitch_acceleration, pitch_disturbance, yaw_disturbance);
        h++;

        // update the gps position list
     /*   if (storage_tag)
        {
            for (int update = 0; update < idtable->size; ++update)
            {
               // Node* p = (Node*)malloc(sizeof(Node));
                Node* p = idtable->head[update].next;
                while (p)
                {
                    p->data = (IDlist*)malloc(sizeof(IDlist));
                    int old_object_id = p->data->objectID;
                    int old_name = p->data->name;
                    // printf("Old name: %d\n", old_name);
                    if (old_name >= 1 && old_name <= 7)
                    {
                        WbNodeRef old_node = wb_supervisor_node_get_from_id(old_object_id);
                        const double* old_nodePosition = wb_supervisor_node_get_position(old_node);
                        for (int position_tag = 0; position_tag < 3; ++position_tag)
                        {
                            p->data->id_gps_position[update] = old_nodePosition[update];
                        }
                    }
                    p = p->next;
                }
            }
        }*/
        print_Table(idtable);
        output_shelf(shelf);
    };
    wb_robot_cleanup();
    destroy_table(idtable);
    return EXIT_SUCCESS;
}