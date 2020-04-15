#include "process_control.h"
#include "mavic_init.h"
#include "motor_contorl.h"

double get_bearing_in_degrees(WbDeviceTag tag) {
    const double* north = wb_compass_get_values(tag);
    double rad = atan2(north[0], north[2]);
    double bearing = rad / M_PI * 180.0;
    return bearing;
}

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
    if (!p)
    {
        Node* q = (Node*)malloc(sizeof(Node));
        if (q)
        {
            q->data = (IDlist*)malloc(sizeof(IDlist));
            if (q->data)
            {
                (q->data)->name = k.name;
                (q->data)->objectID = k.objectID;
                for (int i = 0; i < 3; ++i)
                {
                    (q->data)->id_gps_position[i] = k.id_gps_position[i];
                }
                q->next = h->head[k.name - 1].next;
                h->head[k.name - 1].next = q;
                h->length += 1;
                return;
            }
            else
                printf("Not enough space\n");
        }
        else
            printf("Not enough space\n");
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
            fprintf(w, "[%d-%d-%f-%f-%f] ", p->data->name, p->data->objectID,
                p->data->id_gps_position[0], p->data->id_gps_position[1], p->data->id_gps_position[2]);
            p = p->next;
        }
        fprintf(w, "NULL\n");
    }
    fclose(w);
}