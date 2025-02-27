#ifndef __CONTROLLER_TASK_H
#define __CONTROLLER_TASK_H

#include "stm32f4xx.h"
#include "iwdg.h"
#include "imu_task.h"

extern void controller_task(void const *argu);
extern float clc_dt;
extern float clc_total_distance[3];

typedef struct
{
    float clc_distance[3];
    float clc_total_distance[3];

    float last_speed[3];
    float now_speed[3];
} Controller;

#endif
