#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "stm32f4xx.h"
#include "bsp_imu.h"
// #include "gimbal_task.h"
// #include "cmsis_os.h"
#include "bsp_imu.h"
#include "pid.h"
#include "sys_config.h"
// #include "bsp_io.h"
#include "math.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
// #include "bsp_can.h"
// #include "comm_task.h"
#include "bmi088driver.h"
#include "delay.h"
#include "MahonyAHRS.h"
// #include "IST8310.h"
#include "filters.h"
#include "bsp_dwt.h"
#include "dma.h"
// #include "detect_task.h"
#include "kalman_filter.h"
#include "BMI088Middleware.h"
#include "ahrs.h"
// #include "modeswitch_task.h"
#include "filter.h"
#include "iwdg.h"
#include "pc_task.h"

#include "stdio.h"


/* imu task period time (ms) */
#define IMU_TASK_PERIOD 1
#define SHOOT_TASK_PERIOD 3000

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

typedef struct
{
  int roll_cnt;
  int pitch_cnt;
  int yaw_cnt;
  
  float last_roll;
  float last_pitch;
  float last_yaw;

  float roll;
  float pitch;
  float yaw;
} imu_attitude_t;

void imu_task(void const *argu);


#endif



