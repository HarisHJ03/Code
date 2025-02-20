#ifndef _modeswitch_task_H
#define _modeswitch_task_H


#include "stm32f4xx.h"
#include "string.h"

#define MEMSET(flag,type) (memset((type*)flag,0,sizeof(type)))

/*主要控制模式*/
typedef enum
{
  RELEASE_CTRL,                   
  MANUAL_CTRL,                    //手动
  SEMI_AUTOMATIC_CTRL,            //半自动
  FULL_AUTOMATIC_CTRL,            //全自动
}global_status;
/*云台控制模式*/
typedef enum
{
  GIMBAL_RELEASE,     //云台释放
  GIMBAL_INIT,
  GIMBAL_NORMAL_MODE,
  GIMBAL_DODGE_MODE,  //小陀螺模式
  GIMBAL_SHOOT_BUFF,
  GIMBAL_TRACK_ARMOR,
	GIMBAL_NAVIGATION_MODE,
}gimbal_status;
/*底盘控制模式*/
typedef enum
{
  CHASSIS_RELEASE,
  CHASSIS_NORMAL_MODE,
  CHASSIS_DODGE_MODE,
  CHASSIS_STOP_MODE,
	CHASSIS_NAVIGATION_MODE,
  
}chassis_status;

typedef enum
{
  SHOOT_DISABLE,
  SHOOT_ENABLE,
  
}shoot_status;

typedef enum
{
	RC_NUN,
	RC_W,
	RC_A,
	RC_D,
	RC_S,
}rc_status;

extern global_status global_mode; 
extern global_status last_global_mode;

extern gimbal_status gimbal_mode;
extern gimbal_status last_gimbal_mode;

extern chassis_status chassis_mode;
extern chassis_status last_chassis_mode;

extern shoot_status shoot_mode;
extern shoot_status last_shoot_mode;

void mode_switch_task(void *parm);
void get_last_mode(void);
void get_main_mode(void);
void get_gimbal_mode(void);
void get_chassis_mode(void);
void get_shoot_mode(void);






#endif

