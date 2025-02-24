#ifndef _remote_ctrl_H
#define _remote_ctrl_H

#include "stm32f4xx.h"
#include "rc.h"

/*云台底盘控制*/
#define RC_DODGE_MODE             ((rc.sw2 == RC_MI) && (rc.sw1 == RC_MI) && (rc.iw >= IW_DN))
//#define RC_SEPARATE_MODE          ((rc.sw2 == RC_MI) && (rc.iw >= IW_DN))
#define RC_TRACK_ARMOR_MODE       ((rc.sw2 == RC_UP) && (rc.sw1 == RC_UP)) //((rc.sw2 == RC_UP) && (rc.sw1 == RC_UP)) &&   //半自动 自瞄模式
//#define RC_TRACK_ARMOR_MODE       rc.sw2 == RC_UP
//#define RC_TRACK_ARMOR_MODE       rc.sw2 == RC_MI
/*射击控制*/
#define RC_SINGLE_SHOOT    				((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN))
#define RC_CONTINUE_SHOOT  				(rc.sw1 == RC_DN)
#define RC_CTRL_FRIC_WHEEL 				((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP))
#define RC_CTRL_BALL_STOR	 				((rc.sw2 == RC_MI) && (rc.sw1 ==RC_MI) && (rc.iw >= IW_DN) )
#define RC_CTRL_BALL_STOR_CLOSE	 	((rc.sw2 == RC_MI) && (rc.sw1 ==RC_MI) && (rc.iw <= IW_UP) )
#define RC_VISION_SINGLE_SHOOT          ((rc.sw1 == RC_UP) && (rc.sw2 == RC_UP)&&(rc.iw >= IW_DN))
#define RC_VISION_CONTINUE_SHOOT        ((rc.sw1 == RC_UP) && (rc.sw2 == RC_UP)&&(rc.iw <= IW_UP))

enum
{
  RC_UP = 1,
  RC_MI = 3,
  RC_DN = 2, 
	IW_UP = 694,		//	min 364		middle 1024	拨轮
	IW_DN = 1354,		//	max 1684
	IW_MI = 1024,
};

typedef struct
{
	/*记录拨杆上一次状态*/
  uint8_t last_sw1;
  uint8_t last_sw2;
	uint16_t last_iw;
} sw_record_t;

typedef struct
{
	/*底盘方向*/
  float vx;
  float vy;
  float vw;
  /*云台方向*/
  float pit_v;
  float yaw_v;
	/*导航时的底盘方向*/
	float chassis_yaw_v;
} rc_ctrl_t;

extern rc_ctrl_t rm;
extern sw_record_t glb_sw;

void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf);
void remote_ctrl_chassis_hook(void);
void remote_ctrl_gimbal_hook(void);
void remote_ctrl_shoot_hook(void);




#endif


