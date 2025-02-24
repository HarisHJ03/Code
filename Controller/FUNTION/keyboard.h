#ifndef _keyboard_H
#define _keyboard_H

#include "stm32f4xx.h"

/* control key definition */
//      direction  key
#define FORWARD    (rc.kb.bit.W)
#define BACK       (rc.kb.bit.S)
#define LEFT       (rc.kb.bit.A)
#define RIGHT      (rc.kb.bit.D)
//      speed      key
#define FAST_SPD   (rc.kb.bit.SHIFT)
#define SLOW_SPD   (rc.kb.bit.CTRL)


//      function   key or mouse operate

#define TRACK_CTRL                (km.r_mouse_sta == MOUSE_LONG)


//      shoot relevant       key or mouse operation
#define KB_SINGLE_SHOOT     (km.l_mouse_sta == MOUSE_ONCE)
#define KB_CONTINUE_SHOOT   (km.l_mouse_sta == MOUSE_LONG)

/*      pc con_dodge    */
#define Left_offset         (rc.kb.bit.C)
#define Right_offset        (rc.kb.bit.V)
#define SENTRY_MODE         (rc.kb.bit.Z)
////			shoot mode
//#define KB_SHOOTMAD_MODE		(rc.kb.bit.F)
//#define KB_SHOOTTHROW_MODE	(rc.kb.bit.F && rc.kb.bit.CTRL)
//#define KB_SHOOTNOR_MODE		(rc.kb.bit.F && rc.kb.bit.SHIFT) 
//#define KB_FRIC_COMPE_INC		(rc.kb.bit.C)
//#define KB_FRIC_COMPE_DEC		((rc.kb.bit.C && rc.kb.bit.SHIFT))
/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
//#define W 			0x0001		//bit 0
//#define S 			0x0002
//#define A 			0x0004
//#define D 			0x0008
//#define SHIFT 	0x0010
//#define CTRL 		0x0020
//#define Q 			0x0040
//#define E				0x0080
//#define R 			0x0100
//#define F 			0x0200
//#define G 			0x0400
//#define Z 			0x0800
//#define X 			0x1000
//#define C 			0x2000
//#define V 			0x4000		//bit 15
//#define B				0x8000
/******************************************************/

typedef enum 
{
  MOUSE_RELEASE,
  MOUSE_PRESS,
  MOUSE_DONE,
  MOUSE_ONCE,
  MOUSE_LONG,
  
}MOUSE_STATUS;

typedef struct
{
  MOUSE_STATUS l_mouse_sta;
  MOUSE_STATUS r_mouse_sta;
  uint16_t l_cnt;
  uint16_t r_cnt;
  
  uint8_t kb_enable;
  
  float vx_limit_speed;
  float vy_limit_speed;
  float vw_limit_speed;
  
  /*底盘方向*/
  float vx;
  float vy;
  float vw;
  /*云台方向*/
  float pit_v;
  float yaw_v; 
}kb_ctrl_t;

extern kb_ctrl_t km;

void keyboard_global_hook(void);
void keyboard_chassis_hook(void);
void keyboard_shoot_hook(void);
void keyboard_gimbal_hook(void);



void Software_Reset(void);



#endif
