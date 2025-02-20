#include "remote_ctrl.h"
#include "stdlib.h"
#include "string.h"
#include "sys_config.h"
#include "STM32_TIM_BASE.h"
#include "shoot_task.h"
#include "keyboard.h"
#include "delay.h"
#include "pc_rx_data.h"
#include "chassis_task.h"
#include "modeswitch_task.h"

rc_ctrl_t rm;
sw_record_t glb_sw;

extern int KB_FRIC;
extern int KB_BALL;
extern int direction;
int16_t mouse_x_now=0;
int16_t mouse_x_last=0; 
int16_t deviation=0;
int num=0;
void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf)
{
	rc->ch1 = (dbus_buf[0] | dbus_buf[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (dbus_buf[1] >> 3 | dbus_buf[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (dbus_buf[2] >> 6 | dbus_buf[3] << 2 | dbus_buf[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (dbus_buf[4] >> 1 | dbus_buf[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;
  
  /* prevent remote control zero deviation */
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;
	//由于控不住加速前进，一直会飘所以只能除此下策
  if(rc->ch2>=660)
		rc->ch2 =659 ;
  rc->sw1 = ((dbus_buf[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (dbus_buf[5] >> 4) & 0x0003;
  rc->iw = (dbus_buf[16] | dbus_buf[17] << 8) & 0x07FF;
	
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_info_t));
    return ;
  }
	
	
	
	rc->mouse.l = dbus_buf[12];
  rc->mouse.r = dbus_buf[13];
  rc->mouse.z = dbus_buf[10] | (dbus_buf[11] << 8);
	
	mouse_x_now= dbus_buf[6] | (dbus_buf[7] << 8); 
	
	if(mouse_x_now<500&&mouse_x_now>-500){
	
	deviation=mouse_x_now- mouse_x_last;
	
	if(mouse_x_now!=0||num==5){
	rc->mouse.x = dbus_buf[6] | (dbus_buf[7] << 8); // x axis
  rc->mouse.y = dbus_buf[8] | (dbus_buf[9] << 8);

	num=0;
	}else{
	if(deviation<10&&deviation>-10)num+=1;
	}
	mouse_x_last= dbus_buf[6] | (dbus_buf[7] << 8);
}

  rc->kb.key_code = dbus_buf[14] | dbus_buf[15] << 8; // key borad code
}



/*
* @ RC_RESOLUTION :摇杆最大值 660 
* @ CHASSIS_RC_MAX_SPEED_X
    CHASSIS_RC_MAX_SPEED_Y
    CHASSIS_RC_MAX_SPEED_R ：平移和旋转的速度最大值
  @ CHASSIS_RC_MOVE_RATIO_X
    CHASSIS_RC_MOVE_RATIO_Y
    CHASSIS_RC_MOVE_RATIO_R : 数值方向
*/

static void chassis_operation_func(int16_t forward_back, int16_t left_right, int16_t rotate)
{
	if(direction==1){//换头前后的操作不同
  rm.vx =  forward_back / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X ;
  rm.vy = -left_right / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y ;
	}
	else{
  rm.vx = -forward_back / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X ;
  rm.vy =  left_right / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y ;
	}
  rm.vw = rotate / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_R ;

//	if(pc_recv_mesg.navigation_determine == 1 && 	chassis_mode == GIMBAL_NAVIGATION_MODE){
//	rm.vx += 1000 * -pc_recv_mesg.linear_speed_x;
//	rm.vy += 1000 * -pc_recv_mesg.linear_speed_y;
//	//rm.vw += pc_recv_mesg.angular_speed_w * 57.3;
//	}
}

 

void remote_ctrl_chassis_hook(void)
{
  chassis_operation_func(-rc.ch2, -rc.ch1, rc.ch3);
}

static void gimbal_operation_func(int16_t pit_ctrl, int16_t yaw_ctrl)             //读取遥控值
{
  /* gimbal coordinate system is right hand coordinate system */
  rm.pit_v = -pit_ctrl * 0.0005f * GIMBAL_RC_MOVE_RATIO_PIT;
  rm.yaw_v = -yaw_ctrl * 0.0005f * GIMBAL_RC_MOVE_RATIO_YAW;
	
	//底盘的转动方向
	rm.chassis_yaw_v = -yaw_ctrl * 0.0005f * GIMBAL_RC_MOVE_RATIO_YAW;
//	if(pc_recv_mesg.mode_Union.info.navigation_determine == 1){
//	//rm.yaw_v += -rm.vw / CHASSIS_RC_MAX_SPEED_R * RC_RESOLUTION *  GIMBAL_RC_MOVE_RATIO_YAW * 0.0005f;
//	}
}

void remote_ctrl_gimbal_hook(void)
{
			gimbal_operation_func(rc.ch4, rc.ch3);
/////////////////////////////
}


static void rc_fric_ctrl(uint8_t ctrl_fric)
{
  if (ctrl_fric)
  {
    shoot.fric_wheel_run = !shoot.fric_wheel_run;//切换摩擦轮的状态
		shoot2.fric_wheel_run = !shoot2.fric_wheel_run;//切换摩擦轮的状态     //------------3/16新增
		KB_FRIC=2;
  }
}

//打开弹舱盖
static void rc_ball_storage_ctrl(uint8_t open_storage, uint8_t close_storage)
{
  if(open_storage)
  {
    shoot.ball_storage_open = 1;
		KB_BALL=2;
  }
  if(close_storage)
  {
    shoot.ball_storage_open = 0;
		KB_BALL=2;
  }

}

static void rc_shoot_cmd(uint8_t single_fir, uint8_t cont_fir,uint8_t vision_single_fir,uint8_t vision_cont_fir)
{
    if(vision_single_fir)
    {
    shoot.shoot_cmd   = 1;
    shoot.c_shoot_cmd = 0;
			
			 shoot2.shoot_cmd   = 1;  //-----------3/16新增
    shoot2.c_shoot_cmd = 0;
		km.kb_enable = 1;
    }

  if (single_fir)
  {
    shoot.c_shoot_time = HAL_GetTick();
	
    shoot.shoot_cmd   = 1;
    shoot.c_shoot_cmd = 0;
		
		shoot2.c_shoot_time = HAL_GetTick();//-----------3/16新增
		 shoot2.shoot_cmd   = 1;  
    shoot2.c_shoot_cmd = 0;
		km.kb_enable = 1;
  }
  if ((cont_fir||vision_cont_fir) && (HAL_GetTick() - shoot.c_shoot_time >= 500) && (trig.one_sta == TRIG_INIT))	//单发和连发相隔150毫秒
  {
    shoot.shoot_cmd   = 0;
    shoot.c_shoot_cmd = 1;
		km.kb_enable = 1;
  }
	else shoot.c_shoot_cmd = 0;
	
	 if ((cont_fir||vision_cont_fir) && (HAL_GetTick() - shoot2.c_shoot_time >= 500) && (trig2.one_sta == TRIG_INIT))	//单发和连发相隔150毫秒     //---------3/16新增
  {
    shoot2.shoot_cmd   = 0;
    shoot2.c_shoot_cmd = 1;
		km.kb_enable = 1;
  }
 else shoot2.c_shoot_cmd = 0;
     
 
}

static void kb_enable_hook(void)
{
  if (rc.sw2 == RC_MI)
	  km.kb_enable = 1;
  else
    km.kb_enable = 0;
//	if(shoot.shoot_cmd == 0&&shoot.c_shoot_cmd == 0)
//	{
//	  if (rc.sw2 != RC_DN)
//    km.kb_enable = 1;//0;
//		else
//    km.kb_enable = 0;//1;
//	}
}

void remote_ctrl_shoot_hook(void)
{
	//开关弹舱盖

	rc_ball_storage_ctrl(RC_CTRL_BALL_STOR, RC_CTRL_BALL_STOR_CLOSE);
	
  //开摩擦轮
  rc_fric_ctrl(RC_CTRL_FRIC_WHEEL);
 
	//单发和连发使能
  rc_shoot_cmd(RC_SINGLE_SHOOT, RC_CONTINUE_SHOOT,RC_VISION_SINGLE_SHOOT,RC_VISION_CONTINUE_SHOOT);
  
  //使能键盘鼠标
  kb_enable_hook();
}

