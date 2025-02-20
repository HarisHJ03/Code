#include "keyboard.h"
#include "remote_ctrl.h"
#include "STM32_TIM_BASE.h"
#include "sys_config.h"
#include "ramp.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "modeswitch_task.h"
#include "judge_rx_data.h"

extern int keyboard_flag;
extern int KB_FRIC;
extern int KB_BALL;
extern int FRIC_SPD_UP;
extern int FRIC_SPD_DOWN;
extern int speed_15;
extern int speed_18;
extern int speed_30;
extern int Fric_Spd_Ajt;
extern int SOFT_RESET;
extern int SMALL_BUFF;
extern int BIG_BUFF;
extern int PC_DODGE;
extern int normal_speed;
extern int direction;
extern int direction_change;

extern chassis_t chassis;
kb_ctrl_t km;
ramp_t key_fbramp;
ramp_t key_rlramp;

void get_mouse_status(MOUSE_STATUS *status,uint8_t mouse)
{
  switch(*status)
  {
    case MOUSE_RELEASE:
    {
      if(mouse)
        *status = MOUSE_PRESS;
      else
        *status = MOUSE_RELEASE;
    }break;
    
    case MOUSE_PRESS:
    {
      if(mouse)
        *status = MOUSE_DONE;
      else
        *status = MOUSE_RELEASE;
    }break;
    
    case MOUSE_DONE:
    {
      if(mouse)
      {
        *status = MOUSE_ONCE;
        if(status == &km.l_mouse_sta)
          km.l_cnt = HAL_GetTick();
        else
          km.r_cnt = HAL_GetTick();
      }
      else
        *status = MOUSE_RELEASE;
    }break;
    
    case MOUSE_ONCE:
    {      
      if(mouse)
      {
        if(status == &km.l_mouse_sta)
        {
          if(HAL_GetTick() - km.l_cnt > 200)//100//修改此处可以改变鼠标单击太快会连发的BUG
            *status = MOUSE_LONG;
        }
        else
        {
          if(HAL_GetTick() - km.r_cnt > 100)
            *status = MOUSE_LONG;
        }
      }
      else
        *status = MOUSE_RELEASE;
    }break;
    
    case MOUSE_LONG:
    {
      if(!mouse)
      {
        *status = MOUSE_RELEASE;
      }
    }break;
    
    default:
    {
    }break;
  }
}



void keyboard_global_hook(void)
{
  if (km.kb_enable)
  {
    get_mouse_status(&km.l_mouse_sta, rc.mouse.l);
    get_mouse_status(&km.r_mouse_sta, rc.mouse.r);
		
		static int i=1000;
		
		if(i>=1000&&direction_change==0){
		if(rc.mouse.z<0){
		i=0;
		direction_change=1;
		direction=-direction;
		}
	}else{
		i++;
	}
  }
}

/*控制方向*/
static void chassis_direction_ctrl(uint8_t forward, uint8_t back,
                                uint8_t left,    uint8_t right)
{
  if(back)
  {
    km.vx = 1;
		rm.vx=0;
		rm.vy=0;
		rm.vw=0;
  }
  else if(forward)
  {
    km.vx = -1;
		rm.vx=0;
		rm.vy=0;
		rm.vw=0;
  }
  else
  {
   km.vx = 0;
	 rm.vx=0;
	 rm.vy=0;
	 rm.vw=0;
  }
  
  if(right)
  {
   km.vy =  1 ;
	 rm.vx=0;
	 rm.vy=0;
	 rm.vw=0;
  }
  else if(left)
  {
   km.vy =   -1;
	 rm.vx=0;
	 rm.vy=0;
	 rm.vw=0;
  }
  else
  {
   km.vy = 0;
	 rm.vx=0;
	 rm.vy=0;
	 rm.vw=0;
  }
}

/*控制摩擦轮*/
static void firc_ctrl(uint8_t firc_open)
{
  if(firc_open==0)
		{
    shoot.fric_wheel_run = 0;
			shoot2.fric_wheel_run=0;
		}
	  
  if(firc_open==1)
	{
    shoot.fric_wheel_run = 1;
		 shoot2.fric_wheel_run = 1;
	}
}

/*控制单发和连发命令*/
static void shoot_cmd_ctrl(uint8_t shoot_cmd,uint8_t c_shoot_cmd)
{
  if(shoot_cmd)
  {
    shoot.shoot_cmd = 1;
    shoot.c_shoot_cmd = 0;
		
		   shoot2.shoot_cmd = 1;        //---------------新增3/16
    shoot2.c_shoot_cmd = 0;
  }
		
  if(c_shoot_cmd && (trig.one_sta == TRIG_INIT))
  {
    shoot.shoot_cmd = 0;
    shoot.c_shoot_cmd = 1;
  }
	
	  if(c_shoot_cmd && (trig2.one_sta == TRIG_INIT))     //---------新增3/16
  {
    shoot2.shoot_cmd = 0;
    shoot2.c_shoot_cmd = 1;
  }
	else
	{
		//shoot.shoot_cmd = 0;
//    shoot.c_shoot_cmd = 0;//不注释掉无法通过遥控连发
	}
	
}

/*控制弹舱盖*/
static void shoot_storage_ctrl(uint8_t storage_open)
{
	if(storage_open==0)
    shoot.ball_storage_open = 0;
  if(storage_open==1)
    shoot.ball_storage_open = 1;

}

/*控制射击速度*/
static void kb_shoot_spd_ctrl(uint16_t Shoot_Spd_UP,uint16_t Shoot_Spd_DOWN)
{
	if(Shoot_Spd_UP==1)
	{
		Fric_Spd_Ajt=2;
    }
	if(Shoot_Spd_DOWN==1)
	{
		Fric_Spd_Ajt=1;
    }
}

/*控制pit,yaw速度*/
float yaw_speed=0.01;
static void gimbal_speed_ctrl(int16_t pit_ref_spd, int16_t yaw_ref_spd)
{
	//鼠标往左yaw_ref_spd为正
//	km.pit_v =  pit_ref_spd * 0.015;
////////////////////////////////////

			km.pit_v =  -pit_ref_spd * 0.015;
		
////////////////////////////////////
	km.yaw_v = -yaw_ref_spd * yaw_speed;

}

/*实现软件复位*/
static void sofe_reset(uint16_t Soft_Reset)
{
	if(Soft_Reset==1)
	Software_Reset();
}
void keyboard_chassis_hook(void)
{
  if(km.kb_enable)
  {
		if(direction==1)
    chassis_direction_ctrl(FORWARD, BACK, LEFT, RIGHT);
		else
		chassis_direction_ctrl(BACK, FORWARD,RIGHT ,LEFT);
  }
  else
  {
    km.vx = 0;
    km.vy = 0;
  }
}

void keyboard_shoot_hook(void)
{

	
  if(km.kb_enable)
  {
		
		if(keyboard_flag==1){//松手标志位
    firc_ctrl(KB_FRIC);
    shoot_storage_ctrl(KB_BALL);
	kb_shoot_spd_ctrl(FRIC_SPD_UP,FRIC_SPD_DOWN);
	/////////////////////////////////////////////		
			FRIC_SPD_UP=0;//将摩擦轮加速标志位置0，防止一直加
			FRIC_SPD_DOWN=0;
			//Fric_Spd_Ajt=3;
	/////////////////////////////////////////////
	sofe_reset(SOFT_RESET);
		}
		
	}

		shoot_cmd_ctrl(KB_SINGLE_SHOOT, KB_CONTINUE_SHOOT);   //-------------这里我的想法是两个枪管一起控制

}
void keyboard_gimbal_hook(void)
{
  if (km.kb_enable)
  {
    gimbal_speed_ctrl(rc.mouse.y, rc.mouse.x);
  }
  else
  {
    km.pit_v = 0;
    km.yaw_v = 0;
    gimbal.small_buff_ctrl = 0;
    gimbal.big_buff_ctrl = 0;
    gimbal.track_ctrl = 0;
  }
}
void Software_Reset(void)
{
    NVIC_SystemReset();
}


