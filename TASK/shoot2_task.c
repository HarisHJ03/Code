#include "shoot_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "comm_task.h"
#include "modeswitch_task.h"
#include "detect_task.h"
#include "string.h"
#include "sys_config.h"
#include "math.h"
#include "pid.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "pc_rx_data.h"
#include "rc.h"
#include "gimbal_task.h"
#include "math.h"
#include "remote_ctrl.h"

UBaseType_t shoot2_stack_surplus;

extern TaskHandle_t can_msg_send_Task_Handle;
extern gimbal_t gimbal;

shoot_t   shoot2;
trigger_t trig2;
uint16_t normal2_speed;
uint16_t Fric2_Spd_Ajt;//3为初始未使用状态

/*----2024哨兵----   
	枪口速度上限 30m/s
	枪口热量上限 400
	枪口热量每秒冷却值 80
	每发一颗子弹热量+10	
*/

#if(SENTRY_NUM == SENTRY_1)
/*摩擦轮pid*/
float fric2_pid[3] = {24, 0, 0};
/* 摩擦轮转速 */
uint16_t speed2_30 = 7000;//8200

	/*弹仓盖开关*/
float ccr2_open  = 1550;
float ccr2_close = 430;

#endif

	int close2_down  = 1;     //弹仓盖关闭完成标志位
	int open2_down   = 1;      //弹仓盖打开完成标志位
	
	/* 拨盘转速 */
	int normal2_cshoot				= 1000 ;
	int lspd2 =6000;
	int rspd2=6000;	
	
	/*顺序：普通-基地-吊射-buff*/
	/*热量限制拨盘转速*/
  float heat_limit_pid2[3] = {30, 0, 10};
	/*热量限制连发时间间隔*/
	float heat_time_pid2[3]  = {9, 0, 0};//{3, 0 ,3};
	
	
	//动态射速pid
	float speedlimit2[3]={10,0,1};
	float last_shoot2_speed=0;//记录上一颗子弹的射速，作为PID输入值
	
int settime2=5;//剩余热量的阈值
float	heat_time_p2=1,radio_freq2=40;
float single_shoot2_angle;
uint16_t single_time2 = 100; //单发延时时间间隔初始值
uint16_t continue_time2 = 420;//连发延时时间间隔初始值
float surplus_heat2;   //剩余热量
float extra_time2; 
float shoot2_delay;//发射延迟
uint8_t switch_freq2 = 1;//射频转换标志


	
void shoot2_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
										        (uint32_t) INFO_GET_SHOOT2_SIGNAL, 
									        	(uint32_t *)&Signal, 
									        	(TickType_t) portMAX_DELAY );
    if(STAUS == pdTRUE)
		{
			if(Signal & INFO_GET_SHOOT2_SIGNAL)
			{
        if(shoot_mode != SHOOT_DISABLE)  //------------可能要改3/13
        {
                    
					continue_time2 = 1000/(judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10);//根据枪口冷却速率算出连发时间间隔	
					surplus_heat2 = judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit - judge_recv_mesg.power_heat_data.shooter_17mm_2_barrel_heat;//剩余热量
					radio_freq2 = (10000/judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit)-7;//根据热量上限算出允许的最小连发时间间隔，也就是最大射频
					
          /*热量控制*/
          PID_Struct_Init(&pid_heat_limit2, heat_limit_pid2[0], heat_limit_pid2[1], heat_limit_pid2[2], 7000, 0, DONE );
					/*连发间隔时间控制*/
          PID_Struct_Init(&pid_heat_time2, (1000/(judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10)-40)*0.01-heat_time_p2, heat_time_pid2[1], heat_time_pid2[2], 
					1000/(judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10)-radio_freq2, 0, DONE );//原数据heat_time_p1,radio_freq40
					pid_calc(&pid_heat_time,surplus_heat2,settime2);//0

					if((float)judge_data_limit.shooter_id2_17mm_speed_limit-1-judge_recv_mesg.shoot_data.initial_speed>5){
						speedlimit2[0]=300;
						speedlimit2[1]=0;
						speedlimit2[2]=1;
					}else{
					  speedlimit2[0]=10;
						speedlimit2[1]=0;
						speedlimit2[2]=1;
					}
				
					PID_Struct_Init(&pid_speedlimit2, speedlimit2[0], speedlimit2[1], speedlimit2[2], 7000, 0, DONE );
					
          /*摩擦轮*/
          for(int i=0;i<2;i++)
          {
            PID_Struct_Init(&pid_fric2[i], fric2_pid[0], fric2_pid[1], fric2_pid[2], 8000, 500, DONE ); 
          }


					
					
          shoot2_para_ctrl();						// 射击模式切换
          ball2_storage_ctrl();					// 舵机控制弹仓盖
          fric2_wheel_ctrl();						// 启动摩擦轮
          
          if (shoot2.fric_wheel_run)//若摩擦轮开启
          {
            shoot2_bullet_handler();      
          }
          else
          {
            shoot2.shoot_cmd   = 0;//单发标志位
            shoot2.c_shoot_cmd = 0;//连发标志位
            shoot2.fric_wheel_spd = 1000;//摩擦轮转速
            pid_trigger2_spd.out = 0;//拨盘                                   // 这里可能要另外加一个变量处理拨盘 未处理------------------------2024/2/26
            trig2.angle_ref = moto_trigger2.total_angle; //记录当前拨盘电机编码位
						trig2.one_sta = TRIG_INIT;//保证下次开启摩擦轮时，单发状态为初始化
          }
          
        }
        else
        {
          pid_trigger2_spd.out = 0;
          shoot2.fric_wheel_spd = 0;
        }
        xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                          (uint32_t) SHOT_MOTOR_MSG_SIGNAL, 
                          (eNotifyAction) eSetBits, 
                          (uint32_t *)NULL );
      }
    }
			
    shoot2_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
		
} 



void get_last_shoot2_mode(void)
{
	shoot2.last_para_mode = shoot2.para_mode;
}


/*射击模式选择
* @ SHOOTBUFF_MODE 神符模式
*	@ SHOOTTHROW_MODE 高射速吊射模式
*	@ SHOOTMAD_MODE 低射速近战模式
* @ SHOOTNOR_MODE 普通射速模式
**/

static void shoot2_para_ctrl(void)
{
	
	 if(judge_data_limit.shooter_id2_17mm_speed_limit == 30)
        normal2_speed = speed2_30;
	else
	normal2_speed = speed2_30;


if(shoot2.para_mode == SHOOTBUFF_MODE) 
		shoot2.fric_wheel_spd = speed2_30;
  else	
		shoot2.fric_wheel_spd = normal2_speed;
	
	/* 拨盘转速 */
	trig2.shoot_spd			 = 3000;
	trig2.c_shoot_spd		 = normal2_cshoot;
}

/*摩擦轮控制*/
static void fric2_wheel_ctrl(void)
{
	if (shoot2.fric_wheel_run)
	{
		turn2_on_friction_wheel(shoot2.fric_wheel_spd, shoot2.fric_wheel_spd);
	}
	else
	{
		turn2_off_friction_wheel();
	}
}


/*打开摩擦轮*/
static void turn2_on_friction_wheel(int16_t lspd,int16_t rspd)
{
		pid_calc(&pid_fric2[0], moto_fric[2].speed_rpm,-7000);
		pid_calc(&pid_fric2[1], moto_fric[3].speed_rpm, 7000);
		glb_cur.fric2_cur[0] = pid_fric2[0].out;                                       
		glb_cur.fric2_cur[1] = pid_fric2[1].out;
}

/*关闭摩擦轮*/
static void turn2_off_friction_wheel(void)
{
	pid_calc(&pid_fric2[0], moto_fric[2].speed_rpm, 0);
	pid_calc(&pid_fric2[1], moto_fric[3].speed_rpm, 0);
	glb_cur.fric2_cur[0] = pid_fric2[0].out;
	glb_cur.fric2_cur[1] = pid_fric2[1].out;
}


/*弹仓盖控制*/
static void ball2_storage_ctrl(void)
{
  if (shoot2.ball_storage_open)
  {
    TIM_SetCompare3(TIM8,ccr2_open);
  }
  else
  {
    TIM_SetCompare3(TIM8,ccr2_close);
  }
}

int Angle2=45;
static void shoot2_bullet_handler(void)
{
  if (shoot2.shoot_cmd)//单发
  {
		if(global_err.list[JUDGE_SYS_OFFLINE].err_exist == 1)//没连接裁判系统时
			single_time2 = 100;
		else
			single_time2 = 1000/(judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10 + judge_recv_mesg.game_robot_state.power_management_shooter_output);
  
        shoot2_delay = single_time2;
		if (trig2.one_sta == TRIG_INIT)
		{
			trig2.one_time = HAL_GetTick();
	//发射（single_shoot_angle设为当前角度加上Angle）,就是这个让拨盘拨动
			single_shoot2_angle = moto_trigger2.total_angle + Angle2;
			trig2.one_sta = TRIG_PRESS_DOWN;//发射按键已按下
    }
		/* 发射延时 */
    else if (trig2.one_sta == TRIG_PRESS_DOWN)//若发射按键已按下便进行延时
    {			
        if(RC_VISION_SINGLE_SHOOT)
        {
        shoot2_delay = 1000;
        }
      if (HAL_GetTick() - trig2.one_time >= shoot2_delay)
      {
        trig2.one_sta = TRIG_ONE_DONE;//延时完毕后，状态才为发射已完成
      }
    }
     
    if (trig2.one_sta == TRIG_ONE_DONE)//若发射状态为已完成
    {
		single_shoot2_angle = moto_trigger2.total_angle;                         
        shoot2.shoot_cmd = 0;//单发标志位置零
		trig2.one_sta = TRIG_INIT;//单发状态设为初始化
		trig2.c_sta = TRIG_INIT;//连发状态设为初始化
      shoot2.shoot_bullets++;//发射子弹计数
    }
		trig2.angle_ref = single_shoot2_angle;//拨盘目标角度设为single_shoot_angle
		
  }
	//枪口速度上限 30m/s
	//枪口热量上限 400
	//枪口热量每秒冷却值 80
	//每发一颗子弹热量+10
	
	else if (shoot2.c_shoot_cmd)//连发做多次单发
  {
        //计算延时时间
		if(global_err.list[JUDGE_SYS_OFFLINE].err_exist == 1)//没连接裁判系统时
		{
			if (trig2.c_sta == TRIG_INIT)
			{
				trig2.one_time = HAL_GetTick();
				single_shoot2_angle = moto_trigger2.total_angle + Angle2;
				trig2.c_sta = TRIG_PRESS_DOWN;
			}
			/* 发射处理 */
			else if (trig2.c_sta == TRIG_PRESS_DOWN)
			{
				if (HAL_GetTick() - trig2.one_time >= 200)  //延时
				{
					trig2.c_sta = TRIG_ONE_DONE;
				}
			}
			/* 发射完成 */
			if (trig2.c_sta == TRIG_ONE_DONE)
			{
				single_shoot2_angle = moto_trigger2.total_angle;              
				trig2.c_sta = TRIG_INIT;
				shoot2.shoot_bullets++;//发射子弹计数
			}
			trig2.angle_ref = single_shoot2_angle;
		}
							else if(judge_data_limit.shooter_id2_17mm_speed_limit == 30)//射速优先
		{
			if((judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit-judge_recv_mesg.power_heat_data.shooter_17mm_2_barrel_heat) > 25)
				shoot2_delay = 500/(judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10+judge_recv_mesg.game_robot_state.power_management_shooter_output);
			else
				shoot2_delay = 500/(judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10);
				if (trig2.c_sta == TRIG_INIT)
			{
				trig2.one_time = HAL_GetTick();
				single_shoot2_angle = moto_trigger2.total_angle + Angle2;
				trig2.c_sta = TRIG_PRESS_DOWN;
			}
			/* 发射处理 */
			else if (trig2.c_sta == TRIG_PRESS_DOWN)
			{
				if (HAL_GetTick() - trig2.one_time >= shoot2_delay)  //延时
				{
					trig2.c_sta = TRIG_ONE_DONE;
				}
			}
		/* 发射完成 */
			if (trig2.c_sta == TRIG_ONE_DONE)
			{
				single_shoot2_angle = moto_trigger2.total_angle;
				trig2.c_sta = TRIG_INIT;
				shoot2.shoot_bullets++;//发射子弹计数
			}
			trig2.angle_ref = single_shoot2_angle;
		}
	}
	
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && (judge_recv_mesg.power_heat_data.shooter_17mm_2_barrel_heat 
		>= (judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit-10)))
		trig2.angle_ref = moto_trigger2.total_angle;                          //--------------------------------------------------------未处理 2025/2/26-----
	
	if(trig2.angle_ref%45!=0){//解决二连发问题
	int i=trig2.angle_ref%45;
		if(i>=25)trig2.angle_ref+=45-i;
		else trig2.angle_ref-=i;
	}
   //pid计算
  pid_calc(&pid_trigger2,moto_trigger2.total_angle,trig2.angle_ref);
  trig2.spd_ref = pid_trigger2.out;//
	pid_calc(&pid_trigger2_spd, moto_trigger2.speed_rpm, trig2.spd_ref);
//	pid_calc(&pid_trigger_spd, moto_trigger.speed_rpm, 2000);   //只用速度环控制拨盘
    
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && judge_recv_mesg.game_robot_state.power_management_shooter_output == 0)//摩擦轮被裁判系统断电后让拨盘停转
		pid_trigger2_spd.out = 0;
	//防止裁判系统异常出现超热量的情况
	if(!global_err.list[JUDGE_SYS_OFFLINE].err_exist && (judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value == 0 || judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit == 0))
		pid_trigger2_spd.out = 0;
}


//初始化，只在main调用一次
void shoot2_param_init(void)
{
  memset(&shoot2, 0, sizeof(shoot_t));
  
  shoot2.ctrl_mode      = SHOT_DISABLE;
  shoot2.para_mode			 = SHOOTNOR_MODE;
  
  memset(&trig2, 0, sizeof(trigger_t));
  
  trig2.shoot_spd			 = 0;
  trig2.c_shoot_spd     = 0;
  trig2.one_sta         = TRIG_INIT;
	shoot2.shoot_bullets = 0;
  
  /*热量控制*/
  PID_Struct_Init(&pid_heat_limit2, heat_limit_pid2[0], heat_limit_pid2[1], heat_limit_pid2[2], 7000, 0, INIT );
	/*热量控制时间间隔*/
	PID_Struct_Init(&pid_heat_time2, heat_time_pid2[0], heat_time_pid2[1], heat_time_pid2[2], 400, 0, INIT );
  
  /*摩擦轮*/
  for(int i=0;i<2;i++)
  {
    PID_Struct_Init(&pid_fric2[i], fric2_pid[0], fric2_pid[1], fric2_pid[2], 8000, 500, INIT ); 
  }

}

