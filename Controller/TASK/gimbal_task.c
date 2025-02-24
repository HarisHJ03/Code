#include "gimbal_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "delay.h"
#include "modeswitch_task.h"
#include "comm_task.h"
#include "detect_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "sys_config.h"
#include "pc_rx_data.h"
#include "pid.h"
#include "stdlib.h"
#include "stdlib.h" //abs()函数
#include "math.h"   //fabs()函数
#include "kalman_filter.h"
#include "ladrc.h"

#include "chassis_task.h"
#include "shoot_task.h"
#include "judge_rx_data.h"
#include "bsp_vofa.h"


UBaseType_t gimbal_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

int direction=1;
int direction_change=0;
int initgimbal_flag=0;  //进入云台归中 这个标志位为0
ramp_t pit_ramp;
ramp_t yaw_ramp;
float a[2];

/*调PID波形的参数*/
//正弦波
#define amplitude  20  //设置振幅
double frequency = 3;  // 频率33.33Hz，对应周期0.03秒
float CYCLE_TIME=0.1;  // 执行周期为0.03秒

// 锯齿波
#define AMPLITUDE 20  // 锯齿波的振幅 
#define FREQUENCY 33.33  // 频率33.33Hz 
#define SAWTOOTH_CYCLE_TIME (1.0 / FREQUENCY)  // 周期0.03秒 
#define SAMPLE_RATE 5000  // 采样率1000Hz，每毫秒更新一次 


#if (SENTRY_NUM == SENTRY_1)
//float pit_init_pid[6] = {30, 0.05,0, 30, 0, 0};
//float yaw_init_pid[6] = {50, 0, 20, 90, 0, 5};
float pit_init_pid[6] = {0.3f, 0.001f, 0, 22000, 0, 0};
float yaw_init_pid[6] = {0.25f, 0.0f, 0, 40000, 0, 0};

// 普通参数（小陀螺与普通模式共用一套参数）
//float pit_pid[6] = {70,0.2,15,70,0,0};//
//float yaw_pid[6] = {50, 0, 20, 90, 0, 5};
float pit_pid[6] = {0.3f, 0.001f, 0, 10000, 0, 0};//IMU反馈
float yaw_pid[6] = {0.5f, 0.0f, 0.0f, 20000, 0, 0};

// 自瞄参数
//float pit_vision_pid[6] = {25,0,15,100,0,0};  // 前馈+微分 {50,0.01,0,180,0,0}
//float yaw_vision_pid[6] = {25,0.001,5,120,0,0}; // {30,0.001,5,170,0,10}; {30,0.001,5,170,0,10}    {25, 0.001, 10,140, 0, 10}//5月12日加前馈版本无微分
float pit_vision_pid[6] = {0.3f, 0.001f, 0, 10000, 0, 0};
float yaw_vision_pid[6] = {0.25f, 0.0f, 0.4f, 18000, 0, 0};


// 神符参数
float pit_buff_pid[6] = {27,0.13,5,100,0.1,0}; 
float yaw_buff_pid[6] = {30,0.1,5,80,0,5}; 
// 拨盘参数
float trig_pid[6] = {250, 0, 125, 10, 0, 0}; 
#elif (SENTRY_NUM == SENTRY_2)
float pit_init_pid[6] = {60, 0.01, 30, 60, 0.1, 0};
float yaw_init_pid[6] = {60, 0, 50, 80, 0, 40};

// 普通参数（小陀螺与普通模式共用一套参数）
float pit_pid[6] = {60, 0.01,30,60,0.1,0};
float yaw_pid[6] = {60, 0, 50, 80, 0, 40};


// 自瞄参数
float pit_vision_pid[6] = {30,0.22,5,170,0,0};
float yaw_vision_pid[6] = {20,0,5,78,0,5};
// 神符参数
float pit_buff_pid[6] = {27,0.13,5,100,0.1,0}; 
float yaw_buff_pid[6] = {30,0.1,5,80,0,5}; 

// 拨盘参数
float trig_pid[6] = {250, 0, 120, 10, 0, 0}; 

#endif

int shoot_flag=0;
//左上为正是+号。左下为正为-号
float pit_dir =  1.f;
float yaw_dir =  1.f;

float pit_ctrl_ffc;
float yaw_ctrl_ffc;

FFC p_ffc;
FFC y_ffc;

#define PIT_ANGLE_MAX       25   //5        // 高度限幅      
#define PIT_ANGLE_MIN       -25//
#define YAW_ANGLE_MAX       50
#define YAW_ANGLE_MIN      -50
/*简单速度前馈*/
typedef struct
{
  float Kc;
  float Outner_Expect[2];
  float out;
}Easy_FFC_t;
enum
{
  LAST = 0,
  NOW = 1,
};
/*EASY_FFT*/
void Easy_FFC_Init(Easy_FFC_t* easy_ffc ,float K)
{
	easy_ffc->Kc = K;
}
void Easy_FFC_Calc(Easy_FFC_t* easy_ffc ,float K ,float gimbal_expect_ref)
{
	easy_ffc->Outner_Expect[NOW] = gimbal_expect_ref;
	easy_ffc->out = K*(easy_ffc->Outner_Expect[NOW] - easy_ffc->Outner_Expect[LAST]);
	easy_ffc->Outner_Expect[LAST] = easy_ffc->Outner_Expect[NOW];
}
Easy_FFC_t yaw_easy_ffc;
Easy_FFC_t pit_easy_ffc;
/* 跟踪微分器（pc） */
TD_LADRC td_yaw = {
	.r = 27,
	.h = 0.001,
	.lambda = 90};

TD_LADRC td_pit = {
	.r = 50,
	.h = 0.001,
	.lambda = 90};

LADRC_NUM Vision_Angle_Pit =
{ 
	.h=0.002,//定时时间及时间步长
	.r=57,//跟踪速度参数
};
LADRC_NUM Vision_Angle_Yaw =
{ 
	.h=0.002,//定时时间及时间步长
	.r=30,//跟踪速度参数
};
void PRE_LADRC_TD(TD_LADRC *td_para, float Expect);	
/* 跟踪微分器（pc） */
	
////卡尔曼滤波（pc）
//kalman_filter_init_t pc_kalman_filter_para = {
//  .P_data = {2, 0, 0, 2},
//  .A_data = {1, 0.001, 0, 1},
//  .H_data = {1, 0, 0, 1},
//  .Q_data = {1, 0, 0, 1},
//  .R_data = {6000, 0, 0, 2000}
//};

//kalman_filter_t pc_kalman_filter;
/* 卡尔曼滤波（pc） */
kalman_filter_init_t pc_kalman_filter_para = {
	.P_data = {2, 0, 0, 2},
	.A_data = {1, 0.001, 0, 1},
	.H_data = {1, 0, 0, 1},
	.Q_data = {1, 0, 0, 1},
	.R_data = {6000, 0, 0, 2000}};
kalman_filter_t pc_kalman_filter_pitch;
kalman_filter_t pc_kalman_filter_yaw;
void gimbal_kalman_qr_set()
{
	mat_init(&pc_kalman_filter_pitch.Q, 2, 2, pc_kalman_filter_para.Q_data);
	mat_init(&pc_kalman_filter_pitch.R, 2, 2, pc_kalman_filter_para.R_data);
	mat_init(&pc_kalman_filter_yaw.Q, 2, 2, pc_kalman_filter_para.Q_data);
	mat_init(&pc_kalman_filter_yaw.R, 2, 2, pc_kalman_filter_para.R_data);
}
/* 卡尔曼滤波（pc） */

gimbal_t gimbal;
uint32_t gimbal_time,last_gimbal_time;
/* 角度求余在0~360度以内 */
static float angle_complement(float angle)
{
	while (angle < -180)				//如果角度为负数则累加到正数
		angle += 360.0f;			//累加周期为360度
	while (angle >180)
		angle -= 360.0f;
//	float tmp = angle - (int)angle;	//先保留角度的小数部分
//	angle = (int)angle % 360;		//再对角度的整数部分求余
//	tmp += angle;					//把小数部分加上求余后的整数
	return angle;						//返回结果
}

void gimbal_task(void *parm)
{
  uint32_t Signal;
	BaseType_t STAUS;
  
  while(1)
  {
    STAUS = xTaskNotifyWait((uint32_t) NULL, 
						    (uint32_t) INFO_GET_GIMBAL_SIGNAL, 
						    (uint32_t *)&Signal, 
							(TickType_t) portMAX_DELAY);
    if(STAUS == pdTRUE)
		{
			if(Signal & INFO_GET_GIMBAL_SIGNAL)
			{
        gimbal_time = HAL_GetTick() - last_gimbal_time;
        last_gimbal_time = HAL_GetTick();
				
				//卡尔曼滤波				
				// mat_init(&pc_kalman_filter.Q,2,2, pc_kalman_filter_para.Q_data);
				// mat_init(&pc_kalman_filter.R,2,2, pc_kalman_filter_para.R_data); 
				// 卡尔曼Q R矩阵参数设置
				//gimbal_kalman_qr_set();
				if(gimbal_mode == GIMBAL_TRACK_ARMOR)
        {
			    PID_Struct_Init(&pid_vision_pit, pit_vision_pid[0], pit_vision_pid[1], pit_vision_pid[2], 30, 10,DONE);
          PID_Struct_Init(&pid_vision_pit_spd, pit_vision_pid[3], pit_vision_pid[4], pit_vision_pid[5], 20000, 3000,DONE);
		
			    PID_Struct_Init(&pid_vision_yaw, yaw_vision_pid[0], yaw_vision_pid[1], yaw_vision_pid[2], 30, 10,DONE);	
		      PID_Struct_Init(&pid_vision_yaw_spd, yaw_vision_pid[3], yaw_vision_pid[4], yaw_vision_pid[5], 20000, 3000,DONE);
	      }
        else if(gimbal_mode == GIMBAL_SHOOT_BUFF)
	      {
		      PID_Struct_Init(&pid_buff_pit, pit_buff_pid[0], pit_buff_pid[1], pit_buff_pid[2], 30, 10,DONE);
          PID_Struct_Init(&pid_buff_pit_spd, pit_buff_pid[3], pit_buff_pid[4], pit_buff_pid[5], 20000, 3000,DONE);//25000//6000  
		 
			    PID_Struct_Init(&pid_buff_yaw, yaw_buff_pid[0], yaw_buff_pid[1], yaw_buff_pid[2], 30, 10,DONE);	
		      PID_Struct_Init(&pid_buff_yaw_spd, yaw_buff_pid[3], yaw_buff_pid[4], yaw_buff_pid[5], 20000, 3000,DONE);//25000
	      }
        else
	      {
         /* pit 轴电机的PID参数 */
         PID_Struct_Init(&pid_pit, pit_pid[0], pit_pid[1], pit_pid[2], 30, 10, DONE);//6000,500
         PID_Struct_Init(&pid_pit_spd, pit_pid[3], pit_pid[4], pit_pid[5], 20000, 3000, DONE);//30000//22000,4000

         /* yaw 轴电机的PID参数 */
         PID_Struct_Init(&pid_yaw, yaw_pid[0], yaw_pid[1], yaw_pid[2], 30, 10, DONE);
         PID_Struct_Init(&pid_yaw_spd, yaw_pid[3] , yaw_pid[4], yaw_pid[5], 20000, 3000, DONE); 
         /* 拨盘 电机的PID参数 */
         PID_Struct_Init(&pid_trigger, trig_pid[0], trig_pid[1], trig_pid[2], 8000, 0, DONE);
         PID_Struct_Init(&pid_trigger_spd, trig_pid[3], trig_pid[4], trig_pid[5],8000, 3000,DONE);
	      }   
        if(gimbal_mode != GIMBAL_RELEASE) //如果模式变化 不是释放模式，下面进行二重判断标志位是否归中，如果标志位不是归中位，进行归中
        {
          if(gimbal.state == GIMBAL_INIT_NEVER)
          {
            gimbal_mode = GIMBAL_INIT;
          }
          switch(gimbal_mode)
          {
            case GIMBAL_INIT:
            {
              PID_Struct_Init(&pid_yaw, yaw_init_pid[0], yaw_init_pid[1], yaw_init_pid[2], 6000, 500,DONE);
              PID_Struct_Init(&pid_yaw_spd, yaw_init_pid[3], yaw_init_pid[4], yaw_init_pid[5], 30000, 3000,DONE);
              
              PID_Struct_Init(&pid_pit, pit_init_pid[0], pit_init_pid[1], pit_init_pid[2], 20000, 500,DONE);	
              PID_Struct_Init(&pid_pit_spd, pit_init_pid[3], pit_init_pid[4], pit_init_pid[5], 20000, 15000,DONE);
              init_mode_handler(); //云台归中
            }break;
            /*云台底盘跟随模式*/
            case GIMBAL_NORMAL_MODE:
            {
							if(last_gimbal_mode != GIMBAL_NORMAL_MODE && last_gimbal_mode != GIMBAL_NAVIGATION_MODE) 
							{	
								PID_Clear(&pid_pit);
								PID_Clear(&pid_yaw);
								PID_Clear(&pid_pit_spd);
								PID_Clear(&pid_yaw_spd);
							}
              nomarl_handler();
            }break;
            /*小陀螺模式*/
            case GIMBAL_DODGE_MODE:
            {
							if(last_gimbal_mode != GIMBAL_DODGE_MODE ) 
							{	
								PID_Clear(&pid_pit);
								PID_Clear(&pid_yaw);
								PID_Clear(&pid_pit_spd);
								PID_Clear(&pid_yaw_spd);
							}
              dodge_handler();
            }break;
            /*打能量机关模式*/
            case GIMBAL_SHOOT_BUFF:
            {
							if(last_gimbal_mode != GIMBAL_SHOOT_BUFF ) 
							{	
								PID_Clear(&pid_buff_pit);
								PID_Clear(&pid_buff_yaw);
								PID_Clear(&pid_buff_pit_spd);
								PID_Clear(&pid_buff_yaw_spd);
							}
              shoot_buff_ctrl_handler();
            }break;
            /*自瞄模式*/
            case GIMBAL_TRACK_ARMOR:
            {
							if(last_gimbal_mode != GIMBAL_TRACK_ARMOR ) 
							{	
								PID_Clear(&pid_vision_pit);
								PID_Clear(&pid_vision_yaw);
								PID_Clear(&pid_vision_pit_spd);
								PID_Clear(&pid_vision_yaw_spd);
							}
/*						  if(shoot_flag)
								{
											shoot.fric_wheel_run = 1;
								if( pc_recv_mesg.mode_Union.info.shoot_vaild == 1) //judge_recv_mesg.game_state.game_progress==4 &&
								{
									shoot.shoot_cmd=1;
									shoot_flag=0;
								}
								}*/
              track_aimor_handler();
							if(judge_recv_mesg.game_state.game_progress == 4
								&& judge_recv_mesg.game_state.stage_remain_time>=0)
							{
								if( pc_recv_mesg.mode_Union.info.visual_valid) //judge_recv_mesg.game_state.game_progress == 4 &&
								{
									shoot.fric_wheel_run = 1;
									if(pc_recv_mesg.mode_Union.info.shoot_vaild == 1){
										shoot.c_shoot_cmd = 1;
									}    //----------------新加	
									else shoot.c_shoot_cmd = 0;
								}
								else shoot.fric_wheel_run = 1;
							}
						}
							break;
						case GIMBAL_NAVIGATION_MODE:
						{
							if(last_gimbal_mode != GIMBAL_NORMAL_MODE && last_gimbal_mode != GIMBAL_NAVIGATION_MODE) 
							{	
								PID_Clear(&pid_pit);
								PID_Clear(&pid_yaw);
								PID_Clear(&pid_pit_spd);
								PID_Clear(&pid_yaw_spd);
							}
							gimbal_navigaton_hander();
						}
            
            default:
            {
            }break;
          }
        }
        else
        {
          memset(glb_cur.gimbal_cur,0,sizeof(glb_cur.gimbal_cur));
          gimbal.state = GIMBAL_INIT_NEVER;
        }
//        sawtooth_wave(&gimbal.pid.yaw_angle_ref);
//			 update_sine_wave(&gimbal.pid.pit_angle_ref) ;
				
       a[0] =gimbal.pid.pit_angle_ref;
			 a[1] = gimbal.pid.pit_angle_fdb;
		   JustFloat_Send(a,2,USART1);
//-------------------------------------------模式判断转化分界线，下面各种模式是计算各种pid--------------------------------------------
		/* 将角度转换为0~360度 */
				gimbal.pid.yaw_angle_fdb = angle_complement(gimbal.pid.yaw_angle_fdb);
				gimbal.pid.yaw_angle_ref = angle_complement(gimbal.pid.yaw_angle_ref);
				/* 寻找最短半圈 */
				float tmp = gimbal.pid.yaw_angle_ref - gimbal.pid.yaw_angle_fdb;
				if (tmp >= 180.0f)//大于正半圈即 >=180.0f 就减去一圈
					gimbal.pid.yaw_angle_ref -= 360.0f;
				else if (tmp <= -180.0f)//大于负半圈即 <=-180.0f 就加上一圈
					gimbal.pid.yaw_angle_ref += 360.0f;
        /*pid计算-角度环*/
				if(gimbal_mode == GIMBAL_TRACK_ARMOR)	
				{
					pid_calc(&pid_vision_yaw, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_ref);
					pid_calc(&pid_vision_pit, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_ref);			
				}
				else if(gimbal_mode == GIMBAL_SHOOT_BUFF)
				{
					pid_calc(&pid_buff_yaw, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_ref);
					pid_calc(&pid_buff_pit, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_ref);			
				}
				else
				{
					pid_calc(&pid_yaw,gimbal.pid.yaw_angle_fdb , gimbal.pid.yaw_angle_ref);
					pid_calc(&pid_pit, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_ref);
				}
				
				
        
        /*速度环给定（串级）*/
				if(gimbal_mode == GIMBAL_TRACK_ARMOR)
				{
			//	Easy_FFC_Calc(&yaw_easy_ffc, yaw_easy_ffc.Kc, gimbal.pid.yaw_angle_ref);
      //	Easy_FFC_Calc(&pit_easy_ffc, pit_easy_ffc.Kc, gimbal.pid.pit_angle_ref);
					
			    gimbal.pid.yaw_spd_ref = pid_vision_yaw.out; //+ yaw_easy_ffc.out;
        	gimbal.pid.pit_spd_ref = pid_vision_pit.out/2;//+ pit_easy_ffc.out;
					
//				gimbal.pid.yaw_spd_ref = pid_vision_yaw.out;	 	
	//			gimbal.pid.pit_spd_ref = pid_vision_pit.out;
					gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance*2 ; //*2
					gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;  //速度环 线速度给定 为角速度*半径
					
				}
				
				
				else if(gimbal_mode == GIMBAL_SHOOT_BUFF)
				{
					gimbal.pid.yaw_spd_ref = pid_buff_yaw.out;	 
					gimbal.pid.pit_spd_ref = pid_buff_pit.out;
					gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance;	    
					gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;					
				}
				else
				{
					gimbal.pid.yaw_spd_ref = pid_yaw.out;	 
					gimbal.pid.pit_spd_ref = pid_pit.out;
					gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance;	    
					gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;					
				}
				

				
        

				/*pid计算-速度环*/
				if(gimbal_mode == GIMBAL_TRACK_ARMOR)
				{
					pid_calc(&pid_vision_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);
					pid_calc(&pid_vision_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);			
				}
				else if(gimbal_mode == GIMBAL_SHOOT_BUFF)
				{
					pid_calc(&pid_buff_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);
					pid_calc(&pid_buff_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);					
				}
				else
				{
					pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);					
					pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref); 
        }					
        
        
        
        if (gimbal_is_controllable())//将pid计算的out值代入glb_cur.gimbal_cur[]，后续通过can发出
        {
          if(gimbal_mode == GIMBAL_TRACK_ARMOR)
          {
						/*自瞄 加前馈*/
						pit_ctrl_ffc = getFeedforwardControl(&p_ffc, gimbal.pid.pit_angle_ref) + pid_vision_pit_spd.out;//
						yaw_ctrl_ffc = getFeedforwardControl(&y_ffc, gimbal.pid.yaw_angle_ref) + pid_vision_yaw_spd.out;// 
						glb_cur.gimbal_cur[0] = yaw_dir* yaw_ctrl_ffc;      //* yaw_ctrl_ffc;
						glb_cur.gimbal_cur[1] = pit_dir *pit_ctrl_ffc;//+pid_vision_pit_spd.out
						glb_cur.gimbal_cur[2] = pid_trigger_spd.out;
          }
					else if(gimbal_mode == GIMBAL_SHOOT_BUFF)
          {
            glb_cur.gimbal_cur[0] = yaw_dir * pid_buff_yaw_spd.out;
            glb_cur.gimbal_cur[1] = pit_dir * pid_buff_pit_spd.out;
            glb_cur.gimbal_cur[2] = pid_trigger_spd.out;
          }
          else
          {
            glb_cur.gimbal_cur[0] = yaw_dir * pid_yaw_spd.out;
            glb_cur.gimbal_cur[1] = pit_dir * pid_pit_spd.out;
            glb_cur.gimbal_cur[2] = pid_trigger_spd.out;
          }
        }
        else
        {
          memset(glb_cur.gimbal_cur, 0, sizeof(glb_cur.gimbal_cur));
          gimbal_mode = GIMBAL_RELEASE; //云台模式变为释放
          pid_trigger.iout = 0;
        }
        
        last_gimbal_mode = gimbal_mode;//获取上一次云台状态             
        xTaskGenericNotify( (TaskHandle_t) can_msg_send_Task_Handle, 
                            (uint32_t) GIMBAL_MOTOR_MSG_SIGNAL, 
                            (eNotifyAction) eSetBits, 
                            (uint32_t *)NULL );
      }
    }
    
    gimbal_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}




//PID参数初始化函数，主函数一开始调用
void gimbal_param_init(void)
{  
  memset(&gimbal, 0, sizeof(gimbal_t));
  
  gimbal.state = GIMBAL_INIT_NEVER;
  
  ramp_init(&pit_ramp, 1000);
  ramp_init(&yaw_ramp, 1000);
	
     //自瞄PID
			PID_Struct_Init(&pid_pit, pit_vision_pid[0], pit_vision_pid[1], pit_vision_pid[2], 1000, 500,INIT);
      PID_Struct_Init(&pid_pit_spd, pit_vision_pid[3], pit_vision_pid[4], pit_vision_pid[5], 6000, 3000,INIT);
		
			PID_Struct_Init(&pid_yaw, yaw_vision_pid[0], yaw_vision_pid[1], yaw_vision_pid[2], 1000, 500,INIT);	//1000
		  PID_Struct_Init(&pid_yaw_spd, yaw_vision_pid[3], yaw_vision_pid[4], yaw_vision_pid[5], 25000, 15000,INIT);

     //神符PID
		  PID_Struct_Init(&pid_pit, pit_buff_pid[0], pit_buff_pid[1], pit_buff_pid[2], 1000, 500,INIT);
      PID_Struct_Init(&pid_pit_spd, pit_buff_pid[3], pit_buff_pid[4], pit_buff_pid[5], 6000, 3000,INIT);  
		 
			PID_Struct_Init(&pid_yaw, yaw_buff_pid[0], yaw_buff_pid[1], yaw_buff_pid[2], 1000, 500,INIT);	
		  PID_Struct_Init(&pid_yaw_spd, yaw_buff_pid[3], yaw_buff_pid[4], yaw_buff_pid[5], 25000, 15000,INIT);
	 

	/* pit 轴电机的PID参数 */
	PID_Struct_Init(&pid_pit, pit_pid[0], pit_pid[1], pit_pid[2], 3000, 500, INIT);
  PID_Struct_Init(&pid_pit_spd, pit_pid[3], pit_pid[4], pit_pid[5], 25000, 4000, INIT); //6000

  /* yaw 轴电机的PID参数 */
  PID_Struct_Init(&pid_yaw, yaw_pid[0], yaw_pid[1], yaw_pid[2], 5000, 500, INIT); 
  PID_Struct_Init(&pid_yaw_spd, yaw_pid[3] , yaw_pid[4], yaw_pid[5], 25000, 15000, INIT);  
  
  /* 拨盘 电机的PID参数 */
  PID_Struct_Init(&pid_trigger, trig_pid[0], trig_pid[1], trig_pid[2], 8000, 0, INIT);
  PID_Struct_Init(&pid_trigger_spd, trig_pid[3], trig_pid[4], trig_pid[5],8000, 3000,INIT); 

	
//	//卡尔曼滤波初始化		 
//  kalman_filter_init(&pc_kalman_filter, &pc_kalman_filter_para);	
	/* 卡尔曼滤波初始化 */
	kalman_filter_init(&pc_kalman_filter_pitch, &pc_kalman_filter_para);
	kalman_filter_init(&pc_kalman_filter_yaw, &pc_kalman_filter_para);
	   /*前馈控制器初始化*/
	Easy_FFC_Init(&pit_easy_ffc,200);
	Easy_FFC_Init(&yaw_easy_ffc,300);
  /* ffc 初始化*/
	initFeedforwardParam(&p_ffc, 210, 20);
	initFeedforwardParam(&y_ffc, 200, 40);
  
}

/*注意！！！！！！
        注意！！！！！！
        注意！！！！！！
        relative_angle（相对角度）指的是当前角度与flash记录的归中角度的相对，而不是相对于陀螺仪
        gyro_angle（陀螺仪角度，我们称为绝对角度）才是与陀螺仪的零度的差
        总的来说，相对角度基于flash记录的归中角度
                  绝对角度基于陀螺仪
        */
        
//归中函数 在模式转化中调用
static void init_mode_handler(void)
{    initgimbal_flag=1;  //归中让底盘延迟不动
  /* PIT轴先归中 */
    //将当前角度赋给反馈值
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle; 
    //目标值设为零（斜坡函数需要时可加）
	gimbal.pid.pit_angle_ref = 0 ;//gimbal.sensor.pit_relative_angle * ( 1 - ramp_calc(&pit_ramp));利用斜坡函数使pitch轴的相对角度逐渐变为0
  /* 保持YAW轴不动 */
    //将当前相对角度赋给反馈值
  gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;
    //目标值设为当前相对角度
  gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_relative_angle;
	
  if(gimbal.pid.pit_angle_fdb >= -5.0f && gimbal.pid.pit_angle_fdb <= 5.0f)
  {
    /*  YAW轴归中*/   
    gimbal.pid.yaw_angle_ref = 0 ;//gimbal.sensor.yaw_relative_angle*( 1 - ramp_calc(&yaw_ramp));
    if (gimbal.pid.yaw_angle_fdb >= -2.0f && gimbal.pid.yaw_angle_fdb <= 2.0f)
    {
      //刷新零轴
        /*
        这里的刷新零轴意思是将当前的绝对角度传入gimbal.yaw_offset_angle
        后续用到绝对角度时用陀螺仪反馈的的绝对角度减去gimbal.yaw_offset_angle
        即为基于gimbal.yaw_offset_angle的值的绝对角度，
        此时可以认为gimbal.yaw_offset_angle为零轴，
        故这一步可以认为是刷新零轴
        */

      gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;

      gimbal.state = GIMBAL_INIT_DONE;
    }
  }
}

/*判断yaw轴是否有输入*/
static gimbal_state_t remote_is_action(void)
{
  if ((abs(rc.ch3) >= 10) || (abs(rc.mouse.x) >= 1) || (pc_recv_mesg.angular_speed_w >= 0.001 || pc_recv_mesg.angular_speed_w <= -0.001))
  {
      return IS_ACTION;
  }
  else
  {
      return NO_ACTION;
  }
}

uint8_t input_flag;
int no_action_time;
uint32_t debug_time = 1000;	
//普通模式
static void nomarl_handler(void)
{ 
	if(last_gimbal_mode != GIMBAL_NORMAL_MODE )  
  {   //刷新yaw零轴
        gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle; 
              /*由于pit我们使用的是相对角，所以一般不需要进行刷新，但这里我们将gimbal.pit_offset_angle
      等于当前的相对角加上pid计算中的差值，然后后续再将其赋给目标值以为pit轴抖动上一层保险，
      所以若将目标值与反馈值所对应零处认为是零轴，那么这个变量并非刷新零轴，但某种意义上来说也可以认为是，
      所以该变量与yaw轴刷新零轴的变量所对应*/
      //理论上来说pid调好了的情况下是不会抖动的，但保险多多何乐而不为呢(*^_^*)
		gimbal.pit_offset_angle = gimbal.sensor.pit_relative_angle
														+ (gimbal.pid.pit_angle_ref - gimbal.pid.pit_angle_fdb);
		if(last_gimbal_mode == GIMBAL_INIT || last_gimbal_mode == GIMBAL_SHOOT_BUFF 
			|| last_gimbal_mode == GIMBAL_NAVIGATION_MODE)
			gimbal.pid.yaw_angle_ref = 0;  //从归中模式或打符模式刚进来，把目标值设置为零
		else
		{
			gimbal.pid.yaw_angle_ref -= gimbal.yaw_offset_angle; //将目标值转化为相对于零轴的值（此前的值是相对于绝对角）
			gimbal.pid.pit_angle_ref = gimbal.pit_offset_angle;  //将gimbal.pit_offset_angle赋给目标值以为pit轴抖动上一层保险
		}
		
		input_flag = 1;
		no_action_time = HAL_GetTick();
  }
	gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;//将反馈值转化为相对于零轴的值（此前的值是相对于绝对角）
  gimbal.state = remote_is_action(); //判断yaw轴是否有输入
  
  if(direction_change==1)//检测到鼠标中间的滚轮向下滑动，则进行云台角度的变化
  {
        //当进入此判断时，gimbal.yaw_offset_angle会在get_gimbal_info函数中变化编码值4096（即6020的半圈）
		gimbal.pid.yaw_angle_fdb  = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;
		input_flag = 0;
		if(gimbal.pid.yaw_angle_ref-gimbal.pid.yaw_angle_fdb<3&&gimbal.pid.yaw_angle_ref-gimbal.pid.yaw_angle_fdb>-3)
			direction_change=0;
  }
else if((gimbal.last_state == NO_ACTION)        
        &&(gimbal.state == NO_ACTION)
        &&(HAL_GetTick() - no_action_time > debug_time) ){   //当yaw轴未进行操作
		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;
		gimbal.pid.yaw_angle_ref = 0;     //yaw轴到达归中位置
		input_flag = 0;
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;  //在yaw轴不动的情况下不断刷新0轴
  }
  else if (gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION)//当yaw轴停止操作
  {
		no_action_time = HAL_GetTick();  //更新无动作时间
		input_flag = 1;
  }else{     
		
		gimbal.pid.yaw_angle_fdb  = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;	
	  gimbal.pid.yaw_angle_ref += rm.yaw_v * GIMBAL_RC_MOVE_RATIO_YAW
                               +  km.yaw_v * GIMBAL_PC_MOVE_RATIO_YAW ;
    input_flag = 1;
	}
  if(pc_recv_mesg.mode_Union.info.navigation_determine == 1 && gimbal_mode == GIMBAL_NAVIGATION_MODE) //导航
	{
		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;
		gimbal.pid.yaw_angle_ref = 0;  
		if(pc_recv_mesg.angular_speed_w != 0){
		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;	
		gimbal.pid.yaw_angle_ref = pc_recv_mesg.angular_speed_w * 57.3;
		input_flag = 2;
		}
	}

  /*pitch轴*/
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
  gimbal.pid.pit_angle_ref -= rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT
                         - km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT;
  /* 软件限制pitch轴角度 */
  VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);  
  gimbal.last_state = remote_is_action();//获取上次输入的状态
}
float pit_patrol_direction = 1;
float yaw_patrol_direction = 1;
//小陀螺
static void dodge_handler(void)//反馈量是绝对角度
{ 
	if(pc_recv_mesg.linear_speed_x == 0 && pc_recv_mesg.linear_speed_y == 0 && pc_recv_mesg.angular_speed_w == 0)
	{
		pc_recv_mesg.mode_Union.info.dodge_ctrl = 1;
	}
/*导航模式*/
if(pc_recv_mesg.mode_Union.info.dodge_ctrl == 0 )
{
	if(last_gimbal_mode != GIMBAL_DODGE_MODE) //从跟随模式切换到小陀螺时，刷新零轴
    gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_gyro_angle;
  
		gimbal.state = remote_is_action(); //判断yaw轴是否有输入
  if(gimbal.last_state == NO_ACTION && gimbal.state == NO_ACTION)  //yaw轴未操作  
  {
    gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle;
    gimbal.pid.yaw_angle_ref = gimbal.yaw_offset_angle;
  }
  else if(gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION)//yaw轴停止操作
  {
			gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;//可在后面加减度数抵消小陀螺时yaw轴停止运动时的位移
  }
  else
  {
    gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle;
    gimbal.pid.yaw_angle_ref += pc_recv_mesg.angular_speed_w / RADIAN_COEF * 10;
  }
		
  /*pitch轴*/
	
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
//  gimbal.pid.pit_angle_ref -= (rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT)
//                         + km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT;

	/*pit轴巡航控制*/
//	gimbal.pid.pit_angle_ref += pit_patrol_direction * 0.25f; // yaw轴巡航速度0.185 0.45 !!!!!
	gimbal.pid.pit_angle_ref = 0;
//	/*yaw轴限位，确保它在我们限定的角度巡逻*/
  /* 软件限制pitch轴角度 */
//		if(gimbal.sensor.pit_relative_angle <= -29) // last_lost_angle为退出自瞄最后一帧的yaw轴数值，已改动，原本是<=-80
//		{
//			pit_patrol_direction = 1; //顺时针
//		}
//		else if (gimbal.sensor.pit_relative_angle >= 25) //已改动，原本是>=80
//		{
//			pit_patrol_direction = -1; //逆时针
//		}

	
  VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
  
  gimbal.last_state = remote_is_action();//获取上次输入的状态
}
else{
//巡航模式
		  if(last_gimbal_mode != GIMBAL_DODGE_MODE) //从跟随模式切换到小陀螺时，刷新零轴
    gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_gyro_angle;
  
		if(pc_recv_mesg.init_yaw == 0 && pc_recv_mesg.end_yaw == 0)
		{
			yaw_patrol_direction = 1;
		}
		else
		{
			if(gimbal.sensor.yaw_gyro_angle >= pc_recv_mesg.init_yaw)
			{
				yaw_patrol_direction = -1;
			}
			else if(gimbal.sensor.yaw_gyro_angle <= pc_recv_mesg.end_yaw)
			{
				yaw_patrol_direction = 1;
			}
		}
//		if(gimbal.sensor.yaw_gyro_angle >= 40)
//			{
//				yaw_patrol_direction = -1;
//			}
//			else if(gimbal.sensor.yaw_gyro_angle <= -40)
//			{
//				yaw_patrol_direction = 1;
//			}
		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle;
    gimbal.pid.yaw_angle_ref +=yaw_patrol_direction * 0.080f;//巡航 //0.080f 0.1//0.25
		
		if(pc_recv_mesg.init_yaw == pc_recv_mesg.end_yaw 
		&& pc_recv_mesg.init_yaw != 0)
				   gimbal.pid.yaw_angle_ref = pc_recv_mesg.init_yaw;//巡航 //0.080f 0.1
  /*pitch轴*/
	
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
//  gimbal.pid.pit_angle_ref -= (rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT)
//                         + km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT;
	/*pit轴巡航控制*/
	gimbal.pid.pit_angle_ref += pit_patrol_direction * 0.25f; // yaw轴巡航速度0.1 0.185 0.45 !!!!!
//	/*yaw轴限位，确保它在我们限定的角度巡逻*/
  /* 软件限制pitch轴角度 */
	if (gimbal.sensor.pit_relative_angle >= 8) // last_lost_angle为退出自瞄最后一帧的yaw轴数值，已改动，原本是<=-80
		{
			pit_patrol_direction = -1; //顺时针
		}
		else if (gimbal.sensor.pit_relative_angle <= -15) //已改动，原本是>=80
		{
			pit_patrol_direction = 1; //逆时针
		}
  VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
  
  gimbal.last_state = remote_is_action();//获取上次输入的状态
		}
}  
float yaw_a,pit_a;
extern float pit_rec_real;
extern float yaw_rec_real;
extern WorldTime_RxTypedef PC_KF_Time;

static void track_aimor_handler(void)
{
	/* 记录丢失角度 */
 static float lost_pit;
 static float lost_yaw;
  /*控制角度*/
 static float yaw_ctrl;
 static float pit_ctrl;
 static uint8_t  last_vision_status;
	pid_yaw.iout = 0;//清空其他模式yaw角度环iout累加值
	gimbal.yaw_offset_angle  = gimbal.sensor.yaw_gyro_angle;//备份陀螺仪数据，以免退出自瞄时冲突
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
  gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle;//yaw轴用陀螺仪
	
	//	float Vision_Speed = target_speed_calc(&Vision_speed_Struct, PC_KF_Time.WorldTime, pc_recv_mesg.gimbal_control_data.yaw_ref);

	/* 跟踪微分器 */
	// PRE_LADRC_TD(&td_pit, pit_rec_real);
	// PRE_LADRC_TD(&td_yaw, yaw_rec_real);
	/* 卡尔曼滤波；
	 * 0: 角度给定 
	 * 1: 速度给定  */
	//	float *pc_recv_result_pitch = kalman_filter_calc(&pc_kalman_filter_pitch,
	//																										td_pit.v1,
	//																										td_pit.v2);
	//	float *pc_recv_result_yaw = kalman_filter_calc  (&pc_kalman_filter_yaw,
	//																										td_yaw.v1,
	//	
	//																										td_yaw.v2);	
	
	
	
	/* TD跟踪微分处理 */
	LADRC_TD(&Vision_Angle_Pit, gimbal.sensor.pit_relative_angle - pc_recv_mesg.aim_pitch);
	LADRC_TD(&Vision_Angle_Yaw,  gimbal.sensor.yaw_gyro_angle + pc_recv_mesg.aim_yaw);
	
//	
//	LADRC_TD(&Vision_Angle_Pit, pc_recv_mesg.aim_pitch);
//	LADRC_TD(&Vision_Angle_Yaw,  pc_recv_mesg.aim_yaw);
	
	if(last_gimbal_mode != GIMBAL_TRACK_ARMOR)//进自瞄时，防止疯转冲突
		{
			  pit_ctrl = gimbal.sensor.pit_relative_angle;
			  yaw_ctrl = gimbal.sensor.yaw_gyro_angle;
			
		}	
  		
		
	
	if (pc_recv_mesg.mode_Union.info.visual_valid == 1)
{
			//yaw_ctrl = Vision_Angle_Yaw.v1; //v1为角度，v2为导数
			pit_ctrl = Vision_Angle_Pit.v1;
//			yaw_ctrl = Vision_Angle_Yaw.v1;
   // 相对角
		if (chassis_mode == CHASSIS_DODGE_MODE)
			  yaw_ctrl = gimbal.sensor.yaw_gyro_angle + pc_recv_mesg.aim_yaw; // 补偿小陀螺自瞄时底盘的反方向力
	else
			yaw_ctrl = gimbal.sensor.yaw_gyro_angle + pc_recv_mesg.aim_yaw;
 // 连杆云台步兵的电机和电机直连步兵是反的,pc_recv_mesg.aim_pitch的值得根据实际的机械结构进行改变
	//		pit_ctrl = gimbal.sensor.pit_relative_angle - pc_recv_mesg.aim_pitch;

	 // 绝对角
//	if (chassis_mode == CHASSIS_DODGE_MODE)
//			yaw_ctrl =pc_recv_mesg.aim_yaw; // 补偿小陀螺自瞄时底盘的反方向力
//	else
//			yaw_ctrl = pc_recv_mesg.aim_yaw;
 // 连杆云台步兵的电机和电机直连步兵是反的,pc_recv_mesg.aim_pitch的值得根据实际的机械结构进行改变
//			pit_ctrl = - pc_recv_mesg.aim_pitch;

	last_vision_status = 1;
  }
	/*视觉无效处理*/
	 else if (pc_recv_mesg.mode_Union.info.visual_valid == 0)		
	  {
			if (last_vision_status == 1)
			{		
				lost_yaw = gimbal.sensor.yaw_gyro_angle ;//pc_recv_result[0];//yaw_msg_t.filtered_value;			
				lost_pit = pit_ctrl;//-pc_recv_result[1];//pit_msg_t.filtered_value;//丢失目标后云台不动
				last_vision_status = 2;
				yaw_ctrl = lost_yaw;
				pit_ctrl = lost_pit;
				
			}
			last_vision_status = 0;
		}
	 //PC通讯出现问题
  if (pc_recv_mesg.aim_yaw >= 180 || pc_recv_mesg.aim_yaw <= -180)
	yaw_ctrl = gimbal.sensor.yaw_gyro_angle;
  if (pc_recv_mesg.aim_pitch >= 180 || pc_recv_mesg.aim_pitch <= -180)
	pit_ctrl = gimbal.sensor.pit_relative_angle;

//	 yaw_ctrl= yaw_a;
//	pit_ctrl = pit_a;
  gimbal.pid.yaw_angle_ref = yaw_ctrl;
  gimbal.pid.pit_angle_ref = pit_ctrl;
//		gimbal.pid.yaw_angle_ref = 0;
//		gimbal.pid.pit_angle_ref = 0;
	
//	 yaw_ctrl= yaw_a;
//	pit_ctrl = pit_a;
	/* 软件限制pitch轴角度 */
		VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
}

//导航模式
static void gimbal_navigaton_hander()
{
	if(last_gimbal_mode != GIMBAL_NAVIGATION_MODE )  
  {   //刷新yaw零轴
        gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle; 
              /*由于pit我们使用的是相对角，所以一般不需要进行刷新，但这里我们将gimbal.pit_offset_angle
      等于当前的相对角加上pid计算中的差值，然后后续再将其赋给目标值以为pit轴抖动上一层保险，
      所以若将目标值与反馈值所对应零处认为是零轴，那么这个变量并非刷新零轴，但某种意义上来说也可以认为是，
      所以该变量与yaw轴刷新零轴的变量所对应*/
      //理论上来说pid调好了的情况下是不会抖动的，但保险多多何乐而不为呢(*^_^*)
		gimbal.pit_offset_angle = gimbal.sensor.pit_relative_angle
														+ (gimbal.pid.pit_angle_ref - gimbal.pid.pit_angle_fdb);
		if(last_gimbal_mode == GIMBAL_NORMAL_MODE || last_gimbal_mode == GIMBAL_DODGE_MODE )
			gimbal.pid.yaw_angle_ref = 0;  //从归中模式或打符模式刚进来，把目标值设置为零
		else
		{
			gimbal.pid.yaw_angle_ref -= gimbal.yaw_offset_angle; //将目标值转化为相对于零轴的值（此前的值是相对于绝对角）
			gimbal.pid.pit_angle_ref = gimbal.pit_offset_angle;  //将gimbal.pit_offset_angle赋给目标值以为pit轴抖动上一层保险
		}
		
		input_flag = 2;
		no_action_time = HAL_GetTick();
  }
	gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;//将反馈值转化为相对于零轴的值（此前的值是相对于绝对角）
  gimbal.state = remote_is_action(); //判断yaw轴是否有输入
  
 if((gimbal.last_state == NO_ACTION)        
        &&(gimbal.state == NO_ACTION)
        &&(HAL_GetTick() - no_action_time > debug_time) ){   //当yaw轴未进行操作
		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;
		gimbal.pid.yaw_angle_ref = 0;     //yaw轴到达归中位置
		input_flag = 0;
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_gyro_angle;  //在yaw轴不动的情况下不断刷新0轴
  }
  else if (gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION)//当yaw轴停止操作
  {
		no_action_time = HAL_GetTick();  //更新无动作时间
		input_flag = 2;
  }else{     
		
		gimbal.pid.yaw_angle_fdb  = gimbal.sensor.yaw_gyro_angle - gimbal.yaw_offset_angle;	
		gimbal.pid.yaw_angle_ref += pc_recv_mesg.angular_speed_w / RADIAN_COEF * 10;//算法发过来的是rad/s需解算成deg/s(该值经实测得出，参数无具体特指)
    input_flag = 2;
	}

  /*pitch轴*/
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
  gimbal.pid.pit_angle_ref -= rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT
                         - km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT;
  /* 软件限制pitch轴角度 */
  VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);  
  gimbal.last_state = remote_is_action();//获取上次输入的状态
}

static void shoot_buff_ctrl_handler(void)
{
	/*记录丢失角度*/
	static float lost_pit;
	static float lost_yaw;
	
	/*控制角度*/
  static float yaw_ctrl;
  static float pit_ctrl;
	
	static uint8_t  last_vision_status;
	
	/*获取反馈值*/
	gimbal.yaw_offset_angle  = gimbal.sensor.yaw_gyro_angle; //备份陀螺仪数据，以免退出自瞄时冲突
  gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_relative_angle;
  gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_relative_angle;

  //卡尔曼滤波；
	
	//0: yaw轴给定 ;  
  //1: pit轴给定 ;	
//  float *pc_recv_result   = kalman_filter_calc(&pc_kalman_filter,   pc_recv_mesg.gimbal_control_data.yaw_ref,   pc_recv_mesg.gimbal_control_data.pit_ref);
 
	if (pc_recv_mesg.mode_Union.info.visual_valid == 1)
	{
		last_vision_status = 1;
		yaw_ctrl = pc_recv_mesg.aim_yaw;
		pit_ctrl = -pc_recv_mesg.aim_pitch;
	}

	/*视觉无效处理*/
	else if (pc_recv_mesg.mode_Union.info.visual_valid == 0)
	{
		if (last_vision_status == 1)
		{
			lost_yaw = pc_recv_mesg.aim_yaw;
			lost_pit = -pc_recv_mesg.aim_pitch; // 丢失目标后云台不动
			last_vision_status = 2;
		}
		yaw_ctrl = lost_yaw;
		pit_ctrl = lost_pit;
	}

	gimbal.pid.yaw_angle_ref = yaw_ctrl;
	gimbal.pid.pit_angle_ref = pit_ctrl;

	/*给定角度限制*/
	VAL_LIMIT(gimbal.pid.yaw_angle_ref, -45, 45);
	VAL_LIMIT(gimbal.pid.pit_angle_ref, -28, 18); // 25,15
}



float speed_threshold = 10.0f;
//time_raw,pitch_angel_raw等都来自pc
//此函数可算出速度，调用时position实参为pitch_angel_raw
//函数实为给结构体speed_calc_data_t赋值，初始化参数
float target_speed_calc(speed_calc_data_t *S, uint32_t time, float position)
{
  S->delay_cnt++;

  if (time != S->last_time)
  {
    S->speed = (position - S->last_position) / (time - S->last_time) * 1000;		//计算速度（当前位置-上次位置）/
#if 1
    if ((S->speed - S->processed_speed) < -speed_threshold)		//speed_threshold=10.0f,S->processed_speed未赋过值
    {
        S->processed_speed = S->processed_speed - speed_threshold;
    }
    else if ((S->speed - S->processed_speed) > speed_threshold)
    {
        S->processed_speed = S->processed_speed + speed_threshold;
    }
    else 
#endif
      S->processed_speed = S->speed;													//将前面计算出的速度赋予S->processed_speed
    
    S->last_time = time;
    S->last_position = position;
    S->last_speed = S->speed;
    S->delay_cnt = 0;
  }
  
  if(S->delay_cnt > 200) // delay 200ms speed = 0
  {
    S->processed_speed = 0;
  }

  return S->processed_speed;
}

void initFeedforwardParam(FFC *vFFC,float a,float b)
{
	//初始化前馈补偿
	vFFC->a = a;
	vFFC->b = b;
	vFFC->lastRin = 0;
	vFFC->perrRin = 0;
	vFFC->rin = 0;
}

/*实现前馈控制器*/
float getFeedforwardControl(FFC* vFFC,float v)//yaw轴
{
	vFFC->rin = v;
	float result = vFFC->a * (vFFC->rin - vFFC->lastRin) + vFFC->b * (vFFC->rin - 2 * vFFC->lastRin + vFFC->perrRin);
	vFFC->perrRin = vFFC->lastRin;
	vFFC->lastRin = vFFC->rin;
	return result;
}

/* 带相位超前的跟踪微分器 */
void PRE_LADRC_TD(TD_LADRC *td_para, float Expect)
{
	td_para->fh = -td_para->r * td_para->r * (td_para->v1 - Expect) - 2 * td_para->r * td_para->v2;
	td_para->v1 += td_para->v2 * td_para->h;
	td_para->v2 += td_para->fh * td_para->h;
	// 相位超前 v1+lamda*h*v2
	td_para->pre_v1 = td_para->v1 + td_para->lambda * td_para->fh * td_para->v2;
}

/*用于调pid的正弦波*/
void update_sine_wave(float* value) 
{
    // 假设正弦波从0开始，每次增加2π/频率
    static double phase = 0.0;
    *value = amplitude * sin(2 * PI * frequency * phase);
    phase += 1 / CYCLE_TIME; // 更新相位
    if (phase >= 1) phase -= 1; // 保持相位在0和1之间

}


/*调自瞄的锯齿波*/
void sawtooth_wave(float *value)
{
		static double phase = 0.0;
		
    static float increment = AMPLITUDE / (SAWTOOTH_CYCLE_TIME * SAMPLE_RATE);

      *value += increment;

    if (*value  >= AMPLITUDE)
    {
        *value  = 0.0;  // 重置锯齿波 
    }

	
}

