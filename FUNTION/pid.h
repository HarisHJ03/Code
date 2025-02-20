#ifndef _pid_H
#define _pid_H

#include "stm32f4xx.h"

typedef enum
{
	INIT,
	DONE,
}INIT_STATUS;

enum
{
	NOW_ERR = 0,
	LAST_ERR,
	LLAST_ERR,
};

typedef struct pid
{
	float set;
	float get;
	float error[3];
  
	float kp;
	float ki;
	float kd;
	
	float pout;
	float iout;
	float dout;
	float out; 
	
	
	int32_t maxout;
	int32_t integral_limit;
  float output_deadband;//死区
	
	void (*f_pid_init)(struct pid *pid_t,
										float p,
										float i,
										float d,
										int32_t max_out,
										int32_t integral_limit);
	void (*f_pid_reset)(struct pid *pid_t,
										float p,
										float i,
										float d);
										
}pid_t;


float pid_calc(pid_t *pid,float get,float set);
float fuzzy_pid_calc(pid_t *pid, float get, float set);

void PID_Struct_Init(pid_t *pid,
										 float p,
										 float i,
										 float d,
										 int32_t max_out,
										 int32_t integral_limit,
										 INIT_STATUS init_status);
										 
void PID_Clear(pid_t *pid);//手动清零

extern pid_t pid_yaw;
extern pid_t pid_pit;
extern pid_t pid_yaw_spd;
extern pid_t pid_pit_spd;
extern pid_t pid_spd[4];
extern pid_t pid_chassis_angle;
extern pid_t pid_trigger;
extern pid_t pid_trigger_spd;
extern pid_t pid_fric[2];
extern pid_t pid_heat_limit;
extern pid_t pid_power;
extern pid_t pid_heat_time;
extern pid_t pid_imu_tmp;
										 
										 
//---------第二个头-----------
//----------第二个头
extern pid_t pid_trigger2 ;
extern pid_t pid_trigger2_spd ;
extern pid_t pid_fric2[2];
extern pid_t pid_heat_limit2;
extern pid_t pid_power2;
extern pid_t pid_heat_time2;
extern pid_t pid_speedlimit2;
  
/*自瞄*/
extern pid_t pid_buff_yaw;
extern pid_t pid_buff_pit;
extern pid_t pid_buff_yaw_spd;
extern pid_t pid_buff_pit_spd;  	
/*自瞄*/
extern pid_t pid_vision_yaw;
extern pid_t pid_vision_pit;
extern pid_t pid_vision_yaw_spd;
extern pid_t pid_vision_pit_spd;                     
/*归中*/                     
extern pid_t pid_init_yaw;             
extern pid_t pid_init_pit;            
extern pid_t pid_init_yaw_spd;         
extern pid_t pid_init_pit_spd;                    
                     
extern pid_t pid_chassis_vw;			
extern pid_t chassis_vw_cap;	

extern pid_t pid_speedlimit;
extern pid_t pid_chassis_power_buffer;		  
#endif


