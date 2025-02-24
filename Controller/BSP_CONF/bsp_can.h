#ifndef _bsp_can_H
#define _bsp_can_H

#include "stm32f4xx.h"


#define FILTER_BUF 5
/* 电机编码值 和 角度（度） 的比率 */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)

/* CAN send and receive ID */
typedef enum
{
	CAN_CHASSIS_ALL_ID   = 0x200,
  CAN_3508_M1_ID       = 0x201,
  CAN_3508_M2_ID       = 0x202,
  CAN_3508_M3_ID       = 0x203,
  CAN_3508_M4_ID       = 0x204,
	
	CAN_GIMBAL_ALL_ID    = 0x1ff,
	CAN_YAW_MOTOR_ID     = 0x205,
	CAN_PIT_MOTOR_ID     = 0x206,
//	CAN_TRIGGER_MOTOR_ID = 0x207,
//  CAN_TRIGGER2_MOTOR_ID = 0x208,                //第二个拨盘


	CAN_CAP_ID           = 0x210,
  CAN_SUPER_CAP_ID		 = 0x211,

} can1_msg_id_e;

typedef enum
{
	CAN_FRIC_ALL_ID      = 0x200,
  CAN_FRIC_M1_ID       = 0x201,
  CAN_FRIC_M2_ID       = 0x202, 
  CAN_FRIC2_M1_ID       = 0x203,          //-----第二组摩擦轮
  CAN_FRIC2_M2_ID       = 0x204, 
	
	CAN_TRIGGER_MOTOR_ID = 0x205,  
	CAN_TRIGGER2_MOTOR_ID = 0x206,
	CAN_MPU_ID           = 0x401,

	
} can2_msg_id_e;

typedef struct
{
  uint16_t ecd;
  uint16_t last_ecd;
  
  int16_t  speed_rpm;
  int16_t  given_current;

  int32_t  round_cnt;
  int32_t  total_ecd;
  int32_t  total_angle;
  
  uint16_t offset_ecd;
  uint32_t msg_cnt;
  
  int32_t  ecd_raw_rate;
  int32_t  rate_buf[FILTER_BUF];
  uint8_t  buf_cut;
  int32_t  filter_rate;
} moto_measure_t;

typedef struct
{
  float pit_angle;
  float yaw_angle;
  int16_t pit_spd;
  int16_t yaw_spd;
}mpu_data_t;

extern moto_measure_t moto_chassis[];
extern moto_measure_t moto_pit;
extern moto_measure_t moto_yaw;
extern moto_measure_t moto_trigger;
extern moto_measure_t moto_fric[4];
extern mpu_data_t mpu_data;

void encoder_data_handler(moto_measure_t* ptr, CanRxMsg *message);
void get_moto_offset(moto_measure_t* ptr, CanRxMsg *message);

//void send_gimbal_cur(int16_t pit_iq,int16_t yaw_iq, int16_t trigger_iq);
void send_gimbal_cur(int16_t pit_iq,int16_t yaw_iq);
void send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void send_fric_cur(int16_t iq3, int16_t iq4);
void send_cap_power_can(uint16_t power);
void send_trigger_cur(int16_t trigger_iq);

//------------第二个头
extern moto_measure_t moto_trigger2;
//extern moto_measure_t moto_fric2[2];

#endif 

