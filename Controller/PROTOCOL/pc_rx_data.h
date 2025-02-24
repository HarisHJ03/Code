#ifndef _pc_rx_data_H
#define _pc_rx_data_H

#include "stm32f4xx.h"
#include "data_packet.h"

#define PC_RX_FIFO_BUFLEN 500

typedef enum
{
  GLOBAL_NORMAL        = 0,
  SOFTWARE_WARNING     = 1,
  SOFTWARE_ERROR       = 2,
  SOFTWARE_FATAL_ERROR = 3,
  GIMBAL_ERROR         = 4,
  CHASSIS_ERROR        = 5,
  HARAWARE_ERROR       = 6,
} err_level_e;

typedef enum
{
  NO_CONFIG      = 0,
  DEFAULT_CONFIG = 1,
  CUSTOM_CONFIG  = 3,
} struct_config_e;

/**
 * @brief  PC->��ص����ݶ�robot_rx_data (9�ֽ�) ������֡����5 + 9 + 2 = 16 (�ֽ�)
 */
#pragma pack(1)
typedef struct
{
  union
  {
    uint8_t data_byte;
    struct
    {
      uint8_t task_mode : 1;    // ģʽ ( 0 ����  / 1 ��С��)
      uint8_t visual_valid : 1; // �Ӿ���Чλ
      uint8_t shoot_vaild : 1;  // ������Чλ
			uint8_t navigation_determine : 1;  //������־λ
			uint8_t dodge_ctrl : 1;  
			uint8_t is_birth : 1;			
			uint8_t reserved : 2;//����λ
			} info;
  } mode_Union;
  float aim_pitch; // ŷ����(��) 
  float aim_yaw;   // ŷ����(��)
	float linear_speed_x; //���ٶ�x
	float linear_speed_y; //���ٶ�y
	float angular_speed_w; //���ٶ�w
	float init_yaw ;
	float end_yaw ;
	} robot_rx_data;
#pragma pack()

extern uart_dma_rxdata_t pc_rx_obj;
extern unpack_data_t pc_unpack_obj;
extern robot_rx_data pc_recv_mesg;


void pc_rx_param_init(void);
void pc_data_handler(uint8_t *p_frame);

#endif
