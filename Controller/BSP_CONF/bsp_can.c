#include "bsp_can.h"
#include "detect_task.h"
#include "sys_config.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "stdlib.h"
#include "delay.h"

CanRxMsg rx1_message;
CanRxMsg rx2_message;
//moto_chassis
moto_measure_t moto_chassis[4];        //底盘       3508
moto_measure_t moto_pit;               //pit轴      6020
moto_measure_t moto_yaw;               //yaw轴      6020
moto_measure_t moto_trigger;           //拨盘       2006
moto_measure_t moto_fric[4];           //摩擦轮     3508  包含四个摩擦轮0123
mpu_data_t mpu_data;

//--------第二个头 
moto_measure_t moto_trigger2;           //拨盘2       2006
//moto_measure_t moto_fric2[2];           //摩擦轮2     3508

float bmi160_yaw_angle,bmi160_yaw_acc,bmi160_pit_acc;

float angle;
float *p_angle = &angle;//p_angle指针保存变量angle的地址

float *yaw;
int16_t *gz,*gy;
static void STD_CAN_RxCpltCallback(CAN_TypeDef *_hcan,CanRxMsg *message)
{
	if(_hcan == CAN1)
	{
		switch(message->StdId)
		{
			case CAN_3508_M1_ID://底盘四个电机接收CAN数据
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			{
				static uint8_t i = 0;
                //处理电机ID号
                i = message->StdId - CAN_3508_M1_ID;
                //处理电机数据宏函数
                moto_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[i], message) : encoder_data_handler(&moto_chassis[i], message);
                //记录时间
                err_detector_hook(CHASSIS_M1_OFFLINE + i);

			}break;
			
			case CAN_YAW_MOTOR_ID://yaw轴接收CAN数据
			{
				encoder_data_handler(&moto_yaw, message);
				err_detector_hook(GIMBAL_YAW_OFFLINE);
			}break;
			
			
//			case CAN_TRIGGER_MOTOR_ID://拨盘接收CAN数据
//			{
//				moto_trigger.msg_cnt++ <= 50 ? get_moto_offset(&moto_trigger, message) : encoder_data_handler(&moto_trigger, message);
//        err_detector_hook(TRIGGER_MOTO_OFFLINE);  
//			}break;
//			
//				case CAN_TRIGGER2_MOTOR_ID://拨盘2接收CAN1数据  -------------------2024/3/3新增加
//			{
//				moto_trigger2.msg_cnt++ <= 50 ? get_moto_offset(&moto_trigger2, message) : encoder_data_handler(&moto_trigger2, message);
//        err_detector_hook(TRIGGER2_MOTO_OFFLINE);      
//			}break;
			
			case CAN_SUPER_CAP_ID://超电
			{
					chassis.CapData[0] = (float)((rx1_message.Data[1] << 8 | rx1_message.Data[0])/100.f);		//输入电压
					chassis.CapData[1] = (float)((rx1_message.Data[3] << 8 | rx1_message.Data[2])/100.f);		//电容电压
					chassis.CapData[2] = (float)((rx1_message.Data[5] << 8 | rx1_message.Data[4])/100.f);		//输入电流
					chassis.CapData[3] = (float)((rx1_message.Data[7] << 8 | rx1_message.Data[6])/100.f);		//设定功率	
			}break;
			
			case CAN_PIT_MOTOR_ID://pit轴
			{
				encoder_data_handler(&moto_pit, message);
				err_detector_hook(GIMBAL_PIT_OFFLINE);
			}break;
			
			default:
			{
			}break;
		}
	}
	else
	{
		switch(message->StdId)
    {
      case CAN_FRIC_M1_ID://摩擦轮接收CAN数据
      case CAN_FRIC_M2_ID:
			case CAN_FRIC2_M1_ID://摩擦轮接收CAN数据     //-----------------------新加
      case CAN_FRIC2_M2_ID:
      {
        static uint8_t i = 0;
        //处理电机ID号
        i = message->StdId - CAN_FRIC_M1_ID;
        //处理电机数据函数
        moto_fric[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_fric[i], message) : encoder_data_handler(&moto_fric[i], message);
        err_detector_hook(FRI_MOTO1_OFFLINE + i);
      }break;
			
//			 case CAN_FRIC_M1_ID://摩擦轮接收CAN数据
//			 {
//				  moto_fric[0].msg_cnt++ <= 50 ? get_moto_offset(&moto_fric[0], message) : encoder_data_handler(&moto_fric[0], message);
//			 }
//      	 case CAN_FRIC_M2_ID://摩擦轮接收CAN数据
//			 {
//				  moto_fric[1].msg_cnt++ <= 50 ? get_moto_offset(&moto_fric[1], message) : encoder_data_handler(&moto_fric[1], message);
//			 }
//				 case CAN_FRIC2_M1_ID://摩擦轮接收CAN数据
//			 {
//				  moto_fric2[0].msg_cnt++ <= 50 ? get_moto_offset(&moto_fric2[0], message) : encoder_data_handler(&moto_fric2[0], message);
//			 }
//				 case CAN_FRIC2_M2_ID://摩擦轮接收CAN数据
//			 {
//				  moto_fric2[1].msg_cnt++ <= 50 ? get_moto_offset(&moto_fric2[1], message) : encoder_data_handler(&moto_fric2[1], message);
//			 }
						case CAN_TRIGGER_MOTOR_ID://拨盘接收CAN2数据
			{
				moto_trigger.msg_cnt++ <= 50 ? get_moto_offset(&moto_trigger, message) : encoder_data_handler(&moto_trigger, message);
        err_detector_hook(TRIGGER_MOTO_OFFLINE);  
			}break;
//			
//				case CAN_TRIGGER2_MOTOR_ID://拨盘2接收CAN2数据  -------------------2024/3/3新增加
//			{
//				moto_trigger2.msg_cnt++ <= 50 ? get_moto_offset(&moto_trigger2, message) : encoder_data_handler(&moto_trigger2, message);
//        err_detector_hook(TRIGGER2_MOTO_OFFLINE);      
//			}break;
			
      case CAN_MPU_ID://陀螺仪接收CAN数据及解算
      {

        uint8_t *px = rx2_message.Data;
        for(int i = 0; i < 4; i++)
        {
          *((uint8_t*)p_angle + i) = *(px + i);
        }

//        gimbal.sensor.yaw_gyro_angle = angle;
//        bmi160_yaw_angle=angle;
//				gimbal.sensor.yaw_palstance = (int16_t)(rx2_message.Data[5] << 8 | rx2_message.Data[4]);
//				bmi160_yaw_acc=(int16_t)(rx2_message.Data[5] << 8 | rx2_message.Data[4]);
//				bmi160_yaw_acc/= 16.384f;
//        gimbal.sensor.yaw_palstance /= 16.384f;
//								
//        gimbal.sensor.pit_palstance = -(int16_t)(rx2_message.Data[7] << 8 | rx2_message.Data[6]);
//        gimbal.sensor.pit_palstance /=-16.384f;//左上为正是+号。左下为正为-号
//				bmi160_pit_acc=(int16_t)(rx2_message.Data[7] << 8 | rx2_message.Data[6]);
//				bmi160_pit_acc /=16.384f;
//        err_detector_hook(IMU_OFFLINE);
			}break;

			

      
      default:
      {
      }break;
    }
	}
}


void encoder_data_handler(moto_measure_t* ptr, CanRxMsg *message)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
  
  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
	if(ptr == &moto_trigger || ptr == &moto_trigger2)
		ptr->total_angle = ptr->total_ecd / (ENCODER_ANGLE_RATIO*36);
	else
		ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;
	
	ptr->speed_rpm     = (int16_t)(message->Data[2] << 8 | message->Data[3]);
  ptr->given_current = (int16_t)(message->Data[4] << 8 | message->Data[5]);

}


/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, CanRxMsg *message)
{
    ptr->ecd        = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
    ptr->offset_ecd = ptr->ecd;
}


/**
  * @brief  send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param  current value corresponding motor(yaw/pitch/trigger)
  */
/*can发送电流函数*/
/*该函数can1发送拨盘电机和yaw轴pit轴电机电流*/
//void send_gimbal_cur(int16_t pit_iq,int16_t yaw_iq, int16_t trigger_iq)
//{
//    CanTxMsg TxMessage;
//    TxMessage.StdId = CAN_GIMBAL_ALL_ID;//标准标识符
//    TxMessage.IDE = CAN_ID_STD;         // 定义标识符的类型为标准标识符
//    TxMessage.RTR = CAN_RTR_DATA;       //数据帧
//    TxMessage.DLC = 0x08;               //数据长度为0x08
//    TxMessage.Data[0] = yaw_iq >> 8;
//    TxMessage.Data[1] = yaw_iq;
//    TxMessage.Data[2] = pit_iq>> 8;
//    TxMessage.Data[3] = pit_iq;
//    TxMessage.Data[4] = trigger_iq >> 8;
//    TxMessage.Data[5] = trigger_iq;                   
//    TxMessage.Data[6] = 0;
//    TxMessage.Data[7] = 0;                        

//    CAN_Transmit(GIMBAL_CAN, &TxMessage);
//}

/*该函数can1发送yaw轴pit轴电机电流*/
void send_gimbal_cur(int16_t pit_iq,int16_t yaw_iq)        
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_GIMBAL_ALL_ID;//标准标识符
    TxMessage.IDE = CAN_ID_STD;         // 定义标识符的类型为标准标识符
    TxMessage.RTR = CAN_RTR_DATA;       //数据帧
    TxMessage.DLC = 0x04;               //数据长度为0x08
    TxMessage.Data[0] = yaw_iq >> 8;
    TxMessage.Data[1] = yaw_iq;
    TxMessage.Data[2] = pit_iq >> 8;
    TxMessage.Data[3] = pit_iq;
                     
    CAN_Transmit(GIMBAL_CAN, &TxMessage);
}
/*该函数can2发送拨盘电机电流*/
void send_trigger_cur(int16_t trigger_iq)        //---------------------2024/3/3新增加
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_GIMBAL_ALL_ID;//标准标识符
    TxMessage.IDE = CAN_ID_STD;         // 定义标识符的类型为标准标识符
    TxMessage.RTR = CAN_RTR_DATA;       //数据帧
    TxMessage.DLC = 0x02;     //0X04          //数据长度为0x08

    TxMessage.Data[0] =trigger_iq >> 8;
	  TxMessage.Data[1] =trigger_iq; 

    CAN_Transmit(FRIC_CAN, &TxMessage);
}

/*该函数can1发送底盘四个3508电机电流*/
void send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = iq1 >> 8;
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
    TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;
    TxMessage.Data[6] = iq4 >> 8;
    TxMessage.Data[7] = iq4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}
/*若摩擦轮使用3508电机，则使用该函数发送电流*/
void send_fric_cur(int16_t iq3, int16_t iq4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_FRIC_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = iq3 >> 8;
    TxMessage.Data[1] = iq3;
    TxMessage.Data[2] = iq4 >> 8;
    TxMessage.Data[3] = iq4;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(FRIC_CAN, &TxMessage);
}


void send_cap_power_can(uint16_t power)        //发送超电
{
	  CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CAP_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = power >> 8;
    TxMessage.Data[1] = power;
		 CAN_Transmit(CAN1, &TxMessage);
}

//can1中断
void CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        STD_CAN_RxCpltCallback(CAN1,&rx1_message);
    }
}

//can2中断
void CAN2_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        STD_CAN_RxCpltCallback(CAN2,&rx2_message);
    }
}


