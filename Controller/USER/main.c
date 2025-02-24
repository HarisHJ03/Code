/**
 ******************************************************************************
 * 函数库  ： 基于STM32F4标准库
 * 芯片型号： STM32F407IGHx
 * 代码版本： 第一代框架
 * 完成日期： 2022.6.19
 ******************************************************************************
 *                          RM . 电控之歌
 *
 *                  一年备赛两茫茫，写程序，到天亮。
 *                      万行代码，Bug何处藏。
 *                  机械每天新想法，天天改，日日忙。
 *
 *                  视觉调试又怎样，朝令改，夕断肠。
 *                      相顾无言，惟有泪千行。
 *                  每晚灯火阑珊处，夜难寐，继续肝。
 ******************************************************************************
 **/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
/*freertos*/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
/*config*/
#include "delay.h"
#include "gpio.h"
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "STM32_TIM_BASE.h"
/*bsp*/
#include "bsp_flash.h"
#include "judge_rx_data.h"
#include "judge_tx_data.h"
#include "pc_rx_data.h"
#include "pc_tx_data.h"
/*task*/
#include "start_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "imu_task.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "stdlib.h"
#include "kalman_filter.h"
#include "BMI088driver.h"
#include "bsp_dwt.h"
#include "bsp_usb.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_core.h"
#include "iwdg.h"

void flash_cali(void);
void Config_SystemClock(uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ);

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

int main(void)
{
	// Config_SystemClock(6,168, 2,7);
	SysTick_Init(168);								// 系统滴答定时器初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // 中断分组设置

#ifdef USE_USB_OTG_FS
	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
#endif

	TIM_BASE_Init(10 - 1, 8400 - 1); // 配置定时器4为1ms中断一次，做来系统詗诵屑剖�，便于之后实现一些延时操作
	GPIO_INIT();					 // 板载LED、归中按键、24V电源输出、板载陀螺仪SPI等引脚的初始化，具体看A板的原理图
	// TIM8_DEVICE(20000 - 1, 168 - 1); // 舵机PWM周期需要配置成20ms，舵机0-180°对应为高电平持续时间0.5ms-2.5ms
	// TIM1_DEVICE(2500 - 1, 168 - 1);	 // 摩擦轮
	//	TIM12_DEVICE(400-1,90-1);//蜂鸣器，PWM频率在2700HZ时声音最大，且占空比需要设置成5％的高电平，此处设置为2500HZ
	TIM10_DEVICE(5000 - 1, 0);												 // 提供一路PWM使得加热电阻升温，用于恒温加热IMU
	// CAN1_DEVICE(CAN_Mode_Normal, CAN_SJW_1tq, CAN_BS1_10tq, CAN_BS2_3tq, 3); // CAN1配置
	// CAN2_DEVICE(CAN_Mode_Normal, CAN_SJW_1tq, CAN_BS1_10tq, CAN_BS2_3tq, 3);
	// USART3_DEVICE(); // 用于遥控通信，采用的是DMA加空闲中断的方式接收数据
	USART1_DEVICE(); // 用于PC通信数据，采用的是DMA加空闲中断的方式接收数据
	USART6_DEVICE(); // 用于裁判系统通信，采用的是DMA加空闲中断的方式接收数据

	//	USART8_DEVICE();
	SPI_DEVICE(); // 用于读取板载陀螺仪
	// 云台，底盘，发射三个模块的一些初始化
	// gimbal_param_init();
	// chassis_param_init();
	// shoot_param_init();
	// shoot2_param_init();
	// 离线检测任务的初始化
	// detect_param_init();
	// 裁判系统和小电脑通信任务的初始化
	judgement_rx_param_init();
	judgement_tx_param_init();
	pc_rx_param_init();
	pc_tx_param_init();
	// Kalman
	// Kalman_Init();
	DWT_Init(168);
	/*算法补偿角初始化*/
	//  pc_send_mesg.pc_need_information.pit_set = -0.6f;
	//	pc_send_mesg.pc_need_information.yaw_set = -0.5f;
	/*从板载FLASH读出云台归中位置数据*/
	flash_cali();

	/*如果是因为看门狗复位则直接跳过陀螺仪校准*/
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
	{
	}
	else
	{
		while (BMI088_init())
		{
		};
	}

	GPIO_ResetBits(GPIOH, GPIO_Pin_12);
	GPIO_SetBits(GPIOH, GPIO_Pin_11);
	IWDG_Config(IWDG_Prescaler_64, 1250); // 1s不喂狗自动复位,在modeswitch里边喂
	RCC_ClearFlag();
	TASK_START();		   // 创建各个任务，设定优先级和堆栈大小
	vTaskStartScheduler(); // 开启任务调度
	while (1)
	{
	}
}

void Config_SystemClock(uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ)
{
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	RCC_HSICmd(DISABLE);
	if (RCC_WaitForHSEStartUp() == SUCCESS)
	{
		RCC_ClockSecuritySystemCmd(ENABLE);

		RCC_PLLConfig(RCC_PLLSource_HSE, PLLM, PLLN, PLLP, PLLQ);
		RCC_PLLCmd(ENABLE);

		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div4);
		RCC_PCLK2Config(RCC_HCLK_Div2);

		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
			;
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	}
}

void flash_cali(void)
{
	BSP_FLASH_READ();
	if (cali_param.cali_state != CALI_DONE)
	{
		for (;;)
			;
	}
	else
	{
		gimbal.pit_center_offset = cali_param.pitch_offset;
		gimbal.yaw_center_offset = cali_param.yaw_offset;

		// 手动记录0值
		// gimbal.pit_center_offset =
		// gimbal.yaw_center_offset =
	}
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
