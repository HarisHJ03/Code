#include "imu_task.h"


#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

UBaseType_t imu_stack_surplus;
extern TaskHandle_t imu_Task_Handle;

volatile uint8_t imu_start_flag = 0;

extern IMU_Data_t BMI088;

int32_t send_data[6] = {0};

float EulerAngle[3] = {0};

float totalangle_transfer(float angle);

float imu_pid[3] = {200, 0.1, 0};

fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

// extern ist8310_data_t IST8310;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

// static uint8_t first_temperate;
// static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
// static pid_type_def imu_temp_pid;

// static const float timing_time = 0.001f; // tast run time , unit s.任务运行的时间 单位 s

static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static fp32 gyro_filter[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};		  // euler angle, unit rad.??? ?? rad
fp32 INS_angle_final[3] = {0.0f, 0.0f, 0.0f}; // 转换单位后的角度

void usart_send_32bit(USART_TypeDef *USARTx, uint32_t pData[], uint16_t Size);
void send_int32(int32_t data[], uint8_t Size);
void send_byte(uint8_t data);
void imu_temp_keep(void)
{
	PID_Struct_Init(&pid_imu_tmp, imu_pid[0], imu_pid[1], imu_pid[2], 5000, 1000, INIT);
	if (BMI088.Temperature > 45)
	{
		pid_calc(&pid_imu_tmp, BMI088.Temperature, 45);
		TIM_SetCompare1(TIM10, pid_imu_tmp.out);
	}
	else
	{
		TIM_SetCompare1(TIM10, 5000 - 1);
	}
}

/**
 * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
 * @param[out]     gyro: 加上零漂和旋转
 * @param[out]     accel: 加上零漂和旋转
 * @param[in]      bmi088: 陀螺仪和加速度计数据
 * @retval         none
 */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], IMU_Data_t *bmi088)
{
	for (uint8_t i = 0; i < 3; i++)
	{
		gyro[i] = (bmi088->Gyro[0] + gyro_offset[0]) * gyro_scale_factor[i][0] + (bmi088->Gyro[1] + gyro_offset[1]) * gyro_scale_factor[i][1] + (bmi088->Gyro[2] + gyro_offset[2]) * gyro_scale_factor[i][2];
		accel[i] = bmi088->Accel[0] * accel_scale_factor[i][0] + bmi088->Accel[1] * accel_scale_factor[i][1] + bmi088->Accel[2] * accel_scale_factor[i][2];
		//		mag[i] = ist8310->Mag[0] * mag_scale_factor[i][0] + ist8310->Mag[1] * mag_scale_factor[i][1] + ist8310->Mag[2] * mag_scale_factor[i][2] + mag_offset[i];
	}
}

void imu_task(void const *argu)
{
	float dt;
	uint32_t time_last = 0;
	uint32_t imu_wake_time = osKernelSysTick();
	imu_Task_Handle = xTaskGetHandle(pcTaskGetName(NULL));
	imu_start_flag = 1;

	AHRS_init(INS_quat, INS_accel, INS_mag);

	accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
	accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
	accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];

	while (1)
	{
		BMI088_read(BMI088.Gyro, BMI088.Accel, &BMI088.Temperature);

		/* rotate and zero drift */
		imu_cali_slove(INS_gyro, INS_accel, INS_mag, &BMI088);

		// 加速度计低通滤波--3阶低通滤波
		// accel low-pass filter
		accel_fliter_1[0] = accel_fliter_2[0];
		accel_fliter_2[0] = accel_fliter_3[0];

		accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

		accel_fliter_1[1] = accel_fliter_2[1];
		accel_fliter_2[1] = accel_fliter_3[1];

		accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

		accel_fliter_1[2] = accel_fliter_2[2];
		accel_fliter_2[2] = accel_fliter_3[2];

		accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

#if (ORDER > 0)
		// 陀螺仪低通滤波
		if (i_filter < ORDER + 1)
		{
			i_filter++;
			Filter0(INS_gyro[0], WINDOWS);
			Filter1(INS_gyro[1], WINDOWS);
			Filter2(INS_gyro[2], WINDOWS);
			gyro_filter[0] = INS_gyro[0];
			gyro_filter[1] = INS_gyro[1];
			gyro_filter[2] = INS_gyro[2];
		}
		else
		{
			gyro_filter[0] = Filter0(INS_gyro[0], WINDOWS);
			gyro_filter[1] = Filter1(INS_gyro[1], WINDOWS);
			gyro_filter[2] = Filter2(INS_gyro[2], WINDOWS);
		}
#else
		gyro_filter[0] = INS_gyro[0]; // 加速度
		gyro_filter[1] = INS_gyro[1];
		gyro_filter[2] = INS_gyro[2];
#endif
		dt = DWT_GetDeltaT(&time_last);
		AHRS_update(INS_quat, dt, gyro_filter, accel_fliter_3, INS_mag);
		get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);

		INS_angle_final[0] = INS_angle[0] * RAD_TO_ANGLE;
		INS_angle_final[1] = INS_angle[1] * RAD_TO_ANGLE;
		INS_angle_final[2] = INS_angle[2] * RAD_TO_ANGLE;

		send_data[0] = INS_angle_final[0]; // YAW
		send_data[1] = INS_angle_final[1]; // PIT
		send_data[2] = INS_angle_final[2]; // ROL

		send_data[3] = BMI088.Accel[0]; // 左右
		send_data[4] = BMI088.Accel[1]; // 前后
		send_data[5] = BMI088.Accel[2]; // 上下


		send_int32(send_data, 6);

		// err_detector_hook(IMU_OFFLINE);

		IWDG_Feed(); // 喂狗
		vTaskDelayUntil(&imu_wake_time, IMU_TASK_PERIOD);
	}
}

void send_byte(uint8_t data)
{
	// 假设有一个函数 USART_SendData 用于发送一个字节
	USART_SendData(USART6, data);
}
uint8_t test_arr[4];
fp32 receive_test[6];
void send_int32(int32_t data[], uint8_t Size)
{
	// 发送高字节到低字节
	while (Size > 0U)
	{
		Size--;
		test_arr[0] = (data[Size] >> 0) & 0xFF;
		test_arr[1] = (data[Size] >> 8) & 0xFF;
		test_arr[2] = (data[Size] >> 16) & 0xFF;
		test_arr[3] = (data[Size] >> 24) & 0xFF;

		receive_test[Size] = (fp32)(test_arr[0] | test_arr[1] << 8 | test_arr[2] << 16 | test_arr[3] << 24);
		for (size_t i = 0; i < 4; i++)
		{
			while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET)
				;
			send_byte(test_arr[i]);
		}
	}
}
