#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "stm32f4xx.h"
#include "arm_math.h"

#define mat         arm_matrix_instance_f32 
#define mat_64      arm_matrix_instance_f64
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

#define IIR_FILTER

typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;

#include "FreeRTOS.h"
/**********系统时间对外数据接口************/
typedef struct
{
  uint32_t WorldTime;      // 世界时间
  uint32_t Last_WorldTime; // 上一次世界时间
} WorldTime_RxTypedef;

typedef struct
{
  double raw_value;
  double xbuf[7];
  double ybuf[7];
  double filtered_value;
} iir_filter_t;

void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS);
static uint32_t FPS_Calculate(uint16_t deltaTime);

extern iir_filter_t yaw_msg_t, pit_msg_t;

void   kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float angle, float speed);
void kalman_filter_init_two(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc_two(kalman_filter_t *F, float Pitch, float Pitch_speed, float Yaw, float Yaw_speed);
double iir_filter(iir_filter_t *F);

#endif
