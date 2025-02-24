#include "tim.h"

void TIM8_DEVICE(int per,int psc)//BALL
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
  
  GPIO_PinAFConfig(GPIOI,GPIO_PinSource7,GPIO_AF_TIM8);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOI,&GPIO_InitStructure);
  
  TIM_TimeBaseInitStructure.TIM_Period = per;
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure);
  
	TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 2000;
  TIM_OC1Init(TIM8,&TIM_OCInitStructure);
	TIM_OC2Init(TIM8,&TIM_OCInitStructure);
	TIM_OC3Init(TIM8,&TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM8,TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM8,TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM8,ENABLE);
	TIM_CtrlPWMOutputs(TIM8,ENABLE);//高级定时器TIM1 TIM8必须使能这一句，否则无法输出PWM
  TIM_Cmd(TIM8,ENABLE);
}

void TIM1_DEVICE(int per,int psc)/*SHOOT*/
{
  GPIO_InitTypeDef GPIO_InitStructure1;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure1;
  TIM_OCInitTypeDef TIM_OCInitStructure1;
  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);
  
	GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure1.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure1.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOE,&GPIO_InitStructure1);

  TIM_TimeBaseInitStructure1.TIM_Period = per;
  TIM_TimeBaseInitStructure1.TIM_Prescaler = psc;
  TIM_TimeBaseInitStructure1.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure1.TIM_RepetitionCounter=0;
  TIM_TimeBaseInitStructure1.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure1);
	
	TIM_ARRPreloadConfig(TIM1,ENABLE);
  
	TIM_OCStructInit(&TIM_OCInitStructure1);//必须要加，否则会出现会出现奇怪的问题
  TIM_OCInitStructure1.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure1.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure1.TIM_OutputState=TIM_OutputState_Enable;
  TIM_OCInitStructure1.TIM_Pulse = 1000;
	TIM_OCInitStructure1.TIM_OCIdleState = TIM_OCIdleState_Reset;
	
  TIM_OC1Init(TIM1,&TIM_OCInitStructure1);
  TIM_OC2Init(TIM1,&TIM_OCInitStructure1);
	TIM_OC3Init(TIM1,&TIM_OCInitStructure1);
  TIM_OC4Init(TIM1,&TIM_OCInitStructure1);

  TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
  
  TIM_Cmd(TIM1,ENABLE);
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
}

//void TIM12_DEVICE(int per,int psc)/*DETECT*/
//{
//  GPIO_InitTypeDef GPIO_InitStructure1;
//  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure1;
//  TIM_OCInitTypeDef TIM_OCInitStructure1;
//  
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);
//	
//  GPIO_PinAFConfig(GPIOH,GPIO_PinSource6,GPIO_AF_TIM12);

//	GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_6;
//	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure1.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure1.GPIO_Speed = GPIO_High_Speed;
//	GPIO_Init(GPIOH,&GPIO_InitStructure1);

//  TIM_TimeBaseInitStructure1.TIM_Period = per;
//  TIM_TimeBaseInitStructure1.TIM_Prescaler = psc;
//  TIM_TimeBaseInitStructure1.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseInitStructure1.TIM_RepetitionCounter=0;
//  TIM_TimeBaseInitStructure1.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseInit(TIM12,&TIM_TimeBaseInitStructure1);
//	
//	TIM_ARRPreloadConfig(TIM12,ENABLE);
//  
//	TIM_OCStructInit(&TIM_OCInitStructure1);//必须要加，否则会出现会出现奇怪的问题
//  TIM_OCInitStructure1.TIM_OCMode = TIM_OCMode_PWM1;
//  TIM_OCInitStructure1.TIM_OCPolarity = TIM_OCPolarity_High;
//  TIM_OCInitStructure1.TIM_OutputState=TIM_OutputState_Enable;
//  TIM_OCInitStructure1.TIM_Pulse = 1000;
//	TIM_OCInitStructure1.TIM_OCIdleState = TIM_OCIdleState_Reset;
//  TIM_OC1Init(TIM12,&TIM_OCInitStructure1);

//  TIM_OC1PreloadConfig(TIM12,TIM_OCPreload_Enable);
//  
//  TIM_Cmd(TIM12,ENABLE);
//	TIM_CtrlPWMOutputs(TIM12,ENABLE);
//}

void TIM10_DEVICE(int per,int psc)/*GPYO*/
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);
  
  GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_TIM10);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
  
  TIM_TimeBaseInitStructure.TIM_Period = per;
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM10,&TIM_TimeBaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  
  TIM_OC1Init(TIM10,&TIM_OCInitStructure);

  
  TIM_OC1PreloadConfig(TIM10,TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM10,ENABLE);
  
  TIM_Cmd(TIM10,ENABLE);
}
