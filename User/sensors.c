#include "stm32f10x.h"
#include "port.h"
#include "PWM.h"
#include "sensors.h"
#include "MPU6050.h"
#include "USART.h"
u32 msHcCount;
float distance;
short black1,black2,black3,black4,black5;
extern int right_freq;
extern int left_freq;
extern int right_output;
extern int left_output;
extern int left_target;
extern int right_target;
extern float Angle;

void TIM7_Config(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

  TIM_TimeBaseStructure.TIM_Period = 999;
  TIM_TimeBaseStructure.TIM_Prescaler = 719;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM7, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TIM7_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		
    //Angle_Calculate();
		
  }
}

void sensors_init()
{
  GPIO_InitTypeDef GPIO_initStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_initStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_initStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_initStructure);
	
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_initStructure.GPIO_Pin = GPIO_Pin_10| GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_initStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_initStructure);
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_initStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_initStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_initStructure);
	
}


void GetBlack()
{
  black1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);
  black2 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
	black3 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11);
  black4 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12);
	black5 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10);
	black1|=black3;
	black2|=black5;
}
