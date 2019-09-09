#include "PWM.h"
#include "USART.h"
#include "MPU6050.h"
#include "sensors.h"
#include "port.h"
int right_freq;
int left_freq;
int right_target = 500;
int left_target = 500;

int detect_up = 0;
int left_dir;

const float dt = 1;
const float kp = 0.5;
const float ki = 0.5;
const float kd = 0.1;
const int dlimit = 300;

short stop;
extern short mode;
extern short wifi_ok;

int left_etSum;
int left_dEt;
int left_lastErr;
int left_secondErr;
int left_et,left_et1,left_et2;
int left_output;
int	left_dout;

int right_etSum;
int right_dEt;
int right_lastErr;
int right_secondErr;
int right_et,right_et1,right_et2;
int right_output;
int	right_dout;
		
int left_stop_flag;
int right_stop_flag;
int counter_overflow = 0;

void left_Pid(u32 input,u32 target){  

    left_et = target - input; 
		left_et1 = left_et - left_lastErr;
		left_et2 = left_et - 2*left_lastErr + left_secondErr;
    left_dout = kp * left_et1 + ki * left_et + kd * left_et2;
	if(left_dout>dlimit) {
		left_dout=dlimit;
	}
	else if(left_dout<-dlimit) {
		left_dout=-dlimit;
	}
	left_output += left_dout;
	if(left_output<50){
			left_output=50;
	}
	else if(left_output>999) {
		left_output=999;

	}

	left_secondErr = left_lastErr;
    left_lastErr = left_et;  
} 

void right_Pid(u32 input,u32 target){  

    right_et = target - input; 
	right_et1 = right_et - right_lastErr;
	right_et2 = right_et - 2*right_lastErr + right_secondErr;
    right_dout = kp * right_et1 + ki * right_et + kd * right_et2;
	if(right_dout>dlimit) right_dout=dlimit;
	else if(right_dout<-dlimit) right_dout=-dlimit;
	right_output += right_dout;
	if(right_output<50) right_output=50;
	else if(right_output>999) right_output=999;

	right_secondErr = right_lastErr;
    right_lastErr = right_et; 

	
} 

void TIM3_IRQHandler(void){
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  
		}		
}



void TIM3_PWM_Init(u16 arr,u16 psc){  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitTypeDef GPIO_InitStructure1;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7; 
	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure1);
 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  
	TIM_OC2Init(TIM3, &TIM_OCInitStructure); 
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  
 
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
	
}


void setDir(int left,int right,short stop){
	
	if(stop){
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);
		GPIO_ResetBits(GPIOC,GPIO_Pin_2);
	}
	else if(left>0){
		GPIO_SetBits(GPIOC,GPIO_Pin_2);
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);
	}
	else{
		GPIO_SetBits(GPIOC,GPIO_Pin_3);
		GPIO_ResetBits(GPIOC,GPIO_Pin_2);
	}
	
	if(stop){
		GPIO_ResetBits(GPIOC,GPIO_Pin_1);
		GPIO_ResetBits(GPIOC,GPIO_Pin_0);
	}
	else if(right>0){
		GPIO_SetBits(GPIOC,GPIO_Pin_0);
		GPIO_ResetBits(GPIOC,GPIO_Pin_1);
	}
	else{
		GPIO_SetBits(GPIOC,GPIO_Pin_1);
		GPIO_ResetBits(GPIOC,GPIO_Pin_0);
	}
}

void setPWM_L(u32 left){
	left_target = left;
}

void setPWM_R(u32 right){
	right_target = right;
}

void TIM2_Detect_init(u16 arr,u16 psc){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
   
  TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler = psc ; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
   
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
  TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;
	
  TIM_ICInitStructure.TIM_ICFilter = 0x0;  
  TIM_PWMIConfig(TIM2, &TIM_ICInitStructure); 
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2); 
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset); 
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
  TIM_Cmd(TIM2, ENABLE); 
  TIM_ITConfig(TIM2, TIM_IT_Update|TIM_IT_CC2, ENABLE); 
  TIM_ClearFlag(TIM2,TIM_IT_CC2);
}

void TIM2_IRQHandler(void) { 	
	u32 IC2Value_right;	
	
	if(TIM_GetITStatus(TIM2,TIM_IT_CC2)==SET){

			if(0&&right_stop_flag == 1){
				right_freq = 0;
				right_stop_flag = 0;
			}
			else{
				IC2Value_right = TIM_GetCapture2(TIM2); 
				if(IC2Value_right > 800) 
				{ 
						right_freq = 72000000 /(24 * IC2Value_right); 
					  right_stop_flag = 0;
				} 
				else right_freq = 0;
		  }			
	}else{
		counter_overflow++;	
		right_freq = 0;
		right_stop_flag = 1;
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update|TIM_IT_CC2); 
	TIM_SetCounter(TIM2,0);
} 


void TIM4_Detect_init(u16 arr,u16 psc){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
   
  TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler = psc ; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 

  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
   
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
  TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;
	
  TIM_ICInitStructure.TIM_ICFilter = 0x0;  
  TIM_PWMIConfig(TIM4, &TIM_ICInitStructure); 
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2); 
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset); 
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
  TIM_Cmd(TIM4, ENABLE); 
  TIM_ITConfig(TIM4, TIM_IT_Update|TIM_IT_CC2, ENABLE); 
  TIM_ClearFlag(TIM4,TIM_IT_CC2);
}

void TIM4_IRQHandler(void) { 
	u32 IC2Value;	

	if(TIM_GetITStatus(TIM4,TIM_IT_CC2)==SET){
			if(0&&left_stop_flag == 1){
					left_freq = 0;
				left_stop_flag = 0;
			}
			else{
				IC2Value = TIM_GetCapture2(TIM4); 
				left_dir=detect_up;
				if(IC2Value > 800) 
				{ 
						left_freq = 72000000 /(24 * IC2Value); 
				} 
				else left_freq = 0;		
			}
	}
	else{
		left_freq = 0;
		left_stop_flag = 1;

	}
	
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update|TIM_IT_CC2); 
	TIM_SetCounter(TIM4,0);
} 



void TIM6_Config(void){
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
NVIC_InitTypeDef NVIC_InitStructure;

RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 

TIM_TimeBaseStructure.TIM_Period = 999;
TIM_TimeBaseStructure.TIM_Prescaler = 719;
TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; 
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
TIM_ITConfig(TIM6,TIM_IT_Update, ENABLE);
TIM_Cmd(TIM6, ENABLE);
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
}

void TIM6_IRQHandler(void){
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update); 
		TIM_Cmd(TIM6, DISABLE);
		
	  //正式
		//setDir(left_target,right_target,stop||(wifi_ok==0||mode!=0x01));
		//调试
		//setDir(left_target,right_target,stop);
		setDir(left_target,right_target,stop||(!wifi_ok||mode!=0x01));
		
		left_Pid(left_freq,abs(left_target));
		right_Pid(right_freq,abs(right_target));
		
		TIM_SetCompare1(TIM3,left_output);	
		TIM_SetCompare2(TIM3,right_output);	
		
		GetBlack();
		
		TIM6->CNT = 0;
	  TIM_Cmd(TIM6, ENABLE);
		
	}
}

void TIM5_Detect_init(u16 arr,u16 psc){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
   
  TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler = psc ; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 

  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
   
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
	detect_up = 0;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
  TIM_ICInitStructure.TIM_ICPrescaler= TIM_ICPSC_DIV1;
	
  TIM_ICInitStructure.TIM_ICFilter = 0x0;  
  TIM_PWMIConfig(TIM5, &TIM_ICInitStructure); 
  TIM_SelectInputTrigger(TIM5, TIM_TS_TI1FP1); 
  TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset); 
	TIM_SelectMasterSlaveMode(TIM5, TIM_MasterSlaveMode_Enable);
  TIM_Cmd(TIM5, ENABLE); 
  TIM_ITConfig(TIM5, TIM_IT_Update|TIM_IT_CC1, ENABLE); 
  TIM_ClearFlag(TIM5,TIM_IT_CC1);
}

void TIM5_IRQHandler(void) { 	
	if(TIM_GetITStatus(TIM5,TIM_IT_CC1)==SET){
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC1); 
		if(detect_up){
			TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising);
			detect_up = 0;
		}
		else {
			TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling);
			detect_up = 1;
		}
	}	
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); 	
} 