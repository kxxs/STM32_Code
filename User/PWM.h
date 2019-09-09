#include "stm32f10x_it.h"

void left_Pid(u32 input,u32 target);
void right_Pid(u32 input,u32 target);
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void setDir(int left,int right,short stop);
void setPWM_L(u32 left);
void setPWM_R(u32 right);
void TIM2_Detect_init(u16 arr,u16 psc);
void TIM4_Detect_init(u16 arr,u16 psc);
void TIM5_Detect_init(u16 arr,u16 psc);
void TIM6_Config(void);



