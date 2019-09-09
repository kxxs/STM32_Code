#include"stm32f10x.h"
#include <math.h>
#define print USART2_sendStr

void USART1_init(void);
void USART1_sendByte(u8 byte);
void USART1_sendStr(char* str);
u8 USART1_receiveByte(void);

void USART2_init(void);
void USART2_sendByte(u8 byte);
void USART2_sendStr(char* str);
u8 USART2_receiveByte(void);

void USART3_init(void);
u8 USART3_receiveByte(void);

void itoa(int number,char* buff);
void freq_send(void);
void wireless_init(void);
void dataProcess(void);

void printn(int n);
