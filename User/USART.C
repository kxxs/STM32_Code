#include "stm32f10x.h"
#include "port.h"
#include "USART.h"
#define MPU6050_DMA_Len 100
#define WIFI_DMA_Len 300

extern int right_freq;
extern int left_freq;
extern int left_target;
extern float Angle;
extern short black1,black2,black3,black4;
short dataPrepare;
short AnglePrepare;
short connected = 0;
char dataPackage[64];
char AnglePackage[10];
char a[8];

float FirstAngle;
float angle_base=0;
short AngleInit = 0;

char MPU6050_DMA_Buf[100];
char WIFI_DMA_Buf[300];

int Usart3_Rec_Cnt;
int Usart1_Rec_Cnt;

const char a0[] = "+IPD,64:";
int count=0;
int count2=0;
int count3=0;
short dataFlag;
short mode;
short rounds;
short AX=-270;
short AY=-270;
short BX=-270;
short BY=-270;
short men_num;
short men_state[5];
short men_SX[5];
short men_SY[5];
short men_EX[5];
short men_EY[5];
short penalty_A;
short penalty_B;
short score_A;
short score_B;
short wifi_ok=0;

short comp(const char *a,const char*b,int len){
	for(;len>=0;len--){
		if(a[len]!=b[len]){
			return 0;
		}
	}
	return 1;
}
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx,int DMA_Rec_Len){ 
	DMA_Cmd(DMA_CHx, DISABLE );  //�ر�USART1 TX DMA1 ��ָʾ��ͨ��      
 	DMA_SetCurrDataCounter(DMA_CHx,DMA_Rec_Len);//DMAͨ����DMA����Ĵ�С
 	DMA_Cmd(DMA_CHx, ENABLE);  //ʹ��USART1 TX DMA1 ��ָʾ��ͨ�� 
}	
void dataProcess(){
	//print("12312312");
	mode=dataPackage[0]>>6;
	rounds = ((dataPackage[0]&0x3f)<<8)|dataPackage[1];
	AX = ((dataPackage[2]&0x80)<<1)|dataPackage[5];
	AY = ((dataPackage[2]&0x40)<<2)|dataPackage[6];
	BX = ((dataPackage[2]&0x20)<<3)|dataPackage[7];
	BY = ((dataPackage[2]&0x10)<<4)|dataPackage[8];
	men_num = dataPackage[9]>>2;
	men_state[0] = dataPackage[9]&0x03;
	men_state[1] = (dataPackage[10]>>6)&0x03;
	men_state[2] = (dataPackage[10]>>4)&0x03;
	men_state[3] = (dataPackage[10]>>2)&0x03;
	men_state[4] = dataPackage[10]&0x03;
	men_SX[0] = ((dataPackage[2]&0x08)<<5)|dataPackage[11];
	men_SX[1] = ((dataPackage[3]&0x80)<<1)|dataPackage[15];
	men_SX[2] = ((dataPackage[3]&0x08)<<5)|dataPackage[19];
	men_SX[3] = ((dataPackage[4]&0x80)<<1)|dataPackage[23];
	men_SX[4] = ((dataPackage[4]&0x08)<<5)|dataPackage[27];
	
	men_SY[0] = ((dataPackage[2]&0x04)<<6)|dataPackage[12];
	men_SY[1] = ((dataPackage[3]&0x40)<<2)|dataPackage[16];
	men_SY[2] = ((dataPackage[3]&0x04)<<6)|dataPackage[20];
	men_SY[3] = ((dataPackage[4]&0x40)<<2)|dataPackage[24];
	men_SY[4] = ((dataPackage[4]&0x04)<<6)|dataPackage[28];
	
	men_EX[0] = ((dataPackage[2]&0x02)<<7)|dataPackage[13];
	men_EX[1] = ((dataPackage[3]&0x20)<<3)|dataPackage[17];
	men_EX[2] = ((dataPackage[3]&0x02)<<7)|dataPackage[21];
	men_EX[3] = ((dataPackage[4]&0x20)<<3)|dataPackage[25];
	men_EX[4] = ((dataPackage[4]&0x02)<<7)|dataPackage[29];
	
	men_EY[0] = ((dataPackage[2]&0x01)<<8)|dataPackage[14];
	men_EY[1] = ((dataPackage[3]&0x10)<<4)|dataPackage[18];
	men_EY[2] = ((dataPackage[3]&0x01)<<8)|dataPackage[22];
	men_EY[3] = ((dataPackage[4]&0x10)<<4)|dataPackage[26];
	men_EY[4] = ((dataPackage[4]&0x01)<<8)|dataPackage[30];
	penalty_A = dataPackage[31];
	penalty_B = dataPackage[32];
	score_A = (dataPackage[33]<<8)|dataPackage[35];
	score_B = (dataPackage[34]<<8)|dataPackage[36];
	dataFlag = 1;
}

void USART1_init(void){
	//GPIO
	
	GPIO_InitTypeDef GPIO_initStructure;
	USART_InitTypeDef USART_initStructure;
	NVIC_InitTypeDef NVIC_initStructure;
	DMA_InitTypeDef DMA_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	GPIO_initStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_initStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_initStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_initStructure);
	GPIO_initStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_initStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_initStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_initStructure);
	
	USART_DeInit(USART1);
	//USART
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  USART_initStructure.USART_BaudRate=115200;
	USART_initStructure.USART_WordLength=USART_WordLength_8b;
	USART_initStructure.USART_StopBits=USART_StopBits_1;
	USART_initStructure.USART_Parity=USART_Parity_No;
	USART_initStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_initStructure.USART_Mode= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init( USART1,& USART_initStructure);
	
	//NVIC
	NVIC_initStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_initStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_initStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_initStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(& NVIC_initStructure);

	USART_Init(USART1, &USART_initStructure); //初始化串口
  USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启空闲中断
  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);   //使能串口3 DMA接收
	USART_Cmd(USART1,ENABLE);
	
	  DMA_DeInit(DMA1_Channel5);   //将DMA的通道5寄存器重设为缺省值  串口1对应的是DMA通道5
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;  //DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)WIFI_DMA_Buf;  //DMA内存基地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设读取发送到内存
  DMA_InitStructure.DMA_BufferSize = WIFI_DMA_Len;  //DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道

  DMA_Cmd(DMA1_Channel5, ENABLE);  //正式驱动DMA传输
}

void USART1_IRQHandler(void){
		    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET){  
					int i,j;
					 
          USART_ReceiveData(USART1);//读取数据 注意：这句必须要，否则不能够清除中断标志位。
          Usart1_Rec_Cnt = WIFI_DMA_Len-DMA_GetCurrDataCounter(DMA1_Channel5); //算出接本帧数据长度
		
					 //print("111");
					//print(WIFI_DMA_Buf);
				for(i=0;i<Usart1_Rec_Cnt;i++){
					if(comp(WIFI_DMA_Buf+i,"+IPD,64:",7)){
						wifi_ok=1;
						for(j = 0; j < 64; j++)
							dataPackage[j] = WIFI_DMA_Buf[i+j+8];
						dataProcess();
						break;
					}	
				}
        //*************************************//
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);         //清除中断标志
        MYDMA_Enable(DMA1_Channel5,WIFI_DMA_Len);                   //恢复DMA指针，等待下一次的接收
     } 
}
/*
void USART1_IRQHandler(void){
	u8 byte;
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		byte=USART1_receiveByte();
		//print("this is tim 1 \n");

		if(dataPrepare==0)
		{
			int i,j=7;
			for(i=7;i>0;--i){
				a[i] = a[i-1];
			}
			a[0]=byte;
			
			while(j>=0){
				if(a[7-j]==a0[j]){	
					--j;
				}
				else {
					dataPrepare=0;
					break;
				}
				
				if(j<0){
					dataPrepare=1;
				}
			}
		}
		else
		{
			dataPackage[count]=byte;
			count++;
			if(count>=64)
			{
				if(count2++>3){
				dataProcess();
				count2 = 0;
				}
				
				count=0;
				dataPrepare=0;
			}
		}
		
		
		//USART_SendData(USART2,byte);
		//while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)!=SET);
	}		
	
}
*/

void USART1_sendByte(u8 byte){
	USART_SendData(USART1,byte);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)!=SET);
}

void USART1_sendStr(char* str){
	while(*str!=0)
	{
		USART1_sendByte(*str);
		str++;
	}
}

u8 USART1_receiveByte(void){
	return USART_ReceiveData(USART1);
}


void USART2_init(void){
	GPIO_InitTypeDef GPIO_initStructure;
	USART_InitTypeDef USART_initStructure;

	//GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	GPIO_initStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_initStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_initStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_initStructure);
	GPIO_initStructure.GPIO_Pin=GPIO_Pin_3;
	GPIO_initStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_initStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_initStructure);
	
	//USART
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	USART_initStructure.USART_BaudRate=115200;
	USART_initStructure.USART_WordLength=USART_WordLength_8b;
	USART_initStructure.USART_StopBits=USART_StopBits_1;
	USART_initStructure.USART_Parity=USART_Parity_No;
	USART_initStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_initStructure.USART_Mode= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2,& USART_initStructure);
	USART_Cmd(USART2,ENABLE);
	
	//NVIC
//	NVIC_initStructure.NVIC_IRQChannel=USART2_IRQn;
//	NVIC_initStructure.NVIC_IRQChannelPreemptionPriority=1;
//	NVIC_initStructure.NVIC_IRQChannelSubPriority=0;
//	NVIC_initStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_initStructure);
	
}

void USART2_sendByte(u8 byte){
	USART_ClearFlag(USART2,USART_FLAG_TC);
	USART_SendData(USART2,byte);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);
}

void USART2_sendStr(char* str){
	while(*str!=0)
	{
		USART2_sendByte(*str);
		str++;
	}
}

u8 USART2_receiveByte(void){
	return USART_ReceiveData(USART2);
}
void USART3_init(void){
	//GPIO
	
	GPIO_InitTypeDef GPIO_initStructure;
	USART_InitTypeDef USART_initStructure;
	NVIC_InitTypeDef NVIC_initStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	GPIO_initStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_initStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_initStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_initStructure);
	GPIO_initStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_initStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_initStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_initStructure);
	
	USART_DeInit(USART3);
	//USART
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
  USART_initStructure.USART_BaudRate=115200;
	USART_initStructure.USART_WordLength=USART_WordLength_8b;
	USART_initStructure.USART_StopBits=USART_StopBits_1;
	USART_initStructure.USART_Parity=USART_Parity_No;
	USART_initStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_initStructure.USART_Mode= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init( USART3,&USART_initStructure);
	
	//NVIC
	NVIC_initStructure.NVIC_IRQChannel=USART3_IRQn;
	NVIC_initStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_initStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_initStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(& NVIC_initStructure);

	USART_Init(USART3, &USART_initStructure); //初始化串口
  USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//开启空闲中断
  USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);   //使能串口3 DMA接收
  USART_Cmd(USART3, ENABLE);                    //使能串口 
	
	   //相应的DMA配置
  DMA_DeInit(DMA1_Channel3);   //将DMA的通道5寄存器重设为缺省值  串口1对应的是DMA通道5
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;  //DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)MPU6050_DMA_Buf;  //DMA内存基地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设读取发送到内存
  DMA_InitStructure.DMA_BufferSize = MPU6050_DMA_Len;  //DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道

  DMA_Cmd(DMA1_Channel3, ENABLE);  //正式驱动DMA传输
}

u8 USART3_receiveByte(void){
	return USART_ReceiveData(USART3);
}
void USART3_IRQHandler(void){
				float last_angle=-Angle,t;
	     if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET){  
					int i;
          USART_ReceiveData(USART3);//读取数据 注意：这句必须要，否则不能够清除中断标志位。
          Usart3_Rec_Cnt = MPU6050_DMA_Len-DMA_GetCurrDataCounter(DMA1_Channel3); //算出接本帧数据长度
   
				for(i=0;i<Usart3_Rec_Cnt;i++){
					if(MPU6050_DMA_Buf[i]==0x55&&MPU6050_DMA_Buf[i+1]==0x53){
						/*if(!AngleInit){
						FirstAngle = ((short)MPU6050_DMA_Buf[i+7]<<8| MPU6050_DMA_Buf[i+6])*180/32768;
							AngleInit = 1;
						}
						else{*/
						t = angle_base+((short)MPU6050_DMA_Buf[i+7]<<8| MPU6050_DMA_Buf[i+6])*180/32768; //- FirstAngle;   //Z轴偏航角（z 轴）
						if(t-last_angle>300){
							t-=360;
							angle_base-=360;
						}else if(last_angle-t>300){
							t+=360;
							angle_base+=360;
						}
						Angle=-t;
						//}
						break;
					}
				}
        //*************************************//
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);         //清除中断标志
        MYDMA_Enable(DMA1_Channel3,MPU6050_DMA_Len);                   //恢复DMA指针，等待下一次的接收
     } 
	
}

void itoa(int number,char* buff){
	short length=0;
	short i=0;
	short j,k;
	char tmp;
	short c;
	short flag;
	flag=(number<0);
	number=abs(number);
	for(c=0;c<10;c++){
		buff[c]=0;
	}
	
	if(number==0){
		buff[0]='0';
		buff[1]=0;
	}
	else 
	{
		while(number!=0)
		{
			buff[i]=number%10+'0';
			number/=10;
			i++;
			length++;
		}
		if(flag){
			buff[i]='-';
			length++;
		}
		for(j=0,k=length-1;j<k;j++,k--)
		{
			tmp=buff[j];
			buff[j]=buff[k];
			buff[k]=tmp;
		}
		buff[length]=0;
	}
}

void printn(int n){
	char buffer[10];
	itoa(n,buffer);
	print(buffer);
}

void freq_send(){

//printn((int)Angle);
//	printn(black1);
//	printn(black2);
//	printn(black3);
//	printn(black4);
//	print("\r\n");
//	print(" Angle:");
//	printn(Angle);
//	
//	
//	print(" left_freq:");
//	printn(left_freq);
//	print(" right_freq:");
//	printn(right_freq);
//	print("\r\n");
	
}


void wireless_init(){
/*
AT+CWMODE=3
AT+RST
AT+CWJAP="EDC20","12345678"
AT+CIPSTART="TCP","192.168.1.116",20000
*/
	
	/*
	USART1_sendStr("AT\r\n");
	Delay(100000);
	USART1_sendStr("AT+CWMODE=3\r\n");
	print("----------------CMODE\n");
	Delay(1000000);
	USART1_sendStr("AT+RST\r\n");
	//while(!(USART1_receiveByte()=='I'&&USART1_receiveByte()=='P'));
	print("----------------RST\n");
	Delay(6000000);
	print("----------------RST\n");
	*/
	USART1_sendStr("AT+CWJAP=\"EDC20\",\"12345678\"\r\n");
	//while(USART1_receiveByte()!='K');
	Delay(4000000);
	USART1_sendStr("AT+CIPSTART=\"TCP\",\"192.168.1.124\",20000\r\n");
	//while(USART1_receiveByte()!='K');
	Delay(4000000);
	USART1_sendStr("AT+CIPSTART=\"TCP\",\"192.168.1.124\",20000\r\n");
	Delay(1000000);
}

