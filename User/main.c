#include "main.h"
#include "port.h"
#include "PWM.h"
#include "USART.h"
#include "MPU6050.h"
#include "sensors.h"
#include "math.h"
extern short stop;
extern int counter_overflow;
extern int right_freq;
extern int left_freq;
extern int right_output;
extern int left_output;
extern int left_target;
extern int left_dir;
extern int right_target;
extern float Angle;
extern short black1,black2,black3,black4;
extern short mode;
extern short rounds;
extern short AX;
extern short AY;
extern short BX;
extern short BY;
extern short men_num;
extern short men_state[5];
extern short men_SX[5];
extern short men_SY[5];
extern short men_EX[5];
extern short men_EY[5];
extern short penalty_A;
extern short penalty_B;
extern short score_A;
extern short score_B;
extern short wifi_ok;
int counter_pid = 0;
int counter_locate;
int counter_LR = 0;
int counter_Angle = 0;
int i;
int getman = 0;

int speed=1000;
int *lt=&left_target,*rt=&right_target;
int flag=0;
char pos;
char desti = '#';
char isA;
short car_label;

short *myX,*myY,*myPenalty,*myScore;
short *oppX,*oppY,*oppPenalty,*oppScore;
short going_straight=0;
short turn_around;
int last_angle;

const short debug_mode = 0; //1:debug without wifi 0:wifi_connect then run

#define GPIO_Remap_SWJ_JTAGDisable ((uint32_t)0x00300200)
int dist(int x1,int y1,int x2, int y2){
	return abs(x1-x2) + abs(y1-y2);

}
char dest(){
	float dis_x,dis_y,dis,min_dis = 9999;
	int min_label = 0,dest_dis;
	short i;
	static int dest_label=-1;
	if(dest_label!=-1&&men_state[dest_label]!=0){
		//dest_dis = dist(*myX,*myY,men_SX[dest_label],men_SY[dest_label]);
		getman=0;
	}
	for(i=0;i<men_num;i++){
		if(men_state[i]==car_label){
			getman = 0;
			dest_dis = dist(*myX,*myY,men_EX[i],men_EY[i]);
			dest_label=-1;
			return desti=map[26-men_EY[i]/10][men_EX[i]/10];
		}
	}
	for(i=0;i<men_num;i++){
		if(men_state[i]==0 && men_SX[i]<270 && men_SY[i]<270){
			dis_x = (men_SX[i] - *myX);
			dis_y = (men_SY[i] - *myY);
			dis = abs(dis_x)+ abs(dis_y);
			//尽量不去接C的乘客
			if(map[26-men_SY[i]/10][men_SX[i]/10]=='C' || map[26-men_EY[i]/10][men_EX[i]/10]=='C'){
				dis += 2000;
			}
			if(dis < min_dis){
				min_dis = dis;
				min_label = i;
			}
			
		}
	}

	if(!getman){
		getman = 1;
		dest_label=min_label;
		return desti = map[26-men_SY[min_label]/10][men_SX[min_label]/10];
	}
	else{
		if(min_dis<50&&dest_dis>120){
			return desti = map[26-men_SY[min_label]/10][men_SX[min_label]/10];
		}else{
			return desti;
		}
	}
	return '$';
}
int equ(char a,char* set){
	int i=0;
	char t;
	while((t=set[i++])){
		if(t==a){
			return 1;
		}
	}
	return 0;
}
int eq(char*set){
	return equ(desti,set);
}
char locate(){
	int x=*myX/10,y=*myY/10;
	if(x>26)x=26;
	if(y>26)y=26;
	if(x<0||y<0) return '#';
	return map2[26-y][x];
}

/*char locate_line(){
	int x=*myX/10,y=*myY/10;
	if(x>26)x=26;
	if(y>26)y=26;
	if(x<0||y<0) return '#';
	return (map2[26-y][x]=='l');
}*/

void putchar(char a){
	static char buf[3]={0,'\n',0};
	buf[0]=a;
	print(buf);
}
void wait(int n){
	int i;
for(i = 0; i<n;i++);
}

void straight_old(float targetAngle){
	if(Angle<(targetAngle-3)){
		if(Angle<targetAngle-30){
			right_target=-600;
		}else{
			right_target = 450;
		}
		left_target = 600;
	}
	else if(Angle>(targetAngle+3)){
		if(Angle>targetAngle+30){
			left_target = -600;
		}
		else{
			left_target = 450;
		}
		right_target = 600;
	}
	else{
			left_target = speed;
		right_target = speed;
	}
	
	
}
void straight(float targetAngle){
	if(Angle<(targetAngle-3)){
		if(Angle<targetAngle-30){
			right_target=-speed;
		}else{
			right_target = speed-100;
		}
		left_target = speed;
	}
	else if(Angle>(targetAngle+3)){
		if(Angle>targetAngle+30){
			left_target = -speed;
		}
		else{
			left_target = speed-100;
		}
		right_target = speed;
	}
	else{
			left_target = speed;
		right_target = speed;
	}
	
	
}

void keepStraight(){
			if(going_straight==0){
				last_angle=Angle;
				going_straight=1;
			}
			straight(last_angle);
}
void straight_u(char dest){
	float c=Angle;
	int counter_p=0;
	while(locate()!=dest){
		if(counter_p++%100==0){
			print("\ncurrent position:");
			putchar(locate());
			print("\nlooking for:");
			putchar(dest);
		}
		straight(c);
		wait(1000);
	}
}
void straight_w(char dest){
	float c=Angle;
	int counter_p=0;
	while(locate()==dest){
		if(counter_p++%100==0){
			print("\ncurrent position:");
			putchar(locate());
			print("\nlooking for:");
			putchar(dest);
		}
		straight(c);
		wait(1000);
	}
}

void turnAround(){
	float currentAngle;
//	currentAngle = Angle;
//	left_target=-100;
//	right_target = 2500;
//	while(Angle-currentAngle>-190){// &&!black1)
//		if(left_dir==1){
//			left_target=-100;
//		}
//		else{
//			//left_target=150;
//			left_target=-50;
//			wait(100000);
//		}
//		wait(10000);
//	}
//			left_target = 500;
//	    right_target = 500;
//			wait(2000000);
			currentAngle = Angle;
			left_target = -100;
	    right_target = 1000;
			while(Angle-currentAngle>-180)
				wait(10000);
}
void turn_(int lt,int rt,float ang){
	float cAngle=Angle;
	left_target=lt;
	right_target=rt;
	if(ang>0){
		while(Angle-cAngle<ang){
			wait(10000);
		}
	}else{
		while(Angle-cAngle>ang){
			wait(10000);
		}
	}
	left_target=-lt;
	right_target=-lt;
	wait(1000);
}
void turnLR(int *lt,int *rt,short *black){
	int flag=0,i;
	while(1){		//&&counter_LR++>100		
		if(!*black&&--flag<0){
			going_straight=0;
			*lt = 1600;
	    *rt = 400;
			while(!*black){
				wait(1000);
				if(0&&counter_LR++>500){
				print("\nleft_target:");
				printn(left_target);
				print(" ,left_freq:");
				printn(left_freq);
				print("\nright_target:");
				printn(right_target);
				print(" ,right_freq:");
				printn(right_freq);
							print("\noverflow:");
		printn(counter_overflow);
					counter_LR=0;
				}
				
			}
		}
		else if(*black){
			going_straight=0;
			*lt = -1200;
	    *rt = 1200;
			wait(200000);
			while(*black){
				wait(1000);
				if(0&&counter_LR++>500){
				print("\nleft_target:");
				printn(left_target);
				print(" ,left_freq:");
				printn(left_freq);
				print("\nright_target:");
				printn(right_target);
				print(" ,right_freq:");
				printn(right_freq);
							print("\noverflow:");
		printn(counter_overflow);
					counter_LR=0;
				}
			}
			//wait(200000);
			flag=1;
		}else{
			*lt=*rt=1200;
			for(i=0;i<700&&!*black;++i){
				wait(1000);
				if(0&&counter_LR++>500){
				print("\nleft_target:");
				printn(left_target);
				print(" ,left_freq:");
				printn(left_freq);
				print("\nright_target:");
				printn(right_target);
				print(" ,right_freq:");
				printn(right_freq);
							print("\noverflow:");
		printn(counter_overflow);
					counter_LR=0;
				}
			}
		}
	}
}
void turn(int choice){
	int lt,rt,ang;
	float cAngle;
	if(choice<10){
		switch(choice){
			case 0:
				ang=135;lt=1200;rt=500;break;
			case 1:
				ang=50;lt=1000;rt=-500;break;
			case 2:
				ang=90;lt=1200;rt=500;break;
			case 3:
				ang=120;lt=1200;rt=200;break;
			case 4:
				ang=20;lt=1200;rt=200;break;
		}
		cAngle=Angle;
		left_target=lt;
		right_target=rt;
		while(Angle-cAngle<ang){
			wait(10000);
		}
	}else{
		//左转弯
		switch(choice){
			case 11:
				ang=-50;rt=1000;lt=-500;break;
			case 12:
				ang=-30;rt=1000;lt=-500;break;
		}
		cAngle=Angle;
		left_target=lt;
		right_target=rt;
		while(Angle-cAngle>ang){
			wait(10000);
		}
	}
}
void turnLR_u(int *lt,int *rt,short *black,char dest){
	int flag=0,i,counter=0,counter_p=0;
	float langle=Angle,t;
	while(locate()!=dest&&locate()!='B'){
		if(counter_p++%100==0){
			print("\ncurrent position:");
			putchar(locate());
			print("\nlooking for:");
			putchar(dest);
		}
		
		if(!*black&&--flag<0){
			going_straight=0;
			*lt = 1000;
	    *rt = 400;
			while(!*black){
				wait(1000);
			}
		}
		else if(*black){
			/*if(Angle-langle>90){
				t=Angle;
				*lt=-1000;
				*rt=1000;
				while(Angle-t>-10){
					wait(1000);
				}
			}else if(Angle-langle<-90){
				t=Angle;
				*rt=-1000;
				*lt=1000;
				while(Angle-t<10){
					wait(1000);
				}
			}*/
			going_straight=0;
			*lt = -900;
	    *rt = 1000;
			while(*black){
				wait(1000);
			}
			langle=Angle;
			flag=1;
		}else{
			*lt=*rt=1000;
			for(i=0;i<700&&!*black;++i){
				wait(1000);
			}
		}
	}
}
void turnLRs_u(int *lt,int *rt,short *black,char dest){
	int flag=0,i,counter=0,counter_p=0;
	float langle=Angle,t;
	while(locate()!=dest&&locate()!='B'){
		if(counter_p++%100==0){
			print("\ncurrent position:");
			putchar(locate());
			print("\nlooking for:");
			putchar(dest);
		}
		
		if(!*black&&--flag<0){
			going_straight=0;
			*lt = 1200;
	    *rt = 400;
			while(!*black){
				wait(1000);
			}
		}
		else if(*black){
			/*if(Angle-langle>90){
				t=Angle;
				*lt=-1000;
				*rt=1000;
				while(Angle-t>-10){
					wait(1000);
				}
			}else if(Angle-langle<-90){
				t=Angle;
				*rt=-1000;
				*lt=1000;
				while(Angle-t<10){
					wait(1000);
				}
			}*/
			going_straight=0;
			*lt = -900;
	    *rt = 1000;
			while(*black){
				wait(1000);
			}
			langle=Angle;
			flag=1;
		}else{
			*lt=*rt=1000;
			for(i=0;i<700&&!*black;++i){
				wait(1000);
			}
		}
	}
}
void turnR_u(char dest){
	turnLR_u(&left_target,&right_target,&black2,dest);
}
void turnRs_u(char dest){
	turnLRs_u(&left_target,&right_target,&black2,dest);
}
void turnL_u(char dest){
	turnLR_u(&right_target,&left_target,&black1,dest);
}
void turnLR_w(int *lt,int *rt,short *black,char dest){
	int flag=0,i,counter=0,counter_p=0;
	float langle=Angle,t;
	while(locate()==dest){
		if(counter_p++%100==0){
			print("\ncurrent position:");
			putchar(locate());
			print("\nleaving from:");
			putchar(dest);
		}
		
		if(!*black&&--flag<0){
			going_straight=0;
			*lt = 1000;
	    *rt = 200;
			while(!*black){
				wait(1000);
			}
		}
		else if(*black){
			/*if(Angle-langle>90){
				t=Angle;
				*lt=-1000;
				*rt=1000;
				while(Angle-t>-10){
					wait(1000);
				}
			}else if(Angle-langle<-90){
				t=Angle;
				*rt=-1000;
				*lt=1000;
				while(Angle-t<10){
					wait(1000);
				}
			}*/
			going_straight=0;
			*lt = -900;
	    *rt = 1000;
			while(*black){
				wait(1000);
			}
			//langle=Angle;
			flag=1;
		}else{
			*lt=*rt=1000;
			for(i=0;i<700&&!*black;++i){
				wait(1000);
			}
		}
	}
}
void turnR_w(char dest){
	turnLR_w(&left_target,&right_target,&black2,dest);
}
void turnL_w(char dest){
	turnLR_w(&right_target,&left_target,&black1,dest);
}
void turnR(){
	turnLR(&left_target,&right_target,&black2);
//	int flag=0,i;
//	while(1){
//		if(!black2&&--flag<0){
//			going_straight=0;
//			left_target = 1200;
//	    right_target = 200;
//			while(!black2){
//				wait(1000);
//			}
//		}
//		else if(black2){
//			going_straight=0;
//			left_target = -900;
//	    right_target = 1200;
//			while(black2){
//				wait(1000);
//			}
//			//wait(200000);
//			flag=1;
//		}else{
//			left_target=right_target=1200;
//			for(i=0;i<200&&!black2;++i){
//				wait(1000);
//			}
//		}
//	}
}

void initR(){
//	float cAngle;
	left_target=right_target=1000;
	//while(locate()!='F');
	wait(3500000);
	turn_(1000,-1000,30);
	stop = 1;
	wait(10000);
	stop = 0;
	left_target=right_target=1000;
	wait(2000000);
//	right_target=-1000;
//	cAngle=Angle;
//	while(Angle-cAngle<50){
//		wait(1000);
//	}
//	left_target=right_target=1000;
//	wait(1500000);
//	left_target=1000;
//	right_target=1200;
//	wait(1000000);
}
void turnL(){
	turnLR(&right_target,&left_target,&black1);
//	int flag=0,i;
//	while(1){
//		if(!black1&&--flag<0){
//			going_straight=0;
//			left_target = -500;
//	    right_target = 1200;
//			while(!black1){
//				wait(1000);
//			}
//		}
//		else if(black1){
//			going_straight=0;
//			left_target = 1200;
//	    right_target = -900;
//			wait(200000);
//			flag=2;
//		}else{
//			left_target=right_target=1200;
//			for(i=0;i<200&&!black1;++i){
//				wait(1000);
//			}
//		}
//	}
}
void turnAroundp(){
			float currentAngle = Angle;
			left_target = 200;
	    right_target = 1200;
			while(Angle-currentAngle>-20)
				wait(10000);
			
			left_target=right_target=1000;
			wait(5000000);
			
			//currentAngle = Angle;
			left_target = 1200;
	    right_target = 200;
			while(Angle-currentAngle<10)
				wait(10000);
}

void setAB(){
	  if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)){
			myX = &AX;
			myY = &AY;
			myPenalty = &penalty_A;
			myScore = &score_A;
			isA=1;
			car_label = 0x01;
		}
		else{
			myX = &BX;
			myY = &BY;
			myPenalty = &penalty_B;
			myScore = &score_B;
			isA=0;
			car_label = 0x02;
		}
}
void turn__(){
	turn_(1000,800,5);
}
int main(void){
	int i,j;
	Our_Sys_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	USART2_init();
	print("\n----------\n");
	sensors_init();
	print("sensors ok\n");
	//I2C_Configuration();
	//MPU6050_Init();
	USART3_init();

	print("mpu ok\n");
	Delay(10000);
	

	if(!debug_mode){
	USART1_init();
	wireless_init();
	print("wifi ok\n");
	}

	TIM2_Detect_init(65534,23); 
	TIM4_Detect_init(65534,23); 
	Delay(10000);
	
	TIM3_PWM_Init(999,71);
	Delay(10000);
	TIM6_Config();
	Delay(1000);
	print("TIM OK");
  setAB();
	TIM5_Detect_init(65534,23);
	print("ALL OK");
	//while(mode!=0x01);
	//turnL();
	//turnR();

	
	/*
	while(1){
		if(1&&counter_Angle++>100000){
		print("\nAngle:");
		printn(Angle);
		counter_Angle = 0;
//		print("\nmode:");printn(mode);
//		print("\nround:");printn(rounds);
//		print("\nAX:");printn(AX);
//		print("\nAY:");printn(AY);
//		print("\nBX:");printn(BX);
//		print("\nBY:");printn(BY);
//		print("\n");
		}		
	}	
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////

	getman = 0;
	if(!debug_mode){
		while(locate()=='#'||wifi_ok==0){
			wait(100000);
			print("waiting for wifi...\n");
		}
		print("wifi ok\n");
	}
	
	//正式
	if(!debug_mode){
		while(mode!=0x01);
		wait(1000000);
		initR();
	}
	/*
	while(1){
		turnAround();
		stop = 1;
		wait(10000000);
		stop = 0;
	}
	*/
	//initR();
	for(i=0;;i=(i+1)%10000000){
		if(i%9000==0){
			print("\nAngle:");
			printn(Angle);
			print(",left_freq:");
			printn(left_freq);
//			print("\nleft_dir:");
//			printn(left_dir);		
			print("\nlocate:");
			putchar(locate());
		  print(",dest:");
			putchar(desti);
//			print(",isA:");
//			printn(isA);
			/*
			for(j=0;j<men_num;j++){
				print("\nmen[");
				printn(j);
				print("]:");
				printn(men_state[j]);
			}
			*/
		}
		if((!debug_mode)&&(black4||equ(locate(),"<>`@"))){
			pos=locate();
			desti=dest();
			flag=1;
			switch(pos){
				case '>':
					turnR_w('>');break;
				case '<':
					turnL_w('<');break;
				case '`':
					turnR_u('.');break;
				case '@':
					if(eq("ZH")){
						turnAround();
					};break;
				case 'u':
					if(desti=='D'||desti=='C'||desti=='Z'||desti=='H'||desti=='F'){ //修改
					}else{
					turnR_u('G');
					}break;
				case 'j':
					if(eq("ECXY")){ //修改
						turn_(750,500,35);
					}
					else{
						turnR_u('T');
					}
					break;
				case 'l':
					if(desti=='M'||desti=='P'||desti=='S'||desti=='N'||desti=='T'||desti=='J'){//修改
						turnAround();
					}
					else if(desti=='H'){ //修改
						straight_u('C');
						turn_(-1000,1000,-80);
						straight_u('Z');
					}
					else{
						turnR_u('F');
					}break;
				case 'f':
					if(desti=='D'||desti=='1'||desti=='C'||desti=='Z'||desti=='H'){ //修改
						turnAround();
					}
					else if(desti=='G'||desti=='3'||desti=='O'){ //修改
						turn_(-800,800,-40);
					}
					else {
						turnR_u('Q');
					}
					break;
				case 'n':
					if(desti=='T'||desti=='S'||desti=='O'||desti=='U'||desti=='G'||desti=='J'){
						turnAround();
					}else{
						turnR_u('E');
					}break;
				case 'e':case 'Y': 
					if(desti=='H'){
						turnAround();
					}
					/*else if(desti=='X'||desti=='F'||desti=='Q'||desti=='D'){ //修改
						turnR_w('e');
						straight_w('Y');
						straight_w('C');
					}*/
					else {
						turnR_u('M');
					}break;
				case 'h':
					if(desti=='T'||desti=='S'||desti=='O'||desti=='U'||desti=='G'||desti=='J'){ //修改
						turn_(-800,800,-35);
					}
					else {
						turnAround();
					}
					break;
				case 'd':
					if(eq("F2Q")){
						turnAround();
					}
					else if(eq("XMPSJ")){//修改
						turn_(-800,800,-45);
						straight_w('Z');
						straight_w('C');
					}
					else 
						turnRs_u('H');
					//turn_(1200,-1000,60);
					break;
				case 'm':
					if(equ(desti,"N4EY")){
						turnR_u('N');
					}
					else if(equ(desti,"OU13G")){//修改
						straight_u('S');		
						turn_(-1000,1000,-90);
					}
					else if(equ(desti,"LCX"))
						turnAround();
					else ;//turn__();
					break;
			  case 't':
					if(eq("J65E")){
						turnR_u('J');
					}
					else if(equ(desti,"RLXCF")){//修改
						straight_u('O');		
						turn_(-1000,1000,-90);
					}
					else if(desti=='N'||desti=='E')
						turnAround();
					else ;//turn__();
					break;
				case 'g':case 'O':
					if(desti=='U')
						turnR_u('U');
					else if(equ(desti,"PN4EY")){ //修改
						straight_u('R');		
						turn_(-1000,1000,-90);
					}
					else if(eq("J6TS"))
						turnAround();
					else ;//turn__();
					break;
				case 'q':
					if(desti=='L'||desti=='F'||desti=='D'||desti=='C')
						turnR_u('L');
					else if(equ(desti,"SJ6T")){//修改
						straight_u('P');		
						turn_(-1000,1000,-90);
					}
					else if(desti=='U'||desti=='G'||desti=='O')
						turnAround();
					else ;//turn__();
					break;
				default:;
			}
			if(flag==1){
				going_straight=0; //转弯记得改
				continue;
			}
			wait(200000);
		}
		if(debug_mode&&black4){
			turnAround();
			going_straight=0;
		}else
		if(!(black1^black2)){
			keepStraight();
		}
		else if(black1){
			wait(10000);
			if(black2)continue;
			going_straight=0;
			left_target = 1200;
	    right_target = -1200;
			for(i=0;i<10&&!black2;++i){
				wait(1000);
			}
		}
		else if(black2){
			wait(10000);
			if(black1)continue;
			going_straight=0;
			left_target = -1200;
	    right_target = 1200;
			for(i=0;i<10&&!black1;++i){
				wait(1000);
			}
		}
	}
	
}



