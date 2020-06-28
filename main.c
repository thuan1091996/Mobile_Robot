// *****************************************************************************
//
//------------------------- 0.Project information ------------------------------
// Mobile Robot
// Author : TRAN MINH THUAN
// Date: August, 21th 2019
// Project associate with TM4C123, Keil 5
// Hardware:
// Motor control: QEI peripheral to read motor's speed
//                |  PhA  |  PhB
//         -------+-------+--------
//          QEI0  |  PD6  |  PD7
//          QEI1  |  PC5  |  PC6
//
// Bridge-H Control:	PWM peripheral to control motor's speed
//                  |  IN1  |  IN2  |  EN
//          --------+-------+-------+-------
//          Motor1  |  PE0  |  PE1  |  PE4
//          Motor2  |  PE2  |  PE3  |  PE5
//
// HC - SR Sensors: Detect obstacles with 5 SRF sensors
//                |  Trigger |  Echo
//         -------+----------+--------
//          SR00  |  PB0     |  PA2
//          SR01  |  PB1     |  PA3
//          SR02  |  PB2     |  PA4
//          SR03  |  PB3     |  PA5
//          SR04  |  PA7     |  PA6
// *****************************************************************************


// *****************************************************************************
//
//------------------------- 1.System requirement -------------------------------
//
// Communication with 5 HC-SRF sensors (SRF1-5) - Done
// Programming H-bridge 160W to control 2 motors - Testing
// Build driver to control robot direction
//
// ******************************************************************************


// *****************************************************************************
//
//---------------------- 2.Pre-Processor Directives ----------------------------
//
// *****************************************************************************
#include "Userlibs.h"
#include "BridgeH.h"
#include "Encoder.h"
#include "UART.h"
#include "Algorithm.h"
#include "os.h"
#include "HC_SRF.h"
#include "Profile.h"

#define 	F_Scheduler 				1000 //1000Hz
#define 	SRF_PRIORITY				3
#define		QEI0_PRIORITY				2
#define		QEI1_PRIORITY				2

// *****************************************************************************
//
//--------------------------- 3.Global Declarations ----------------------------
//
// *****************************************************************************

int32_t i32_Block;		   			 						//Blocking semaphore


// Function declaration
void Detect_Obstacles(void);
void Motor0_Control(void);
void Motor1_Control(void);
void Task4(void);
void Task5(void);
void Task6(void);
// *****************************************************************************
//
//------------------------ 4. Subroutines definition ---------------------------
//
// *****************************************************************************
int main(void)
{
		IntMasterDisable();
    SysCtlClockSet(SYSCTL_SYSDIV_2_5| SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //80MHz
		Profile_Init();													//Profile pins Init
//		BridgeH_GPIO_Init();										//Bridge - H GPIO Init
//    BridgeH_PWM_Init();			 								//Bridge - H PWM Init
//	  QEI_Init(QEI0_PRIORITY,QEI1_PRIORITY);	//QEI Init
		OS_AddThreads(&Detect_Obstacles,&Motor0_Control,&Motor1_Control,&Task4,&Task5,&Task6);
		OS_Launch(SysCtlClockGet()/F_Scheduler);
    while(1)																//This loop 
    {

    } 
}

int i=0,j=0,k=0;


// *****************************************************************************
//
//! Thread 0: Detect Obstacles
//! This thread is used to get distance data from SRF sensors
//! First initialize the SRF modules
//! Read data distance each SRF module at a time
//
//******************************************************************************
int32_t i32Data_SRF;		   			 						//New SRF Data Semaphore
int32_t i32Mutex_SRF;				     						//Mutex SRF (only read 1 at a time)
uint32_t SRF_Data[NUMB_SRF]={0}; 						//Store the Distances data
uint8_t  Running_SRF=4;					 						//Current reading SRF 
int32_t Distance_test;
void Detect_Obstacles(void)
{
		SRFs_Init(SRF_PRIORITY);								//Initialization for SRFs sensors
		OS_InitSemaphore(&i32Data_SRF,1);				//Data SRF synchronization semaphore
		OS_InitSemaphore(&i32Mutex_SRF,1);			//SRF Mutex semaphore
		while(1)
		{
					SRF_GetDistance(Running_SRF);
//        Running_SRF=(Running_SRF+1)%5;
//				if( Running_SRF == 0)	OS_Suspend();	//After read 5 sensor, call os suspend
		}
}




//Control Motor Speed
int8_t  i8Duty0=0;               //Duty cycle of Motor 0
int8_t  i8Duty1=0;               //Duty cycle of Motor 1
int16_t Vel_Motor0=0;            //Current velocity of motor 0
int16_t Vel_Motor1=0;            //Current velocity of motor 1
int16_t Ref_Vel0=100;            //Command velocity of motor 0
int16_t Ref_Vel1=100;            //Command velocity of motor 1
int32_t i32Data_Velocity0;			 //New Velocity Data Semaphore
int32_t i32Data_Velocity1;			 //New Velocity Data Semaphore
float   Pos_Motor0=0;            //Current position of motor 1
float   Pos_Motor1=0;            //Current position of motor 1
//float	 	kp0=0.5;

float	 	kp0=0.45;
float 	ki0=0.1;


void Motor0_Control(void)
{
//		OS_InitSemaphore(&i32Data_Velocity0,0);
		OS_InitSemaphore(&i32_Block,0);
		OS_Wait(&i32_Block);
		while(1)
		{
//			OS_Wait(&i32Data_Velocity0);
//			Update_Velocity0(&Vel_Motor0);	//Read velocity
//			i8Duty0=PID_Controller(kp0, ki0, 0, Ref_Vel0-Vel_Motor0, Vel_Motor0);	//Calculate new control variable
//			Update_PWM0(i8Duty0);
//			OS_Suspend();
		}
}

float	 	kp1=0.45;
float 	ki1=0.17;
void Motor1_Control(void)
{
//	OS_InitSemaphore(&i32Data_Velocity1,0);
	OS_Wait(&i32_Block);
	while(1)
	{
//			OS_Wait(&i32Data_Velocity1);
//			Update_Velocity1(&Vel_Motor1);	//Read velocity
//			i8Duty1=PID_Controller1(kp1, ki1, 0, Ref_Vel1-Vel_Motor1, Vel_Motor1);	//Calculate new control variable
//			Update_PWM1(i8Duty1);
//				OS_Suspend();
		
//			OS_Wait(&i32Data_Velocity1);
//			OS_Wait(&i32Data_Velocity0);
//			Update_Velocity0(&Vel_Motor0);	//Read velocity
//			Update_Velocity1(&Vel_Motor1);	//Read velocity
//			i8Duty0=PID_Controller(kp0, ki0, 0, Ref_Vel0-Vel_Motor0, Vel_Motor0);	//Calculate new control variable
//			i8Duty1=PID_Controller1(kp1, ki1, 0, Ref_Vel1-Vel_Motor1, Vel_Motor1);	//Calculate new control variable
//			Update_PWM0(i8Duty0);
//			Update_PWM1(i8Duty1);
//			OS_Suspend();
	} 
}


void Task4(void)
{
//		UART_Init(115200);														//UART Init
			OS_Wait(&i32_Block);

		while(1)
		{
//			UART_SendSpeed(Vel_Motor1);
//			OS_Suspend();
//			SysCtlDelay(1000);
//			UART_SendSpeed(i8Duty1);
//			OS_Suspend();
		}
}
void Task5(void)
{
		while(1)
		{
//			OS_InitSemaphore(&i32_Block,0);
//			OS_Wait(&i32_Block);
//		UART_SendSpeed(Vel_Motor1);
//		UART_SendSpeed(i8Duty1);
			Profile_Toggle3();
//			OS_Suspend();
		}
}

void Task6(void)
{
		while(1)
		{
			OS_Wait(&i32_Block);
//		UART_SendSpeed(Vel_Motor1);
//		UART_SendSpeed(i8Duty1);
//			Profile_Toggle3();
//			OS_Suspend();
		}
}
//////------------------------------------------------------------------------///////
// Testing feature
//////------------------------------------------------------------------------///////

//////------------------------------------------------------------------------///////
