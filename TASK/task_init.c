#include "system_config.h"
//#include "com_prop.h"
#include "RobotCAN_proplist.h"
#include "motor_control.h"

#include "task_init.h"
extern float Yaw_PC_Angle;
extern float Distance,Pitch_Position_Motor;
extern volatile MotorTypeDef Motorleft, Motorright, Motorpitch;
extern int on_shoot_speed;



void TaskIndicate(void *p_arg)
{
    OS_ERR err;
    while(1)
    {

        MY_LED_On(LED1);
        MY_LED_Off(LED2);
        OSTimeDlyHMSM(0, 0, 0, 300, OS_OPT_TIME_HMSM_STRICT, &err);
        MY_LED_Off(LED1);
        MY_LED_On(LED2);
        OSTimeDlyHMSM(0, 0, 0, 300, OS_OPT_TIME_HMSM_STRICT, &err);   
        
//        LED_Toggle(LED1);
//        OSTimeDlyHMSM(0, 0, 0, 300, OS_OPT_TIME_HMSM_STRICT, &err);
//        LED_Toggle(LED2);
//        OSTimeDlyHMSM(0, 0, 0, 300, OS_OPT_TIME_HMSM_STRICT, &err);
//        LED_Toggle(LED3);
//        OSTimeDlyHMSM(0, 0, 0, 300, OS_OPT_TIME_HMSM_STRICT, &err);
        
			OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);

    }
}


/***********USART**********/

BlueTooth_cmd bluetooth_cmd;
float dstpos1,dstpos2,dstpos3;

void TaskUSART(void *p_arg)
{
		u16 len;	
    OS_ERR err;
		
		while(1)
		{
			printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",Motor1.PositionMeasure,Motor2.PositionMeasure,Motor3.PositionMeasure,
																									Motor1.PositionExpected,Motor2.PositionExpected,Motor3.PositionExpected);		
				if(USART_RX_STA1&0x8000)
				{
					len=USART_RX_STA1&0x3fff;//得到此次接收到的数据长度
					if(len == 1 && USART_RX_BUF1[0] == 0x61)
					{
						printf("Forward！");
						bluetooth_cmd = forward;
					}
					if(len == 1 && USART_RX_BUF1[0] == 0x62)
					{
						printf("stop！");
						bluetooth_cmd = stop;
					}	
					if(len == 1 && USART_RX_BUF1[0] == 0x63)
					{
						printf("turn round！");
						bluetooth_cmd = turn;
					}	
					if(len == 1 && USART_RX_BUF1[0] == 0x64)
					{
						printf("grey trakc！");
						bluetooth_cmd = grey_track;
					}	
					if(len == 1 && USART_RX_BUF1[0] == 0x65)
					{
						Motor1.PositionMeasure = 0;
						Motor2.PositionMeasure = 0;
						Motor3.PositionMeasure = 0;
						

						printf("leftmove！");
						bluetooth_cmd = moveleft;
						dstpos1 = Motor1.PositionMeasure - 20;
						dstpos2 = Motor2.PositionMeasure - 20;
						dstpos3 = Motor3.PositionMeasure + 40;

					}	
					if(len == 1 && USART_RX_BUF1[0] == 0x66)
					{
						Motor1.PositionMeasure = 0;
						Motor2.PositionMeasure = 0;
						Motor3.PositionMeasure = 0;
						

						printf("turn_onestep!");
						bluetooth_cmd = turn_onestep;
						dstpos1 = Motor1.PositionMeasure + 20;
						dstpos2 = Motor2.PositionMeasure + 20;
						dstpos3 = Motor3.PositionMeasure + 20;
						
					}	
					USART_RX_STA1=0;
				}
				OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
					//			ZY_VisualScope_Send(on_shoot_speed,frictionleft.SpeedMeasure,frictionright.SpeedMeasure,Motor4.PositionMeasure*10);	
					//			//ZY_VisualScope_Send(Motor4.PositionExpected*10,Motor4.PositionMeasure*10,Motor4.SpeedMeasure,Motor4.CurrentMeasure);	
					//			OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
					//			//OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
		}
}


