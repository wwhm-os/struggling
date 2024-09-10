#include "motor_control.h"

/*****PID*****/
ClassicPidStructTypedef MotorPositionPid1,MotorSpeedPid1,MotorCurrentPid1;
ClassicPidStructTypedef MotorPositionPid2b,MotorPositionPid2,MotorPositionPid2r,MotorPositionPid2t,MotorSpeedPid2,MotorCurrentPid2;
ClassicPidStructTypedef MotorPositionPid3,MotorPositionPid3r,MotorSpeedPid3,MotorCurrentPid3;
ClassicPidStructTypedef MotorPositionPid4,MotorPositionPid4r,MotorSpeedPid4,MotorCurrentPid4;
ClassicPidStructTypedef MotorPositionPid5,MotorPositionPid5r,MotorSpeedPid5,MotorCurrentPid5;
ClassicPidStructTypedef MotorPositionPid6,MotorPositionPid6r,MotorSpeedPid6,MotorCurrentPid6;
ClassicPidStructTypedef MotorPositionPid7,MotorSpeedPid7,MotorCurrentPid7;
ClassicPidStructTypedef MotorPositionPid8,MotorSpeedPid8,MotorCurrentPid8;


volatile MotorTypeDef  EC22_Motor,Motor1,Motor2,Motor3,Motor4,Motor5,Motor6,Motor7,Motor8;
volatile MotorTypeDef  motorring;
volatile MotorTypeDef  frictionright,frictionleft,rotationmotor;
DifferenceTypeDef Diff1 = {0,1000,0,5,1};
volatile float HELMSpeed_Limit = 25000;
volatile float WHEELSpeed_Limit = 10000;

volatile MonotrochState monotroch_state, pre_state;

uint8_t flag_get_pid = false;
uint8_t flag_step;
uint8_t flag_task;

void MotorCurrentPidInit(void)
{
	MotorCurrentPid1.Kp = 1.0f;
	MotorCurrentPid1.Ki = 0.01f; 
	MotorCurrentPid1.Kd = 0.0f;
	MotorCurrentPid1.LimitOutput = 16000.0f;
	MotorCurrentPid1.LimitIntegral = 16000.0f;
	MotorCurrentPid1.Integral = 0;
	MotorCurrentPid1.PreError = 0;
    
  MotorCurrentPid2.Kp = 1.0f;
	MotorCurrentPid2.Ki = 0.01f; 
	MotorCurrentPid2.Kd = 0.0f;
	MotorCurrentPid2.LimitOutput = 16000.0f;
	MotorCurrentPid2.LimitIntegral = 16000.0f;
	MotorCurrentPid2.Integral = 0;
	MotorCurrentPid2.PreError = 0;

 MotorCurrentPid3.Kp = 1.0f;
	MotorCurrentPid3.Ki = 0.01f; 
	MotorCurrentPid3.Kd = 0.0f;
	MotorCurrentPid3.LimitOutput = 16000.0f;
	MotorCurrentPid3.LimitIntegral = 3800.0f;
	MotorCurrentPid3.Integral = 0;
	MotorCurrentPid3.PreError = 0;
	
	MotorCurrentPid4.Kp = 1.3f;
	MotorCurrentPid4.Ki = 0.03f; 
	MotorCurrentPid4.Kd = 0.0f;
	MotorCurrentPid4.LimitOutput = 16000.0f;
	MotorCurrentPid4.LimitIntegral = 3800.0f;
	MotorCurrentPid4.Integral = 0;
	MotorCurrentPid4.PreError = 0;
	
	MotorCurrentPid5.Kp = 1.3f;
	MotorCurrentPid5.Ki = 0.03f; 
	MotorCurrentPid5.Kd = 0.0f;
	MotorCurrentPid5.LimitOutput = 16000.0f;
	MotorCurrentPid5.LimitIntegral = 3800.0f;
	MotorCurrentPid5.Integral = 0;
	MotorCurrentPid5.PreError = 0;
	
	MotorCurrentPid6.Kp = 1.3f;
	MotorCurrentPid6.Ki = 0.03f; 
	MotorCurrentPid6.Kd = 0.0f;
	MotorCurrentPid6.LimitOutput = 16000.0f;
	MotorCurrentPid6.LimitIntegral = 3800.0f;
	MotorCurrentPid6.Integral = 0;
	MotorCurrentPid6.PreError = 0;
	
	MotorCurrentPid7.Kp = 1.3f;
	MotorCurrentPid7.Ki = 0.03f; 
	MotorCurrentPid7.Kd = 0.0f;
	MotorCurrentPid7.LimitOutput = 16000.0f;
	MotorCurrentPid7.LimitIntegral = 3800.0f;
	MotorCurrentPid7.Integral = 0;
	MotorCurrentPid7.PreError = 0;
	
	MotorCurrentPid8.Kp = 1.3f;
	MotorCurrentPid8.Ki = 0.03f; 
	MotorCurrentPid8.Kd = 0.0f;
	MotorCurrentPid8.LimitOutput = 16000.0f;
	MotorCurrentPid8.LimitIntegral = 3800.0f;
	MotorCurrentPid8.Integral = 0;
	MotorCurrentPid8.PreError = 0;
}

void MotorSpeedPidInit(void)
{
  MotorSpeedPid1.Kp = 200.0f;  
	MotorSpeedPid1.Ki =1.0f;     
	MotorSpeedPid1.Kd = 0.0f;
	MotorSpeedPid1.LimitOutput = 4000.0f;
	MotorSpeedPid1.LimitIntegral = 5000.0f;
	MotorSpeedPid1.Integral = 0;
	MotorSpeedPid1.PreError = 0;
    
  MotorSpeedPid2.Kp = 200.0f;//250
	MotorSpeedPid2.Ki =1.0f;//5    
	MotorSpeedPid2.Kd = 0.0f;
	MotorSpeedPid2.LimitOutput = 4000.0f;
	MotorSpeedPid2.LimitIntegral = 5000.0f;
	MotorSpeedPid2.Integral = 0;
	MotorSpeedPid2.PreError = 0;
    
  MotorSpeedPid3.Kp = 200.0f;//500
	MotorSpeedPid3.Ki =1.0f;//5
	MotorSpeedPid3.Kd = 0.0f;
	MotorSpeedPid3.LimitOutput = 4000.0f;
	MotorSpeedPid3.LimitIntegral = 5000.0f;
	MotorSpeedPid3.Integral = 0;
	MotorSpeedPid3.PreError = 0;
	
	MotorSpeedPid4.Kp = 300.0f;  
	MotorSpeedPid4.Ki =1.0f;
	MotorSpeedPid4.Kd = 0.0f;
	MotorSpeedPid4.LimitOutput =5000.0f;
	MotorSpeedPid4.LimitIntegral = 5000.0f;
	MotorSpeedPid4.Integral = 0;
	MotorSpeedPid4.PreError = 0;
	
	MotorSpeedPid5.Kp = 300.0f;  
	MotorSpeedPid5.Ki =0.0f;
	MotorSpeedPid5.Kd = 2.0f;
	MotorSpeedPid5.LimitOutput = 4000.0f;
	MotorSpeedPid5.LimitIntegral = 5000.0f;
	MotorSpeedPid5.Integral = 0;
	MotorSpeedPid5.PreError = 0;
	
	MotorSpeedPid6.Kp = 300.0f;  
	MotorSpeedPid6.Ki =0.0f;
	MotorSpeedPid6.Kd = 2.0f;
	MotorSpeedPid6.LimitOutput = 4000.0f;
	MotorSpeedPid6.LimitIntegral = 5000.0f;
	MotorSpeedPid6.Integral = 0;
	MotorSpeedPid6.PreError = 0;
	
	MotorSpeedPid7.Kp = 200.0f;  
	MotorSpeedPid7.Ki =1.0f;
	MotorSpeedPid7.Kd = 0.0f;
	MotorSpeedPid7.LimitOutput = 4000.0f;
	MotorSpeedPid7.LimitIntegral = 5000.0f;
	MotorSpeedPid7.Integral = 0;
	MotorSpeedPid7.PreError = 0;
	
	MotorSpeedPid8.Kp = 200.0f;  
	MotorSpeedPid8.Ki =1.0f;
	MotorSpeedPid8.Kd = 0.0f;
	MotorSpeedPid8.LimitOutput = 4000.0f;
	MotorSpeedPid8.LimitIntegral = 5000.0f;
	MotorSpeedPid8.Integral = 0;
	MotorSpeedPid8.PreError = 0;
}

void MotorPositionPidInit(void)
{
  MotorPositionPid1.Kp = 2.0f;
	MotorPositionPid1.Ki = 0.0f;   
	MotorPositionPid1.Kd = 4.1f;
	MotorPositionPid1.LimitOutput = 250.0f;// 
	MotorPositionPid1.LimitIntegral = 250.0f;
	MotorPositionPid1.Integral = 0;
	MotorPositionPid1.PreError = 0;
    
	MotorPositionPid2b.Kp = 1.0f;
	MotorPositionPid2b.Ki = 0.0f;
	MotorPositionPid2b.Kd = 4.1f;
	MotorPositionPid2b.LimitOutput = 500.0f;// 280
	MotorPositionPid2b.LimitIntegral = 500.0f;
	MotorPositionPid2b.Integral = 0;
	MotorPositionPid2b.PreError = 0;
	
    MotorPositionPid2.Kp = 2.0f;
	MotorPositionPid2.Ki = 0.0f;
	MotorPositionPid2.Kd = 8.1f;
	MotorPositionPid2.LimitOutput = 250.0f;// 280
	MotorPositionPid2.LimitIntegral = 250.0f;
	MotorPositionPid2.Integral = 0;
	MotorPositionPid2.PreError = 0;
	
	MotorPositionPid2r.Kp = 0.8f;//2.0f;//4
	MotorPositionPid2r.Ki = 0.0f;//0.02
	MotorPositionPid2r.Kd = 0.0f;//3.5f;//4.5
	MotorPositionPid2r.LimitOutput = 500.0f;// 
	MotorPositionPid2r.LimitIntegral = 200.0f;
	MotorPositionPid2r.Integral = 0;
	MotorPositionPid2r.PreError = 0;
	
	MotorPositionPid2t.Kp = 5.0f;//4
	MotorPositionPid2t.Ki = 0.0f;//0.02
	MotorPositionPid2t.Kd = 7.5f;//4.5
	MotorPositionPid2t.LimitOutput = 500.0f;// 
	MotorPositionPid2t.LimitIntegral = 200.0f;
	MotorPositionPid2t.Integral = 0;
	MotorPositionPid2t.PreError = 0;
   
  MotorPositionPid3.Kp = 6.0f;//1.5
	MotorPositionPid3.Ki = 0.0f;
	MotorPositionPid3.Kd = 4.1f;//6.5
	MotorPositionPid3.LimitOutput = 500.0f;// 
	MotorPositionPid3.LimitIntegral = 500.0f;
	MotorPositionPid3.Integral = 0;
	MotorPositionPid3.PreError = 0;
	
	MotorPositionPid3r.Kp = 0.8f;
	MotorPositionPid3r.Ki = 0.0f;
	MotorPositionPid3r.Kd = 3.8f;
	MotorPositionPid3r.LimitOutput = 250.0f;// 
	MotorPositionPid3r.LimitIntegral = 200.0f;
	MotorPositionPid3r.Integral = 0;
	MotorPositionPid3r.PreError = 0;
	
	MotorPositionPid4.Kp = 2.2f;//5
	MotorPositionPid4.Ki = 0.0f;
	MotorPositionPid4.Kd = 3.5f;//6.5
	MotorPositionPid4.LimitOutput = 500.0f;// 
	MotorPositionPid4.LimitIntegral = 200.0f;
	MotorPositionPid4.Integral = 0;
	MotorPositionPid4.PreError = 0;
	
	MotorPositionPid4r.Kp = 2.2f;//2.2
 
	MotorPositionPid4r.Ki = 0.0f;
	MotorPositionPid4r.Kd = 6.4f;//20,10
	MotorPositionPid4r.LimitOutput = 500.0f;// 
	MotorPositionPid4r.LimitIntegral = 200.0f;
	MotorPositionPid4r.Integral = 0;
	MotorPositionPid4r.PreError = 0;
	
	MotorPositionPid5.Kp = 1.5f;//2.5
	MotorPositionPid5.Ki = 0.0001f;
	MotorPositionPid5.Kd = 0.044f;//20,10
	MotorPositionPid5.LimitOutput = 20.0f;//***********20 
	MotorPositionPid5.LimitIntegral = 10.0f;
	MotorPositionPid5.Integral = 0;
	MotorPositionPid5.PreError = 0;
	
	MotorPositionPid5r.Kp = 1.5f;//2.2
	MotorPositionPid5r.Ki = 0.0001f;
	MotorPositionPid5r.Kd = 0.044f;//20,10
	MotorPositionPid5r.LimitOutput = 20.0f;//************20 
	MotorPositionPid5r.LimitIntegral = 10.0f;
	MotorPositionPid5r.Integral = 0;
	MotorPositionPid5r.PreError = 0;

	/***************************  not get *********************/
	MotorPositionPid6.Kp = 2.0f;//2
	MotorPositionPid6.Ki = 0.001f;//9.99999975e-05f;
	MotorPositionPid6.Kd = 0.004f;//10.0f;//100
	MotorPositionPid6.LimitOutput = 100.0f;// 
	MotorPositionPid6.LimitIntegral = 20.0f;
	MotorPositionPid6.Integral = 0;
	MotorPositionPid6.PreError = 0;
    
	/***************************  get *********************/
	MotorPositionPid6r.Kp = 2.0f;//2.2
	MotorPositionPid6r.Ki = 0.001f;
	MotorPositionPid6r.Kd = 0.004f;//20,10
	MotorPositionPid6r.LimitOutput = 70.0f;// 
	MotorPositionPid6r.LimitIntegral = 20.0f;
	MotorPositionPid6r.Integral = 0;
	MotorPositionPid6r.PreError = 0;
	
	MotorPositionPid7.Kp = 2.0f;//2
	MotorPositionPid7.Ki = 0.0f;
	MotorPositionPid7.Kd = 1.0f;//100
	MotorPositionPid7.LimitOutput = 500.0f;// 
	MotorPositionPid7.LimitIntegral = 2000.0f;
	MotorPositionPid7.Integral = 0;
	MotorPositionPid7.PreError = 0;
	
	MotorPositionPid8.Kp = 2.0f;//2
	MotorPositionPid8.Ki = 0.0f;
	MotorPositionPid8.Kd = 1.0f;//100
	MotorPositionPid8.LimitOutput = 500.0f;// 
	MotorPositionPid8.LimitIntegral = 2000.0f;
	MotorPositionPid8.Integral = 0;
	MotorPositionPid8.PreError = 0;
	
	
}



void PidInit(void)
{
	MotorCurrentPidInit();
	MotorSpeedPidInit();	  
    MotorPositionPidInit();
}

float VESCPidRegulate(float Reference, float PresentFeedback,ClassicPidStructTypedef *PID_Struct)
{
	float error;
	float error_inc;
	float pTerm;
	float iTerm;
	float dTerm;
	float dwAux;
	float output;
	/*error computation*/
	error = utils_angle_difference(Reference, PresentFeedback);	//走劣弧
	
	/*proportional term computation*/
	pTerm = error * PID_Struct->Kp;
	/*Integral term computation*/
	
	iTerm = error * PID_Struct->Ki;
	
	dwAux = PID_Struct->Integral + iTerm;
	/*limit integral*/
	if (dwAux > PID_Struct->LimitIntegral)
	{
		PID_Struct->Integral = PID_Struct->LimitIntegral;
	} else if (dwAux < -1*PID_Struct->LimitIntegral)
	{
		PID_Struct->Integral = -1*PID_Struct->LimitIntegral;
	} else
	{
	  PID_Struct->Integral = dwAux;
	}
	/*differential term computation*/
	
	error_inc = error - PID_Struct->PreError;
	dTerm = error_inc * PID_Struct->Kd;
	PID_Struct->PreError = error;

	output = pTerm + PID_Struct->Integral + dTerm;

	/*limit output*/
	if (output >= PID_Struct->LimitOutput)
	{
		return (PID_Struct->LimitOutput);
	} 
	else if (output < -1.0f*PID_Struct->LimitOutput)
	{
		return (-1.0f*PID_Struct->LimitOutput);
	} 
	else
	{
		return output;
	}
}

float utils_angle_difference(float angle1, float angle2) {
	float difference = angle1 - angle2;
	while (difference < -180.0f) difference += 2.0f * 180.0f;
	while (difference > 180.0f) difference -= 2.0f * 180.0f;
	return difference;
}

float ClassicPidRegulate(float Reference, float PresentFeedback,ClassicPidStructTypedef *PID_Struct)
{
	float error;
	float error_inc;
	float pTerm;
	float iTerm;
	float dTerm;
	float dwAux;
	float output;
	/*error computation*/
	error = Reference - PresentFeedback;
	
	
	
	/*proportional term computation*/
	pTerm = error * PID_Struct->Kp;
	/*Integral term computation*/
	
	iTerm = error * PID_Struct->Ki;
	
	dwAux = PID_Struct->Integral + iTerm;
	/*limit integral*/
	if (dwAux > PID_Struct->LimitIntegral)
	{
		PID_Struct->Integral = PID_Struct->LimitIntegral;
	} else if (dwAux < -1*PID_Struct->LimitIntegral)
	{
		PID_Struct->Integral = -1*PID_Struct->LimitIntegral;
	} else
	{
	  PID_Struct->Integral = dwAux;
	}
	/*differential term computation*/
	
	error_inc = error - PID_Struct->PreError;
	dTerm = error_inc * PID_Struct->Kd;
	PID_Struct->PreError = error;

	output = pTerm + PID_Struct->Integral + dTerm;

	/*limit output*/
	if (output >= PID_Struct->LimitOutput)
	{
		return (PID_Struct->LimitOutput);
	} else if (output < -1.0f*PID_Struct->LimitOutput)
	{
		return (-1.0f*PID_Struct->LimitOutput);
	} else
	{
		return output;
	}
}

void MotorUpdate(short I1, short I2,short I3,short I4)
{
	CanTxMsg TxMessage;
	//uint16_t i = 0;
	uint8_t TransmitMailbox = CAN_TxStatus_NoMailBox;	
	uint16_t  time_out_count = 0;
	TxMessage.StdId=0x200;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=8;
	
	TxMessage.Data[0]=I1>>8;
	TxMessage.Data[1]=I1&0xff;
	TxMessage.Data[2]=I2>>8;
	TxMessage.Data[3]=I2&0xff;
	TxMessage.Data[4]=I3>>8;
	TxMessage.Data[5]=I3&0xff;
	TxMessage.Data[6]=I4>>8;
	TxMessage.Data[7]=I4&0xff;
	
	while((TransmitMailbox == CAN_TxStatus_NoMailBox) && (time_out_count++ != 0xFF))
  {
		TransmitMailbox = CAN_Transmit(CAN2, &TxMessage);
	}
}

void MotorUpdate1(short I1, short I2,short I3,short I4)
{
	CanTxMsg TxMessage;
	//uint16_t i = 0;
	uint8_t TransmitMailbox = CAN_TxStatus_NoMailBox;	
	uint16_t  time_out_count = 0;
	TxMessage.StdId=0x1ff;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=8;
	
	TxMessage.Data[0]=I1>>8;
	TxMessage.Data[1]=I1&0xff;
	TxMessage.Data[2]=I2>>8;
	TxMessage.Data[3]=I2&0xff;
	TxMessage.Data[4]=I3>>8;
	TxMessage.Data[5]=I3&0xff;
	TxMessage.Data[6]=I4>>8;
	TxMessage.Data[7]=I4&0xff;
	
	while((TransmitMailbox == CAN_TxStatus_NoMailBox) && (time_out_count++ != 0xFF))
  {
		TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
	}
}

void Motor_Position_Update(float Motor1_position,float Motor2_position,float Motor3_position,float Motor4_position,float Motor5_position,float Motor6_position,float Motor7_position,float Motor8_position )
{
	Motor1.State=PIDPOSITION;
	Motor2.State=PIDPOSITION;
	Motor3.State=PIDPOSITION;
	Motor4.State=PIDPOSITION;
	Motor5.State=PIDPOSITION;
	Motor6.State=PIDSPEED;
	Motor7.State=PIDPOSITION;
	Motor8.State=PIDPOSITION;
	//Motor3.State=PIDSPEED;
	
	Motor1.PositionExpected=Motor1_position;
	Motor2.PositionExpected=Motor2_position;
	Motor3.PositionExpected=Motor3_position;
	Motor4.PositionExpected=Motor4_position;
	Motor5.PositionExpected=Motor5_position;
	Motor6.PositionExpected=Motor6_position;
	Motor7.PositionExpected=Motor7_position;
	Motor8.PositionExpected=Motor8_position;
	//Motor3.SpeedExpected=Motor3_position;
}
void Difference1MotorSend(DifferenceTypeDef *Diffx){
	Motor1.PositionExpected = Diffx->helm_angle;

	/********************speed limit*******************/
	if(Diffx->helm_speed > HELMSpeed_Limit)
		Diffx->helm_speed = HELMSpeed_Limit;
	if(Diffx->wheel_speed > WHEELSpeed_Limit)
		Diffx->wheel_speed = WHEELSpeed_Limit;

	/*******************set inferior arc & 180° equivalent*********************/
	while(Motor1.PositionExpected - Motor1.PositionMeasure > 90){
		Motor1.PositionExpected -= 180; 
		Diffx->dir = -Diffx->dir;
	}
	while(Motor1.PositionExpected - Motor1.PositionMeasure < -90){
		Motor1.PositionExpected += 180;
		Diffx->dir = -Diffx->dir;
	}
	
	/*******************Helm 's speed loop，position loop & calculate wheel motor speed*********************/
	float FixDelta = Motor1.PositionExpected - Motor1.PositionMeasure;
	if(fabs(FixDelta) > Diffx->position_loop_range){
		if(FixDelta > 0){
			Motor1.SpeedExpected = -Diffx->helm_speed;
		}else{
			Motor1.SpeedExpected = Diffx->helm_speed;
		}
	
	}else{

	}

	/********************send wheel motor speed*******************/
	
}

void Difference1SpeedSend(DifferenceTypeDef *Diffx){

	
}


/*******************   my     ************************/
void my_forward(float speed)
{
    Motor1.SpeedExpected = 0.0f;
    Motor2.SpeedExpected = - speed;
    Motor3.SpeedExpected = speed;

}

void my_trace_forward(float speed, float motor3_coeffcient, float motor2_coeffcient, float motor1_coeffcient)
{
    Motor1.SpeedExpected = speed * motor1_coeffcient;
    Motor2.SpeedExpected = - speed * motor2_coeffcient; // left
    Motor3.SpeedExpected = speed * motor3_coeffcient; // right
}

void my_trace_backward(float speed, float motor3_coeffcient, float motor2_coeffcient, float motor1_coeffcient)
{
    Motor1.SpeedExpected = speed * motor1_coeffcient;
    Motor2.SpeedExpected = speed * motor2_coeffcient; // left
    Motor3.SpeedExpected = - speed * motor3_coeffcient; // right
}

void my_backward(float speed)
{
    Motor1.SpeedExpected = 0.0f;
    Motor2.SpeedExpected = speed;
    Motor3.SpeedExpected = - speed;

}

void my_turn_left(float speed)
{
    Motor1.SpeedExpected = - speed;
    Motor2.SpeedExpected = - speed;
    Motor3.SpeedExpected = - speed;

}

void my_turn_right(float speed)
{
    Motor1.SpeedExpected = speed;
    Motor2.SpeedExpected = speed;
    Motor3.SpeedExpected = speed;

}

void my_allstop(void)
{
    Motor1.SpeedExpected = 0.0f;
    Motor2.SpeedExpected = 0.0f;
    Motor3.SpeedExpected = 0.0f;

}

void my_motor_control(float motor1_speed, float motor2_speed, float motor3_speed)
{
    Motor1.SpeedExpected = motor1_speed;
    Motor2.SpeedExpected = motor2_speed;
    Motor3.SpeedExpected = motor3_speed;

}
void my_right_translation(float speed)
{
    Motor1.SpeedExpected = - speed * 2.0f / 3.0f;
    Motor2.SpeedExpected = speed / 3.0f;
    Motor3.SpeedExpected = speed / 3.0f;
}

void my_left_translation(float speed)
{
    Motor1.SpeedExpected = speed * 2.0f / 3.0f;
    Motor2.SpeedExpected = - speed / 3.0f;
    Motor3.SpeedExpected = - speed / 3.0f;
}

/**
  * @brief  trace_straight_route
  * @note   
  * @param  speed 速度大小   
  *         flag
  *         取值：FORWARD 1
  *               BACKWARD 0
  * @retval 7 for all lited(need to change speed or stop)
  *         8 for all unlited(need to stop)
  *         1 for on the road
  *         2 for 偏移 the road
  *         0 for out_road but uncertain state
  *
  */
uint8_t trace_straight_route(float speed, uint8_t *p_sensor_bit_info, uint8_t flag)
{
    uint8_t flag_route_state = 0;

    switch(*p_sensor_bit_info)
    {
        case 0x7F: // all_unlited  0111 1111 
                    my_allstop();
                    flag_route_state = 8;            
                    break;            

        case 0x77:// only_D4_lited  0111 0111
                    if(flag)
                    {
                        my_trace_forward(speed, 1.0f, 1.0f, 0.0f);                        
                    }
                    else
                    {
                        my_trace_backward(speed, 1.0f, 1.0f, 0.0f);                                                
                    }
                    flag_route_state = 1;
                    break;
        
        case 0x7B:// only_D3_lited  0111 1011
                    if(flag)
                    {
                        my_trace_forward(speed, 1.0f, 0.4f, 0.1f);                        
                    }
                    else
                    {
                        my_trace_backward(speed, 0.4f, 1.0f, 0.1f);                        
                    }
                    flag_route_state = 1;
                    break;

        case 0x6F:// only_D5_lited  0110 1111
                    if(flag)
                    {
                        my_trace_forward(speed, 0.4f, 1.0f, -0.1f);                      
                    }
                    else
                    {
                        my_trace_backward(speed, 1.0f, 0.4f, -0.1f);                        
                    }
                    flag_route_state = 1;
                    break;

        case 0x73:// only_D4D3_lited  0111 0011
                    flag_route_state = 1;
                    if(flag)
                    {
                        my_trace_forward(speed, 1.0f, 0.7f, 0.0f);                    
                    }
                    else
                    {
                        my_trace_backward(speed, 0.7f, 1.0f, 0.0f);                        
                    } 

                    break;
        
        case 0x67:// only_D4D5_lited  0110 0111
                    if(flag)
                    {
                        my_trace_forward(speed, 0.7f, 1.0f, -0.0f);                
                    }
                    else
                    {
                        my_trace_backward(speed, 1.0f, 0.7f, -0.0f);                        
                    } 
                    flag_route_state = 1;

                    break;
        
        case 0x00:  // all_lited  0000 0000
                    //my_allstop(); 
                    flag_route_state = 7;
                    break;
        default:
                    flag_route_state = 0;
                    break;
    } // end of switch_case(xunxian)
    
    return flag_route_state;
}

uint8_t trace_round_route(float speed, uint8_t *p_sensor_bit_info)
{
    uint8_t flag_route_state = 1;
    
    switch(*p_sensor_bit_info)
    {
        case 0x7F: // all_unlited  0111 1111  
                    flag_route_state = 8;                
                    my_allstop();
                    break;            

        case 0x77:// only_D4_lited  0111 0111
                    my_trace_forward(speed, 1.0f, 1.0f, 0.0f);
                    break;
        
        case 0x7B:// only_D3_lited  0111 1011
                    my_trace_forward(speed, 1.0f, 0.2f, 0.4f);
                    break;

        case 0x6F:// only_D5_lited  0110 1111
                    my_trace_forward(speed, 0.2f, 1.0f, -0.4f);
                    break;

        case 0x73:// only_D4D3_lited  0111 0011
                    my_trace_forward(speed, 1.0f, 0.35f, 0.3f);
                    break;
        
        case 0x67:// only_D4D5_lited  0110 0111
                    my_trace_forward(speed, 0.35f, 1.0f, -0.3f);
                    break;
        
        case 0x4F:// only_D6D5_lited  0100 1111
                    my_trace_forward(speed, 0.0f, 1.0f, -0.45f);
                    break;
        
        case 0x79:// only_D2D3_lited  0111 1001
                    my_trace_forward(speed, 1.0f, 0.0f, 0.45f);
                    break;
        
        case 0x5F:// only_D6_lited  0101 1111
                    my_trace_forward(speed, -0.3f, 1.0f, -0.5f);
                    break;
        
        case 0x7D:// only_D2_lited  0111 1101
                    my_trace_forward(speed, 1.0f, -0.3f, 0.5f);
                    break;
        
        case 0x1F:// only_D6D7_lited  0001 1111
                    my_trace_forward(speed, -0.5f, 1.0f, -0.5f);
                    break;
        
        case 0x7C:// only_D2D1_lited  0111 1100
                    my_trace_forward(speed, 1.0f, -0.5f, 0.5f);
                    break;
        
        case 0x00:  // all_lited  0000 0000
                    //my_allstop(); 
                    flag_route_state = 7;
                    break;
                    
        default:
                    flag_route_state = 0;
    } // end of switch_case(xunxian)
    if(*p_sensor_bit_info == 0x4F | *p_sensor_bit_info == 0x5F | *p_sensor_bit_info == 0x79 | *p_sensor_bit_info == 0x7D)
    {
        flag_route_state = 2;
    }
    return flag_route_state;
}

/**
  * @brief  motor5和motor6 复位
  * @note   
  * @param None
  * @retval None
  */
void motor_reset(void)
{
    OS_ERR err;
    
    Motor5.State=PIDPOSITION;
    Motor6.State=PIDPOSITION;
    
    set_servo_angle(LOOSE);
    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
    flag_get_pid = false;    
    Motor5.PositionExpected = POS_RESET;
    Motor6.PositionExpected = POS_RESET;
    while(!FLAG_MOTOR5_ARRIVAL1(POS_RESET, 1.0f) || !FLAG_MOTOR6_ARRIVAL1(POS_RESET, 1.0f))
    {
         OSTimeDlyHMSM(0, 0, 0, 5, OS_OPT_TIME_HMSM_STRICT, &err);
    }
    Motor5.State=IDLE;
    Motor6.State=IDLE;
}

/**
  * @brief  拾取砝码,并放置于STORE区域
  * @note   **** motor and servo must arrive at the RESET_POS or WAIT_POS before use the function****
  * @param  放置砝码的盘的位置
  *         取值：
  *             @arg POS_STORE1
  *             @arg POS_STORE2
  *             @arg -POS_STORE1
  *             @arg -POS_STORE2
  * @retval None
  */
bool get_and_store(float store_pos)
{
    OS_ERR ERR;
    uint8_t flag_innal_step = STEP1;
    uint8_t temp = false;
    
    while(1)
    {
        switch(flag_innal_step)
        {
            case STEP1:
                        Motor6.PositionExpected = POS_GET;
                        if(FLAG_MOTOR6_ARRIVAL(POS_GET))
                        {
                            flag_innal_step = STEP2;
                        }
                        break;
            case STEP2:
                        set_servo_angle(CLOSE);
                        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &ERR); 
            
                        flag_get_pid = true;
                        Motor6.PositionExpected = POS_SAVE_GET;
                        if(FLAG_MOTOR6_ARRIVAL(POS_SAVE_GET))
                        {
                            flag_innal_step = STEP3;
                        }
                        break;
            case STEP3:
                        Motor5.PositionExpected = store_pos;
                        if(FLAG_MOTOR5_ARRIVAL(store_pos))
                        {
                            flag_innal_step = STEP4;
                        }
                        break;
            case STEP4:
                        Motor6.PositionExpected = POS_SAVE_PUT;
                        if(FLAG_MOTOR6_ARRIVAL(POS_SAVE_PUT))
                        {
                            flag_innal_step = STEP5;
                        }
                        break;
            case STEP5:
                        set_servo_angle(LOOSE);
                        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &ERR); 
            
                        flag_get_pid = false;
                        Motor5.PositionExpected = POS_RESET;
                        if(FLAG_MOTOR5_ARRIVAL(POS_RESET))
                        {
                            flag_innal_step = STEP_OVER;
                        }
                        break;
            case STEP_OVER:
                        return true;
        }
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &ERR); 
    }

}

bool get_from_the_store(float store_pos)
{
    OS_ERR ERR;
    uint8_t flag_innal_step = STEP1;
    uint8_t temp = false;
    
    while(1)
    {
        switch(flag_innal_step)
        {
            case STEP1:
                        Motor6.PositionExpected = POS_SAVE_PUT;
                        Motor5.PositionExpected = store_pos;
                        if(FLAG_MOTOR6_ARRIVAL(POS_SAVE_PUT) && FLAG_MOTOR5_ARRIVAL(store_pos))
                        {
                            flag_innal_step = STEP2;
                        }
                        break;
            case STEP2:
                        set_servo_angle(CLOSE);
                        OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &ERR); 
            
                        flag_get_pid = true;
                        Motor6.PositionExpected = POS_SAVE_GET;
                        if(FLAG_MOTOR6_ARRIVAL(POS_SAVE_GET))
                        {
                            flag_innal_step = STEP3;
                        }
                        break;
            case STEP3:
                        Motor6.PositionExpected = POS_PRE_PUT_HIGH;
                        Motor5.PositionExpected = POS_RESET;
                        if(FLAG_MOTOR6_ARRIVAL(POS_PRE_PUT_HIGH) && FLAG_MOTOR5_ARRIVAL(POS_RESET))
                        {
                            flag_innal_step = STEP_OVER;
                        }
                        break;
            case STEP_OVER:
                        return true;
        }
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &ERR); 
    }
}



/**
  * @brief  放置砝码于STORE区域
  * @note   **** weight has been gotten before use the function ****
  * @param  放置砝码的盘的位置
  *         取值：
  *             @arg POS_STORE1
  *             @arg POS_STORE2
  *             @arg -POS_STORE1
  *             @arg -POS_STORE2
  * @retval None
  */
bool store_weight(float store_pos)
{
    OS_ERR ERR;
    uint8_t flag_innal_step = STEP2;
    uint8_t temp = false;
    
    while(1)
    {
        switch(flag_innal_step)
        {
            case STEP2:
                        flag_get_pid = true;
                        Motor6.PositionExpected = POS_SAVE_GET;
                        if(FLAG_MOTOR6_ARRIVAL(POS_SAVE_GET))
                        {
                            flag_innal_step = STEP3;
                        }
                        break;
            case STEP3:
                        Motor5.PositionExpected = store_pos;
                        if(FLAG_MOTOR5_ARRIVAL(store_pos))
                        {
                            flag_innal_step = STEP4;
                        }
                        break;
            case STEP4:
                        Motor6.PositionExpected = POS_SAVE_PUT;
                        if(FLAG_MOTOR6_ARRIVAL(POS_SAVE_PUT))
                        {
                            flag_innal_step = STEP_OVER;
                        }
                        break;
            case STEP_OVER:
                        return true;
        }
        OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &ERR); 
    }

}

void get_and_put(float store_pos)
{
    
}


