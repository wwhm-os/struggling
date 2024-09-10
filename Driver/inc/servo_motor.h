#ifndef __SERVO_MOTOR
#define __SERVO_MOTOR

#include "stm32f4xx.h"


#define SERVO_PIN GPIO_Pin_6
#define SERVO_PORT GPIOA
#define SERVO_TIM TIM3
#define SERVO_CH TIM_Channel_1

void servo_init(uint16_t arr, uint16_t psc);
void set_servo_angle(int angle);


#endif
