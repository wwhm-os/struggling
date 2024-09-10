#include "servo_motor.h"


void servo_init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /* GPIO_INIT */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   /* 时钟使能 */

    GPIO_InitStructure.GPIO_Pin = SERVO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(SERVO_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(SERVO_PORT, GPIO_PinSource6, GPIO_AF_TIM3); /* GPIO_AF */

    /* TIM3_INIT */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    /* 时钟使能 */
    /* base */
    TIM_TimeBaseStructure.TIM_Period = arr ;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(SERVO_TIM, &TIM_TimeBaseStructure);
    /* PWM mode */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 940; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC1Init(SERVO_TIM, &TIM_OCInitStructure);
    /* counter enable */
    TIM_Cmd(SERVO_TIM, ENABLE);
    
    set_servo_angle(45);
}

void set_servo_angle(int angle)
{
    int pulse_width = ( 1- (angle + 45)/1800.0 ) * (1000 - 1);
    if(pulse_width > 974)
    {
        pulse_width = 974;
    }
    else if(pulse_width < 875)
    {
        pulse_width = 875;
    }   
    TIM_SetCompare1(SERVO_TIM, pulse_width);
}
