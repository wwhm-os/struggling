#ifndef _SENSOR_H
#define _SENSOR_H

#include "stm32f4xx.h"                  // Device header


/* CLOCK */
#define CLK_SENSOR_ROUTE   RCC_AHB1Periph_GPIOD
#define CLK_SENSOR_POS2   RCC_AHB1Periph_GPIOD  //光电开关-用于对准木块 
#define CLK_SENSOR_POS1   RCC_AHB1Periph_GPIOC  //行程开关-用于对准砝码
#define CLK_SENSOR_RESET   RCC_AHB1Periph_GPIOC  //行程开关-用于复位motor5和6

/* PORT */
#define GPIO_PORT_SENSOR_ROUTE   GPIOD
#define GPIO_PORT_SENSOR_POS2    GPIOD  //光电开关-用于对准木块 
#define GPIO_PORT_SENSOR_POS1    GPIOC  //行程开关-用于对准砝码
#define GPIO_PORT_SENSOR_RESET   GPIOC  //行程开关-用于复位motor5和6

/* PIN */
#define GPIO_PIN_SENSOR_ROUTE1   GPIO_Pin_0
#define GPIO_PIN_SENSOR_ROUTE2   GPIO_Pin_1
#define GPIO_PIN_SENSOR_ROUTE3   GPIO_Pin_2
#define GPIO_PIN_SENSOR_ROUTE4   GPIO_Pin_3
#define GPIO_PIN_SENSOR_ROUTE5   GPIO_Pin_4
#define GPIO_PIN_SENSOR_ROUTE6   GPIO_Pin_5
#define GPIO_PIN_SENSOR_ROUTE7   GPIO_Pin_6

#define GPIO_PIN_SENSOR_POS2     GPIO_Pin_7       //光电开关-用于对准木块 
#define GPIO_PIN_SENSOR_POS1     GPIO_Pin_7     //行程开关-用于对准砝码
#define GPIO_PIN_SENSOR_RESET    GPIO_Pin_8   //行程开关-用于复位motor5和6

typedef struct
{
    uint8_t sensor_route[7];
    uint8_t sensor_pos1;  //行程开关-用于对准砝码
    uint8_t sensor_pos2;  //光电开关-用于对准木块 
    uint8_t sensor_reset; //行程开关-用于复位motor5和6
}SensorTypeDef;

typedef enum
{
    SENSOR_ROUTE1,
    SENSOR_ROUTE2,
    SENSOR_ROUTE3,
    SENSOR_ROUTE4,
    SENSOR_ROUTE5,
    SENSOR_ROUTE6,
    SENSOR_ROUTE7
	
}Sensor_Route_enum_TypeDef;


extern volatile SensorTypeDef sensor;

void sensor_init(void);
void sensor_read(void);

#endif
