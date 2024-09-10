#include "sensor.h"

volatile SensorTypeDef sensor;

void sensor_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    /* sensor route GPIO initial */
    RCC_AHB1PeriphClockCmd(CLK_SENSOR_ROUTE,ENABLE);             /* 使能时钟 */
        
    gpio_init_struct.GPIO_Pin = GPIO_PIN_SENSOR_ROUTE1;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;
    gpio_init_struct.GPIO_PuPd= GPIO_PuPd_NOPULL;
    GPIO_Init(GPIO_PORT_SENSOR_ROUTE, &gpio_init_struct);

    gpio_init_struct.GPIO_Pin = GPIO_PIN_SENSOR_ROUTE2;
    GPIO_Init(GPIO_PORT_SENSOR_ROUTE, &gpio_init_struct);
    
    gpio_init_struct.GPIO_Pin = GPIO_PIN_SENSOR_ROUTE3;
    GPIO_Init(GPIO_PORT_SENSOR_ROUTE, &gpio_init_struct);
    
    gpio_init_struct.GPIO_Pin = GPIO_PIN_SENSOR_ROUTE4;
    GPIO_Init(GPIO_PORT_SENSOR_ROUTE, &gpio_init_struct);
    
    gpio_init_struct.GPIO_Pin = GPIO_PIN_SENSOR_ROUTE5;
    GPIO_Init(GPIO_PORT_SENSOR_ROUTE, &gpio_init_struct);
    
    gpio_init_struct.GPIO_Pin = GPIO_PIN_SENSOR_ROUTE6;
    GPIO_Init(GPIO_PORT_SENSOR_ROUTE, &gpio_init_struct);
    
    gpio_init_struct.GPIO_Pin = GPIO_PIN_SENSOR_ROUTE7;
    GPIO_Init(GPIO_PORT_SENSOR_ROUTE, &gpio_init_struct);
    
    
    /* sensor postion1(行程开关) GPIO initial */    
    RCC_AHB1PeriphClockCmd(CLK_SENSOR_POS1,ENABLE);             /* 使能时钟 */
    
    gpio_init_struct.GPIO_Pin = GPIO_PIN_SENSOR_POS1;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;
    gpio_init_struct.GPIO_PuPd= GPIO_PuPd_NOPULL;
    GPIO_Init(GPIO_PORT_SENSOR_POS1, &gpio_init_struct);
    
    
    /* sensor postion1(光电开关) GPIO initial */    
    RCC_AHB1PeriphClockCmd(CLK_SENSOR_POS2,ENABLE);             /* 使能时钟 */
    gpio_init_struct.GPIO_Pin = GPIO_PIN_SENSOR_POS2;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;
    gpio_init_struct.GPIO_PuPd= GPIO_PuPd_NOPULL;
    GPIO_Init(GPIO_PORT_SENSOR_POS2, &gpio_init_struct);
    
    /*  行程开关-用于复位motor5和6 */
    RCC_AHB1PeriphClockCmd(CLK_SENSOR_RESET,ENABLE);             /* 使能时钟 */
    gpio_init_struct.GPIO_Pin = GPIO_PIN_SENSOR_RESET;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;
    gpio_init_struct.GPIO_PuPd= GPIO_PuPd_NOPULL;
    GPIO_Init(GPIO_PORT_SENSOR_RESET, &gpio_init_struct);
    
}

void sensor_read(void)
{
    /* sensor route */
    sensor.sensor_route[SENSOR_ROUTE1] = GPIO_ReadInputDataBit(GPIO_PORT_SENSOR_ROUTE,GPIO_PIN_SENSOR_ROUTE1);
    sensor.sensor_route[SENSOR_ROUTE2] = GPIO_ReadInputDataBit(GPIO_PORT_SENSOR_ROUTE,GPIO_PIN_SENSOR_ROUTE2);
    sensor.sensor_route[SENSOR_ROUTE3] = GPIO_ReadInputDataBit(GPIO_PORT_SENSOR_ROUTE,GPIO_PIN_SENSOR_ROUTE3);
    sensor.sensor_route[SENSOR_ROUTE4] = GPIO_ReadInputDataBit(GPIO_PORT_SENSOR_ROUTE,GPIO_PIN_SENSOR_ROUTE4);
    sensor.sensor_route[SENSOR_ROUTE5] = GPIO_ReadInputDataBit(GPIO_PORT_SENSOR_ROUTE,GPIO_PIN_SENSOR_ROUTE5);
    sensor.sensor_route[SENSOR_ROUTE6] = GPIO_ReadInputDataBit(GPIO_PORT_SENSOR_ROUTE,GPIO_PIN_SENSOR_ROUTE6);
    sensor.sensor_route[SENSOR_ROUTE7] = GPIO_ReadInputDataBit(GPIO_PORT_SENSOR_ROUTE,GPIO_PIN_SENSOR_ROUTE7);
    
    /* sensor postion1(行程开关) */
    sensor.sensor_pos1 = GPIO_ReadInputDataBit(GPIO_PORT_SENSOR_POS1,GPIO_PIN_SENSOR_POS1);
    
    /* sensor postion1(光电开关) */
    sensor.sensor_pos2 = GPIO_ReadInputDataBit(GPIO_PORT_SENSOR_POS2,GPIO_PIN_SENSOR_POS2);
    
    /* 行程开关-用于复位motor5和6 */
    sensor.sensor_reset = GPIO_ReadInputDataBit(GPIO_PORT_SENSOR_RESET,GPIO_PIN_SENSOR_RESET);
}

