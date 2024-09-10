#ifndef     KEY_H
#define     KEY_H
#include "delay.h"

/***** 定义KEY引脚 *****/
#define KEYO_GPIO_PIN  GPIO_Pin_4
#define KEY1_GPIO_PIN  GPIO_Pin_3

/***** 定义KEY引脚 end *****/

void key_init(void);
uint8_t key_scan(uint32_t pin);

#endif
