#ifndef     KEY_H
#define     KEY_H
#include "delay.h"

/***** ����KEY���� *****/
#define KEYO_GPIO_PIN  GPIO_Pin_4
#define KEY1_GPIO_PIN  GPIO_Pin_3

/***** ����KEY���� end *****/

void key_init(void);
uint8_t key_scan(uint32_t pin);

#endif
