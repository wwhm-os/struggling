#include "my_led.h"

GPIO_TypeDef *MY_LED_GPIO_PORT[LEDn]={
								LED1_GPIO_PORT,
								LED2_GPIO_PORT,
								};
const uint16_t MY_LED_PIN[LEDn]={	
								LED1_PIN,
								LED2_PIN,
								};
const uint32_t MY_LED_GPIO_CLK[LEDn]={	
								LED1_GPIO_CLK,
								LED2_GPIO_CLK,
								};
void MY_LED_Init(LED_TypeDef LED)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(MY_LED_GPIO_CLK[LED],ENABLE);

	GPIO_InitStructure.GPIO_Pin = MY_LED_PIN[LED];
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(MY_LED_GPIO_PORT[LED],&GPIO_InitStructure);
	GPIO_SetBits(MY_LED_GPIO_PORT[LED],MY_LED_PIN [LED]);
}

void MY_LED_InitAll(void)
{
    MY_LED_Init(LED1);
    MY_LED_Init(LED2);
}

void MY_LED_On(LED_TypeDef LED)
{
	GPIO_ResetBits(MY_LED_GPIO_PORT[LED],MY_LED_PIN [LED]);
}

void MY_LED_Off(LED_TypeDef LED)
{
	GPIO_SetBits(MY_LED_GPIO_PORT[LED],MY_LED_PIN [LED]);
}

