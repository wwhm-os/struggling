#ifndef __MY_LED_H
#define __MY_LED_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"



/* Exported types ------------------------------------------------------------*/
typedef enum
{
	LED1 ,
	LED2 
	
}LED_TypeDef;
/* Exported constants --------------------------------------------------------*/
#define LEDn												    		2
//                                                         
#define LED1_PIN												    GPIO_Pin_9            
#define LED1_GPIO_PORT									GPIOF
#define LED1_GPIO_CLK										RCC_AHB1Periph_GPIOF

#define LED2_PIN											    	GPIO_Pin_10                  
#define LED2_GPIO_PORT									GPIOF
#define LED2_GPIO_CLK										RCC_AHB1Periph_GPIOF


/* Exported functions --------------------------------------------------------*/
void MY_LED_Init(LED_TypeDef LED);
void MY_LED_On(LED_TypeDef LED);
void MY_LED_Off(LED_TypeDef LED);
void MY_LED_Toggle(LED_TypeDef LED);
void MY_LED_InitAll(void);

#endif


