#include "key.h"


/**
 * @brief       初始化key1与key0 相关IO口, 并使能时钟
 * @param       无
 * @retval      无
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);             /* 使能时钟 */
    
    gpio_init_struct.GPIO_Pin = GPIO_Pin_4;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;
    gpio_init_struct.GPIO_PuPd= GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &gpio_init_struct);
    
    gpio_init_struct.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOE, &gpio_init_struct);
    
}

/**
 * @brief       初始化key1与key0 相关IO口, 并使能时钟
 * @param       uint32_t key_pin 按键引脚号，在key.h中有相应宏定义
 * @retval      
 */
uint8_t key_scan(uint32_t key_pin)
{
    if(GPIO_ReadInputDataBit(GPIOE,key_pin) == 0)
    {
        delay_us(10*1000);      /* delay for debounce */
        if(GPIO_ReadInputDataBit(GPIOE,key_pin) == 0)
        {
            while(GPIO_ReadInputDataBit(GPIOE,key_pin) == 0);     /* wait for releasing the button */
            return 1;      /* press the key */
        }      
    }
    return 0;      /* NOT press the key */
}