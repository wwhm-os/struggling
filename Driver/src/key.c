#include "key.h"


/**
 * @brief       ��ʼ��key1��key0 ���IO��, ��ʹ��ʱ��
 * @param       ��
 * @retval      ��
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);             /* ʹ��ʱ�� */
    
    gpio_init_struct.GPIO_Pin = GPIO_Pin_4;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;
    gpio_init_struct.GPIO_PuPd= GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &gpio_init_struct);
    
    gpio_init_struct.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOE, &gpio_init_struct);
    
}

/**
 * @brief       ��ʼ��key1��key0 ���IO��, ��ʹ��ʱ��
 * @param       uint32_t key_pin �������źţ���key.h������Ӧ�궨��
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