#include "./BSP/FAN/fan.h"

/**
 * @brief       初始化FAN相关IO口, 并使能时钟
 * @param       无
 * @retval      无
 */
void fan_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    FAN_GPIO_CLK_ENABLE();                                 /* FAN时钟使能 */

    gpio_init_struct.Pin = FAN_GPIO_PIN;                   /* FAN引脚 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
    
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
    HAL_GPIO_Init(FAN_GPIO_PORT, &gpio_init_struct);       /* 初始化FAN引脚 */

    FAN(0);                                                /* 关闭 FAN */
}



