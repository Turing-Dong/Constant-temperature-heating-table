#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./BSP/EXTI/exti.h"
#include "./BSP/BEEP/beep.h"
#include "./BSP/LCD/lcd.h"

extern u16 temp_set;
extern u8 SYS_STATE;
/**
 * @brief       外部中断服务程序
 * @param       无
 * @retval      无
 */
void EN_INT_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(EN_A_INT_GPIO_PIN | EN_SW_INT_GPIO_PIN);         /* 调用中断处理公用函数 清除KEY1所在中断线 的中断标志位 */
    __HAL_GPIO_EXTI_CLEAR_IT(EN_A_INT_GPIO_PIN | EN_SW_INT_GPIO_PIN);         /* HAL库默认先清中断再处理回调，退出时再清一次中断，避免按键抖动误触发 */
}

/**
 * @brief       外部中断服务程序
 * @param       无
 * @retval      无
 */
//void EN_SW_INT_IRQHandler(void)
//{ 
//    HAL_GPIO_EXTI_IRQHandler(EN_SW_INT_GPIO_PIN);        /* 调用中断处理公用函数 清除KEY1所在中断线 的中断标志位，中断下半部在HAL_GPIO_EXTI_Callback执行 */
//    __HAL_GPIO_EXTI_CLEAR_IT(EN_SW_INT_GPIO_PIN);        /* HAL库默认先清中断再处理回调，退出时再清一次中断，避免按键抖动误触发 */
//}

/**
 * @brief       中断服务程序中需要做的事情
                在HAL库中所有的外部中断服务函数都会调用此函数
 * @param       GPIO_Pin:中断引脚号
 * @retval      无
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BEEP(1);
    delay_ms(5);               /* 消抖 */
//    switch(GPIO_Pin)
//    {
//        case EN_A_INT_GPIO_PIN:
				if(HAL_GPIO_ReadPin(EN_GPIO_PORT, EN_A_GPIO_PIN) == 0)
				{
						if(HAL_GPIO_ReadPin(EN_GPIO_PORT, EN_B_GPIO_PIN) == 0)
						{
								if(++temp_set==250)
									temp_set = 250;
						}
						else if(HAL_GPIO_ReadPin(EN_GPIO_PORT, EN_B_GPIO_PIN) == 1)
						{
							if(--temp_set==0)
									temp_set = 0;
						}
				}
//						break;
//        case EN_SW_INT_GPIO_PIN:
				else if(HAL_GPIO_ReadPin(EN_GPIO_PORT, EN_SW_GPIO_PIN) == 0)
				{
            if(SYS_STATE == SYS_NORMAL)
            {
                SYS_STATE = SYS_WORKING;
            }
            else
            {
                SYS_STATE = SYS_NORMAL;
            }
				}
//						break;
//        default:
//						break;
//    }
	BEEP(0);
}

/**
 * @brief       外部中断初始化程序
 * @param       无
 * @retval      无
 */
void exti_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    EN_INT_GPIO_CLK_ENABLE();                                 /*时钟使能 */

    gpio_init_struct.Pin = EN_A_INT_GPIO_PIN | EN_SW_INT_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;            	/*下升沿触发 */
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(EN_GPIO_PORT, &gpio_init_struct);    				/*配置为下降沿触发中断 */
    
    HAL_NVIC_SetPriority(EN_INT_IRQn, 2, 2);              	 	/*抢占0，子优先级2 */
    HAL_NVIC_EnableIRQ(EN_INT_IRQn);                       		/*使能中断线1 */
		//EN_INT_IRQn		EXTI15_10_IRQn
}
