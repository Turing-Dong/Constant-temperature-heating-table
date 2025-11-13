#ifndef __EXTI_H
#define __EXTI_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* 引脚 和 中断编号 & 中断服务函数 定义 */
#define SYS_NORMAL 1
#define SYS_WORKING 2

#define EN_INT_GPIO_PORT              	GPIOB
#define EN_A_INT_GPIO_PIN               GPIO_PIN_14
#define EN_SW_INT_GPIO_PIN              GPIO_PIN_13
#define EN_B_INT_GPIO_PIN               GPIO_PIN_12
#define EN_INT_GPIO_CLK_ENABLE()      	do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PC口时钟使能 */

#define EN_INT_IRQn                   	EXTI15_10_IRQn
#define EN_INT_IRQHandler             	EXTI15_10_IRQHandler
/******************************************************************************************/


void exti_init(void);  /* 外部中断初始化 */

#endif

























