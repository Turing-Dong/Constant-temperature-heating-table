#ifndef _BEEP_H
#define _BEEP_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 引脚 定义 */

#define BEEP_GPIO_PORT                  GPIOA
#define BEEP_GPIO_PIN                   GPIO_PIN_6
#define BEEP_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)             /* PA口时钟使能 */

/******************************************************************************************/
/* LED端口定义 */
#define BEEP(x)   do{ x ? \
                      HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN, GPIO_PIN_RESET);\
                  }while(0)      /* LED0翻转 */

/* LED取反定义 */
#define BEEP_TOGGLE()   do{ HAL_GPIO_TogglePin(BEEP_GPIO_PORT, BEEP_GPIO_PIN); }while(0)        /* 翻转LED0 */

/******************************************************************************************/
/* 外部接口函数*/
void beep_init(void);                                                                            /* 初始化 */

#endif


