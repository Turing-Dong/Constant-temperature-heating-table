#ifndef _FAN_H
#define _FAN_H
#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 引脚 定义 */

#define FAN_GPIO_PORT                  GPIOA
#define FAN_GPIO_PIN                   GPIO_PIN_7
#define FAN_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)             /* PA口时钟使能 */

/******************************************************************************************/
/* BEEP端口定义 */
#define FAN(x)   do{ x ? \
                      HAL_GPIO_WritePin(FAN_GPIO_PORT, FAN_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(FAN_GPIO_PORT, FAN_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)      /* LED0翻转 */

/* BEEP取反定义 */
#define FAN_TOGGLE()   do{ HAL_GPIO_TogglePin(FAN_GPIO_PORT, FAN_GPIO_PIN); }while(0)        /* 翻转BEEP */

/******************************************************************************************/
/* 外部接口函数*/
void fan_init(void);                                                                            /* 初始化 */

#endif
