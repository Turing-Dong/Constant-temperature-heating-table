#ifndef __KEY_H
#define __KEY_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* 引脚 定义 */

#define EN_GPIO_PORT                     GPIOB
#define EN_A_GPIO_PIN                    GPIO_PIN_14
#define EN_SW_GPIO_PIN                   GPIO_PIN_13
#define EN_B_GPIO_PIN                    GPIO_PIN_12

#define ENCODER_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PC口时钟使能 */

#define SEG_GPIO_PORT                	 GPIOA
#define SEG_GPIO_PIN                     GPIO_PIN_8
#define SEG_GPIO_CLK_ENABLE()            do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

/* PA口时钟使能 */

/******************************************************************************************/

#define EN_A        HAL_GPIO_ReadPin(EN_GPIO_PORT, EN_A_GPIO_PIN)
#define EN_B        HAL_GPIO_ReadPin(EN_GPIO_PORT, EN_B_GPIO_PIN)
#define EN_SW       HAL_GPIO_ReadPin(EN_GPIO_PORT, EN_SW_GPIO_PIN)

#define SEG_ADC0    HAL_GPIO_WritePin(SEG_GPIO_PORT, SEG_GPIO_PIN, GPIO_PIN_SET)
#define SEG_ADC1    HAL_GPIO_WritePin(SEG_GPIO_PORT, SEG_GPIO_PIN, GPIO_PIN_RESET)

#define EN_A_PRES    1
#define EN_B_PRES    2
#define EN_SW_PRES   3

void key_init(void);                /* 按键初始化函数 */
void seg_init(void);                /* 模拟开关初始化函数 */
//uint8_t key_scan(uint8_t mode);     /* 按键扫描函数 */
#endif


















