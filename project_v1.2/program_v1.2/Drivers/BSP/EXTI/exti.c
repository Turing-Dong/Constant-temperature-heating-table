#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/ENCODER/encoder.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./BSP/EXTI/exti.h"
#include "./BSP/BEEP/beep.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/GLOBALS/globals.h"

// 统一回调（根据 Pin 区分）
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == EN_A_GPIO_PIN || GPIO_Pin == EN_B_GPIO_PIN) {
        // ⚡ 极快！只读2个IO，计算格雷码状态，喂给状态机
        uint8_t a = HAL_GPIO_ReadPin(EN_GPIO_PORT, EN_A_GPIO_PIN);
        uint8_t b = HAL_GPIO_ReadPin(EN_GPIO_PORT, EN_B_GPIO_PIN);
        encoder_update(a, b); // 你封装好的状态机函数（见下文）
    }
    else if (GPIO_Pin == EN_SW_GPIO_PIN) {
        // 🔘 按键：仅置位标志，消抖交给主循环（更可靠！）
        sw_press = 1; // 主循环检测到后，再延时20ms验证
    }
}

/**
 * @brief       外部中断初始化程序
 * @param       无
 * @retval      无
 */
void exti_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
	__HAL_RCC_AFIO_CLK_ENABLE(); 
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // 配置 PB5 为 EN_A 的外部中断
    gpio_init_struct.Pin = GPIO_PIN_5;
    gpio_init_struct.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    // 配置 PB10 为 EN_SW 的外部中断
    gpio_init_struct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    // 配置 PB11 为 EN_B 的外部中断
    gpio_init_struct.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    // 配置 AFIO 复用功能，将 PB5/PB10/PB11 映射到对应 EXTI 线
    AFIO->EXTICR[1] |= 0x00010000; // EXTI5 (PB5) 映射到 EXTICR1[15:12]
    AFIO->EXTICR[2] |= 0x00000001; // EXTI10 (PB10) 映射到 EXTICR0[3:0]
    AFIO->EXTICR[2] |= 0x00000010; // EXTI11 (PB11) 映射到 EXTICR0[7:4]

    // NVIC 配置
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);     // PB5 的 EXTI5 中断优先级
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 1);   // PB10/PB11 的 EXTI10/11 中断优先级
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
