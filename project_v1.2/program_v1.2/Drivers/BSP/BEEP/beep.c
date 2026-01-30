#include "./BSP/BEEP/beep.h"

/**
 * @brief       初始化LED相关IO口, 并使能时钟
 * @param       无
 * @retval      无
 */
//void beep_init(void)
//{
//    GPIO_InitTypeDef gpio_init_struct;
//    BEEP_GPIO_CLK_ENABLE();                                 /* LED0时钟使能 */

//    gpio_init_struct.Pin = BEEP_GPIO_PIN;                   /* LED0引脚 */
//    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
//    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
//    
//    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
//    HAL_GPIO_Init(BEEP_GPIO_PORT, &gpio_init_struct);       /* 初始化LED0引脚 */

//    BEEP(0);                                                /* 关闭 LED0 */
//}
static TIM_HandleTypeDef htim4;
static volatile uint16_t beep_duration_ms = 0; // 剩余播放时间（ms）
static volatile uint8_t  beep_active = 0;       // 当前是否在响

// ✅ 初始化：配置 PB0 为推挽输出，TIM4 为 1ms 基础定时
void beep_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    // 🔊 蜂鸣器引脚：PB0（推挽输出，默认高电平关断）
    GPIO_InitTypeDef gpio_init_struct;
    BEEP_GPIO_CLK_ENABLE();                                 /* LED0时钟使能 */

    gpio_init_struct.Pin = BEEP_GPIO_PIN;                   /* LED0引脚 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
    
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
    HAL_GPIO_Init(BEEP_GPIO_PORT, &gpio_init_struct);       /* 初始化LED0引脚 */

    BEEP(0);                                                /* 关闭 LED0 */

    // ⏱️ TIM4：1ms 定时中断（72MHz / 72 = 1MHz → PSC=71；ARR=999 → 1000计数=1ms）
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 71;           // 72-1 = 71 → 1MHz 计数时钟
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 999;             // 1000 个计数 = 1ms
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim4);
    HAL_TIM_Base_Start_IT(&htim4);

    // 🔔 使能 TIM4 中断（优先级建议设为 2~3，低于编码器EXTI）
    HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

// 🎵 请求播放提示音（非阻塞！立即返回）
void Buzzer_Tone(uint16_t ms) {
    if (ms == 0) return;
    
    // ⚡ 原子操作：开启蜂鸣器 + 设置倒计时（即使被中断打断也安全）
    BEEP(1); // 立即响起
    beep_duration_ms = ms;
    beep_active = 1;
}

// 📣 TIM4 中断服务函数（每 1ms 进入一次）
void TIM4_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim4);
}

// 🧮 定时器溢出回调（由 HAL 自动生成，用于减计时）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        if (beep_active && beep_duration_ms > 0) {
            beep_duration_ms--;
        } else if (beep_active && beep_duration_ms == 0) {
            BEEP(0); // 关闭
            beep_active = 0;
        }
    }
}



