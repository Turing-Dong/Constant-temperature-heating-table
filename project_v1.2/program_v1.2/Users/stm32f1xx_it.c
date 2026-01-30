/**
  ******************************************************************************
  * @file    Templates/Src/stm32f1xx.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal.h"

#include "./SYSTEM/usart/usart.h"
#include "./BSP/GLOBALS/globals.h"
#include "./BSP/ENCODER/encoder.h"

__attribute__((used)) uint32_t hardfault_cfsr = 0;
__attribute__((used)) uint32_t hardfault_hfsr = 0;
__attribute__((used)) uint32_t hardfault_bfar = 0;
__attribute__((used)) uint32_t hardfault_mmfar = 0;

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
		// ? 1. 读取内核故障寄存器（CMSIS 标准定义，Keil/GCC/IAR 全支持）
    hardfault_cfsr = SCB->CFSR;        // Configurable Fault Status Register
    hardfault_hfsr = SCB->HFSR;        // HardFault Status Register
    hardfault_bfar = SCB->BFAR;        // BusFault Address Register
    hardfault_mmfar = SCB->MMFAR;      // MemManage Address Register

    // ? 2. 打印诊断信息（需确保串口已初始化！）
    printf("\r\n*** HARDFAULT DETECTED ***\r\n");
    printf("CFSR = 0x%08lX\r\n", (long)hardfault_cfsr);
    printf("HFSR = 0x%08lX\r\n", (long)hardfault_hfsr);

    // ? 3. 解析具体错误类型（Keil 用户最需要的！）
    if (hardfault_cfsr & 0x00000100) { // MEMMANAGEACTIVE bit
        printf("→ Memory Management Fault at 0x%08lX\r\n", (long)hardfault_mmfar);
    }
    if (hardfault_cfsr & 0x00000200) { // BUSFAULTACTIVE bit
        printf("→ Bus Fault at 0x%08lX\r\n", (long)hardfault_bfar);
    }
    if (hardfault_cfsr & 0x00000400) { // USAGEFAULTACTIVE bit
        printf("→ Usage Fault (e.g., invalid instruction, unaligned access)\r\n");
    }
    if (hardfault_hfsr & 0x40000000) { // FORCED bit → 表示是上述三类之一触发的 HardFault
        printf("→ Forced HardFault (due to MemManage/BUS/Usage fault)\r\n");
    }

    // ? 4. 主动停机（LED 报警）
    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
        HAL_Delay(200);
    }
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

// EXTI5（PB5）和 EXTI9_5 共享 IRQ：处理 PB5（EN_A）
void EXTI9_5_IRQHandler(void) {
    // 检查是否是 PB5 触发（EXTI5）
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET) {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5); // 手动清除（HAL 可能未清）
    }
}

// EXTI10/11（PB10/PB11）共享 IRQ：处理按键和 EN_B
void EXTI15_10_IRQHandler(void) {
    // 检查 PB10（按键）
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET) {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
    }
    // 检查 PB11（EN_B）
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET) {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
    }
}

/******************************************************************************/
/*                 STM32F1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f1xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
