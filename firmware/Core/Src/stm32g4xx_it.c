/**
 * @file    stm32g4xx_it.c
 * @brief   中断向量实现。
 *
 * 优先级规划（数字越小优先级越高，NVIC 4bit 抢占）：
 *   0 : SysTick（HAL 时基）
 *   1 : TIM1_UP    —— 20kHz 控制环。这一条必须最高，抖动直接影响电流环稳定性
 *   2 : FDCAN1_IT0 —— CAN 收发。比控制环低，保证控制不被 CAN 打断
 *   3 : EXTI15_10  —— 相机帧同步。相机丢一个沿只是丢一帧图，不影响安全
 *   4 : DMA1_CH1   —— ADC5 循环
 *   6 : USART1     —— 调试串口（可选）
 *
 * 特别说明：TIM1_UP 里调用了 FDCAN 的发送函数（向 TX FIFO 写），
 * 但 FDCAN 中断优先级更低，所以不存在重入问题；反过来 FDCAN 回调里
 * 调用 motor_* 系列函数时，若此刻控制环正在跑，也不会被打断。
 */
#include "stm32g4xx_hal.h"
#include "board.h"   /* CAM_SYNC_PIN / htim1 / hfdcan1 */

void NMI_Handler(void)        { for (;;) { } }
void HardFault_Handler(void)  { for (;;) { } }
void MemManage_Handler(void)  { for (;;) { } }
void BusFault_Handler(void)   { for (;;) { } }
void UsageFault_Handler(void) { for (;;) { } }

void SVC_Handler(void)        { }
void DebugMon_Handler(void)   { }
void PendSV_Handler(void)     { }

extern TIM_HandleTypeDef htim1;
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

extern FDCAN_HandleTypeDef hfdcan1;
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

extern DMA_HandleTypeDef hdma_adc5;
void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc5);
}

/* 相机帧同步（PB13，属 EXTI15_10 组）。
 * 这里用 CAM_SYNC_PIN 而不是硬编码位号 —— 换板改引脚时只改 board.h 一处。 */
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(CAM_SYNC_PIN);
}

/* 其余 EXTI 分组（如果后续把 nFAULT 也接成中断） */
void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(DRV_FAULT0_PIN);
}
