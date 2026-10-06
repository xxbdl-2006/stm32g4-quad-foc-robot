/**
 * @file    pump.h
 * @brief   气泵控制：PWM 软启动斜坡、压力闭环、堵转检测。
 *
 * 硬件：DRV8874 H 桥，IN1 接 TIM1_CH4（20kHz PWM），IN2 接地（单向驱动）。
 *       压力传感器 0-100kPa -> ADC5 规则组通道 1。
 *       电流检测走 DRV8874 的 IPROPI 引脚 -> ADC（用于堵转判断，无闭环）。
 *
 * 为什么不用简单的 GPIO 开关泵：隔膜泵启动电流是额定值的 6~8 倍，
 * 24V/5A 泵直启会把母线拉到欠压阈值以下，四路电机会同时报欠压。
 * 因此必须有 250ms 的 PWM 斜坡，把启动浪涌摊开。
 */
#ifndef PUMP_H
#define PUMP_H

#include <stdint.h>
#include <stdbool.h>
#include "can_proto.h"

typedef enum {
    PUMP_STATE_IDLE    = 0,
    PUMP_STATE_RAMPING = 1,
    PUMP_STATE_RUNNING = 2,
    PUMP_STATE_FAULT   = 3,
    PUMP_STATE_SOFTSTOP = 4
} pump_state_t;

#define PUMP_FAULT_NONE        0x00U
#define PUMP_FAULT_STALL       0x01U
#define PUMP_FAULT_SENSOR_OPEN 0x02U
#define PUMP_FAULT_OVERCURRENT 0x04U

void pump_init(void);
/** @brief 1kHz 调用：跑斜坡、压力环、堵转判断。 */
void pump_tick_1k(void);
/** @brief 处理上位机指令帧。ISR 上下文，只设标志与目标。 */
void pump_handle_cmd(const can_pump_cmd_t *cmd);
/** @brief 立即停机。@param soft true=按斜坡降速，false=立刻断 PWM。 */
void pump_stop(bool soft);
void pump_get_state(can_pump_state_t *out);
float pump_get_pressure_kpa(void);

#endif /* PUMP_H */
