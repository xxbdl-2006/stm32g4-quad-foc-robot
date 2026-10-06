/**
 * @file    encoder.h
 * @brief   AS5047P 磁编（14bit 绝对式）读取、多圈累加、机械/电角度换算。
 *
 * 硬件：4 片 AS5047P 共享 SPI1 SCK/MISO/MOSI，各自独立 CS（PA4/PB0/PB1/PB2）。
 * 时序：每片在 20kHz 电流环里被读一次 → 4 片共用一条 10MHz SPI，
 *       单帧 16bit ≈ 1.6us，4 片 6.4us，占一个 50us 周期 13%，可接受。
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_config.h"

typedef struct {
    /* --- 原始 --- */
    uint16_t raw;               /* 本拍 14bit 原始码 */
    uint16_t raw_prev;
    uint16_t parity_err_cnt;
    uint16_t crc_err_cnt;
    uint32_t spi_timeout_cnt;

    /* --- 解算 --- */
    int32_t  turns;             /* 多圈计数（带符号） */
    float    mech_rad;          /* 单圈机械角 [0,2π) */
    float    mech_rad_multi;    /* 多圈机械角，单调累加 */
    float    elec_rad;          /* 电角度 [0,2π) */
    float    elec_offset;       /* 电角度零点，标定后写 Flash */
    float    elec_offset_enc;   /* 标定得到的编码器角（存储用） */

    /* --- 速度估计 --- */
    float    mech_speed;        /* rad/s，一阶差分 + 低通 */
    float    speed_lpf_alpha;

    /* --- 状态 --- */
    bool     inited;
    bool     align_done;
    bool     fault;
    int32_t  last_delta;        /* 上一拍多圈增量（码），用于跳变检测 */
} encoder_t;

void encoder_init(encoder_t *e, float elec_offset_rad);
/** @brief 上电一次性灌满 4 片 AS5047P 的 SPI 流水线（在使能电机前调用）。 */
void encoder_prime_pipelines(void);
/** @brief 纯解算：把 raw 喂进来，刷新全部派生量。ISR 上下文，无阻塞。 */
void encoder_update(encoder_t *e, uint16_t raw, bool crc_ok);
/** @brief 读 SPI 并更新，内部处理 CS 与超时。返回是否成功。 */
bool encoder_sample(encoder_t *e, uint8_t motor_idx);
/** @brief 把当前编码器角作为电角度零点（对齐流程调用）。 */
void encoder_set_zero_from_current(encoder_t *e);
float encoder_get_elec(const encoder_t *e);
float encoder_get_mech_multi(const encoder_t *e);
float encoder_get_speed(const encoder_t *e);
/** @brief 零速注入：电机静止时速度估计会因量化噪声输出 ±0.3rad/s 抖动，此处强拉零。 */
void encoder_zero_speed_hint(encoder_t *e, bool is_commanded_stop);
void encoder_clear_fault(encoder_t *e);

#endif /* ENCODER_H */
