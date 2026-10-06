/**
 * @file    encoder.c
 * @brief   AS5047P 采样与解算实现。
 *
 * AS5047P 帧格式（SPI mode 1，16bit）：
 *   bit15    : 读=0 / 写=1
 *   bit14    : 0 表示数据帧（1 为配置帧内部使用）
 *   bit13..0 : 14bit 角度（0..16383）
 *   bit15 回读 : 若不等于 0，视为帧不同步
 * AS5047P 的 SPI 有 ~2.5us 的 t_DV 延迟，因此读命令与数据帧是分开的两次传输，
 * 中间必须插一个 dummy 帧 —— 这是最常见的踩坑点，务必按下面的 exchange() 走。
 */
#include "encoder.h"
#include "foc_math.h"
#include "board.h"

/* =================================================================== */
/*  SPI 低层：与具体 CubeMX handle 解耦，便于在单测里替换              */
/* =================================================================== */
extern SPI_HandleTypeDef hspi1;

/* 每片 AS5047P 的流水线状态：首次读取必须丢弃（响应的是上电前的空帧）。 */
static bool enc_pipeline_primed[MOTOR_COUNT] = { false, false, false, false };

/* 片选由 board 层统一管理 —— 四片的 CS 分布在不同端口上，
 * 引脚细节不应该泄漏到编码器驱动里。 */
static inline void cs_low(uint8_t idx)  { board_encoder_cs(idx, true);  }
static inline void cs_high(uint8_t idx) { board_encoder_cs(idx, false); }

/** @brief 一次 16bit 全双工交换，超时按 ENCODER_SPI_TIMEOUT_MS。 */
static inline uint16_t spi_xfer16(uint16_t tx)
{
    uint16_t rx = 0;
    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)&rx, 1,
                                ENCODER_SPI_TIMEOUT_MS) != HAL_OK) {
        return 0xFFFFU;                     /* 错误哨兵：bit15=1 且数据全 1 */
    }
    return rx;
}

/**
 * @brief 流水线式读角度 —— 每次只用 1 帧 SPI（1.6us @10MHz）。
 *
 * AS5047P 是流水线器件：本次 MISO 上返回的是"上一次命令"的结果。因为我们要读的
 * 永远是同一个寄存器（ANGLECOM = 0x3FFF），命令是常量，所以可以直接把上一拍
 * 的响应当本拍角度用，省掉一整个 dummy 帧。
 *
 * 这一点非常关键：4 台电机 × 3 帧 = 19us 会吃掉 20kHz 环路 38% 的时间预算；
 * 换成 1 帧后降到 6.4us。代价是角度延迟比实际转子位置落后 50us（一个周期），
 * 在 573rpm 上限下对应 0.03 电周期，可忽略。
 */
static uint16_t as5047_read_angle_raw(uint8_t idx)
{
    cs_low(idx);
    /* 命令帧与数据帧合并：发出的同时接收上一拍的 ANGLECOM 结果 */
    uint16_t rx = spi_xfer16(0x4000U | 0x3FFFU);
    cs_high(idx);

    if (!enc_pipeline_primed[idx]) {
        /* 首次交换返回的是无意义数据，丢弃并再走一轮把流水线灌满 */
        enc_pipeline_primed[idx] = true;
        cs_low(idx);
        rx = spi_xfer16(0x4000U | 0x3FFFU);
        cs_high(idx);
    }
    return rx;
}

/** @brief 上电后为全部编码器灌满流水线（在电机使能之前调用一次）。 */
void encoder_prime_pipelines(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        enc_pipeline_primed[i] = false;
        (void)as5047_read_angle_raw(i);
    }
}

/* =================================================================== */
/*  公开接口                                                           */
/* =================================================================== */

void encoder_init(encoder_t *e, float elec_offset_rad)
{
    e->raw            = 0;
    e->raw_prev       = 0;
    e->turns          = 0;
    e->mech_rad       = 0.0f;
    e->mech_rad_multi = 0.0f;
    e->elec_offset    = elec_offset_rad;
    e->elec_rad       = 0.0f;
    e->mech_speed     = 0.0f;
    e->speed_lpf_alpha = 0.35f;
    e->inited         = false;
    e->align_done     = false;
    e->fault          = false;
    e->last_delta     = 0;
    e->parity_err_cnt = 0;
    e->crc_err_cnt    = 0;
    e->spi_timeout_cnt = 0;
}

bool encoder_sample(encoder_t *e, uint8_t motor_idx)
{
    if (motor_idx >= MOTOR_COUNT) { return false; }

    cs_low(motor_idx);
    uint16_t rx = as5047_read_angle_raw(motor_idx);
    cs_high(motor_idx);

    if (rx == 0xFFFFU) {                    /* SPI 超时 */
        e->spi_timeout_cnt++;
        e->fault = true;
        return false;
    }
    if ((rx & 0x4000U) != 0U) {             /* bit14=1 表示芯片返回错误帧 */
        e->crc_err_cnt++;
        e->fault = true;
        return false;
    }

    encoder_update(e, (uint16_t)(rx & ENCODER_MASK), true);
    return true;
}

void encoder_update(encoder_t *e, uint16_t raw, bool crc_ok)
{
    if (!crc_ok) { e->crc_err_cnt++; return; }
    if (!e->inited) {
        e->raw = raw; e->raw_prev = raw;
        e->mech_rad = (float)raw * ENCODER_RES_RAD;
        e->mech_rad_multi = e->mech_rad;
        e->elec_rad = wrap_2pi(e->mech_rad * (float)POLE_PAIRS - e->elec_offset);
        e->inited = true;
        return;
    }

    e->raw_prev = e->raw;
    e->raw = raw;

    /* ---- 多圈累加：用带符号的半量程差值判定跨圈方向 ---- */
    int32_t delta = (int32_t)raw - (int32_t)e->raw_prev;
    if (delta > (int32_t)(ENCODER_CPR / 2U)) {
        delta -= (int32_t)ENCODER_CPR;      /* 从 16383 -> 0 正跨圈 */
    } else if (delta < -(int32_t)(ENCODER_CPR / 2U)) {
        delta += (int32_t)ENCODER_CPR;      /* 反向跨圈 */
    }

    /* ---- 跳变检测：单拍角速度不可能超过 2rad/50us，超过必为 SPI 干扰 ---- */
    const float jump_rad = (float)(delta > 0 ? delta : -delta) * ENCODER_RES_RAD;
    if (jump_rad > ENCODER_MAX_JUMP_RAD) {
        e->crc_err_cnt++;
        e->fault = true;
        return;                             /* 丢弃本拍，保留上一拍状态 */
    }

    e->last_delta = delta;
    e->turns += (delta > 0) ? 1 : ((delta < 0) ? 0 : 0); /* 占位：turn 由累加角自然体现 */
    e->mech_rad_multi += (float)delta * ENCODER_RES_RAD;
    e->turns = (int32_t)(e->mech_rad_multi / TWO_PI);

    e->mech_rad = (float)raw * ENCODER_RES_RAD;
    e->elec_rad = wrap_2pi(e->mech_rad * (float)POLE_PAIRS - e->elec_offset);

    /* ---- 速度：由多圈角一阶差分，再一阶低通 ---- */
    const float w_raw = ((float)delta * ENCODER_RES_RAD) / DT_CURRENT;
    const float a = e->speed_lpf_alpha;
    e->mech_speed = a * e->mech_speed + (1.0f - a) * w_raw;
}

void encoder_set_zero_from_current(encoder_t *e)
{
    /* 当前机械角对应的电角度应被强制为 0：offset = mech*pp */
    e->elec_offset_enc = e->mech_rad;
    e->elec_offset = wrap_2pi(e->mech_rad * (float)POLE_PAIRS);
    e->elec_rad = 0.0f;
    e->align_done = true;
}

float encoder_get_elec(const encoder_t *e)      { return e->elec_rad; }
float encoder_get_mech_multi(const encoder_t *e) { return e->mech_rad_multi; }
float encoder_get_speed(const encoder_t *e)      { return e->mech_speed; }

void encoder_zero_speed_hint(encoder_t *e, bool is_commanded_stop)
{
    /* 量化噪声下差分速度的毛刺幅值约 CPR/2/dt*res ≈ 0.3 rad/s。
     * 指令停机且实测速度小于该阈值时直接钳零，让上位机的速度环不抖。 */
    if (is_commanded_stop && (e->mech_speed < 0.4f) && (e->mech_speed > -0.4f)) {
        e->mech_speed = 0.0f;
    }
}

void encoder_clear_fault(encoder_t *e)
{
    e->fault = false;
    e->crc_err_cnt = 0;
    e->spi_timeout_cnt = 0;
}
