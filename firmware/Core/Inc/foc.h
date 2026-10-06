/**
 * @file    foc.h
 * @brief   单轴 FOC 内核：电流环 + 电压限幅 + 弱磁 + SVPWM 落点。
 *
 * 职责边界：本模块只管"给定 iq/id 指令 + 电角度 + 母线电压 -> 三相占空比"。
 * 不涉及编码器读取、模式切换、故障判断 —— 那些在 motor.c。
 * 这样切分是为了让 foc.c 可以脱离硬件、在 PC 上用同一份源码跑数值回归
 * （见 ros2_ws/src/hw_bringup/test/foc_golden_test）。
 */
#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_math.h"
#include "pid.h"
#include "foc_config.h"

typedef struct {
    /* ---- 指令 ---- */
    float id_ref;               /* d 轴电流指令，通常 0；弱磁时给负值 */
    float iq_ref;               /* q 轴电流指令，由外环给出 */
    float current_limit;        /* 矢量幅值上限，安培 */

    /* ---- 反馈 ---- */
    float ia, ib, ic;
    alphabeta_t i_ab;
    dq_t        i_dq;
    float       theta_e;        /* 电角度，rad */
    float       vbus;

    /* ---- 环 ---- */
    pid_t  pid_d;
    pid_t  pid_q;

    /* ---- 输出 ---- */
    dq_t        v_dq;
    alphabeta_t v_ab;
    pwm_duty_t  duty;

    /* ---- 弱磁 ---- */
    bool  field_weakening_en;
    float fw_id_min;            /* 弱磁允许的最大去磁电流，负值 */
    float fw_voltage_margin;    /* 触发弱磁的电压利用率阈值 0..1 */

    /* ---- 统计 ---- */
    float v_mag_last;
    uint32_t saturation_cnt;    /* 进入电压饱和的周期计数 */
} foc_t;

/** @brief 初始化电流环参数（按 PHASE_R/L 与环路带宽自动算初值）。 */
void foc_init(foc_t *f);
/** @brief 重置积分器，用于故障恢复或模式切换。 */
void foc_reset(foc_t *f);

/**
 * @brief 单个电流环周期。必须在 20kHz 的 TIM1_UP ISR 里调用，禁止阻塞。
 *
 * @param f        内核状态
 * @param theta_e  当前电角度（rad），由编码器给出
 * @param ia/ib/ic 三相电流（A）
 * @param vbus     母线电压（V）
 */
void foc_step(foc_t *f, float theta_e, float ia, float ib, float ic, float vbus);

/** @brief 弱磁：当 v_dq 幅值逼近母线极限时，按比例注入负 id 以压低反电动势。 */
void foc_field_weakening(foc_t *f);

/** @brief 设定电流矢量限幅（同时作用于 d/q 合成矢量，不是分别限幅）。 */
void foc_set_current_limit(foc_t *f, float amps);

/** @brief 只算 SVPWM 不开环 —— 用于电角度零点标定流程。 */
void foc_align_step(foc_t *f, float theta_e, float vd_align, float vbus);

/** @brief 按 R/L 推算 PI 初值：带宽取开关频率的 1/20，相位裕度 60°。 */
void foc_auto_tune_pi(foc_t *f, float r_ohm, float l_henry, float bandwidth_hz);

#endif /* FOC_H */
