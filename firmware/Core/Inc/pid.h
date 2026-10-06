/**
 * @file    pid.h
 * @brief   抗积分饱和的并联式 PID，含前馈与微分低通。三个环（电流/速度/位置）共用。
 *
 * 说明：电流环 PI 使用"反算抗饱和"（back-calculation），速度环使用"条件积分"，
 * 位置环额外启用微分先行的低通。这些差异由 pid_config_t 的字段开关控制，
 * 而不是写三个类 —— 嵌入式里少一层抽象就少一层调参心智负担。
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float out_min;          /* 输出下限（含符号，例如 -15A） */
    float out_max;          /* 输出上限 */

    float dt;               /* 采样周期，秒 */

    /* 抗饱和 */
    bool  use_backcalc;     /* true: 反算抗饱和；false: 条件积分 */
    float kaw;              /* 反算增益，通常取 1/ki 量级 */

    /* 微分通道 */
    bool  d_on_measurement; /* true: 微分作用在测量值上（避免设定值阶跃冲击） */
    float d_lpf_alpha;      /* 微分一阶低通系数 0..1，0 表示不滤波 */
} pid_config_t;

typedef struct {
    pid_config_t cfg;

    float ref;              /* 设定值 */
    float fdb;              /* 反馈值 */
    float err;              /* 当前误差 */
    float err_prev;

    float integ;            /* 积分累加器 */
    float d_state;          /* 微分低通状态 */
    float fdb_prev;

    float out;              /* 输出 */
    float out_unsat;        /* 饱和前输出，供反算使用 */

    bool  enabled;
} pid_t;

void  pid_init(pid_t *p, const pid_config_t *cfg);
void  pid_reset(pid_t *p);
/** @brief 单步计算。ref/fdb 每周期刷新，内部完成全部状态更新。 */
float pid_step(pid_t *p, float ref, float fdb);
/** @brief 带前馈的版本：out = pid + ff（前馈不参与积分，绕过饱和限制参与反算）。 */
float pid_step_ff(pid_t *p, float ref, float fdb, float ff);
/** @brief 冻结积分（例如电机进入 FAULT 时调用，保留状态便于复位后平滑恢复）。 */
void  pid_freeze(pid_t *p);
void  pid_set_gains(pid_t *p, float kp, float ki, float kd);

#endif /* PID_H */
