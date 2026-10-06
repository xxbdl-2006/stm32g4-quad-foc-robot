/**
 * @file    motor.h
 * @brief   单台电机的完整对象：编码器 + 电流环 + 速度环 + 位置环 + 状态机 + 故障。
 *
 * 三环结构（全部在 20kHz 的同一个 ISR 里跑，外环按分频计数降频执行）：
 *
 *   position_ref ──►[位置P]──► velocity_ref ──►[速度PI]──► iq_ref ──►[电流PI]──► SVPWM
 *        ▲            1kHz                       1kHz                    20kHz
 *        └── encoder_get_mech_multi()
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "foc.h"
#include "pid.h"
#include "encoder.h"
#include "current_sense.h"
#include "foc_config.h"

typedef struct {
    uint8_t  id;                    /* 0..3 */

    /* ---- 子模块 ---- */
    foc_t            foc;
    encoder_t        enc;
    float            i_offset[3];   /* 本电机三相零电流偏置码值 */

    /* ---- 外环 ---- */
    pid_t  pid_velocity;
    pid_t  pid_position;

    /* ---- 模式与指令 ---- */
    motor_mode_t mode;
    motor_mode_t mode_pending;      /* 模式切换在环路边界生效，避免中途换环 */
    motor_state_t state;

    float position_ref;             /* rad，多圈 */
    float velocity_ref;             /* rad/s */
    float current_ref;              /* A，iq 指令（电流模式直接给） */
    float velocity_ff;              /* 速度前馈，用于位置模式提高跟随精度 */
    float current_limit;            /* 动态限流，可被上层按工况收紧 */
    float duty_openloop;            /* 开环模式下的占空比 -1..1 */

    /* ---- 反馈快照（供 CAN 组帧，保证原子性） ---- */
    float fb_position;
    float fb_velocity;
    float fb_current;
    float fb_duty;
    uint16_t fb_fault;

    /* ---- 分频计数 ---- */
    uint32_t tick;

    /* ---- 标定 ---- */
    bool  calib_requested;
    bool  calib_in_progress;
    uint32_t calib_tick;
    float calib_vd;

    /* ---- 保护 ---- */
    uint32_t overcurrent_cnt;
    uint32_t encoder_fault_cnt;
    bool     nfault_pin_low;
    bool     enabled_cmd;           /* 上位机给的使能位 */

    /* ---- 统计（上位机可通过 CLS_DIAG 读） ---- */
    uint32_t loop_overrun_cnt;
    float    iq_rms_acc;
    uint32_t iq_rms_n;
} motor_t;

/* 全局电机数组，定义在 motor.c */
extern motor_t g_motors[MOTOR_COUNT];

void motor_init_all(void);
void motor_init(motor_t *m, uint8_t id);
void motor_reset_fault(motor_t *m);

/**
 * @brief 20kHz 环路，逐台电机调用。必须由 TIM1_UP ISR 驱动。
 *        内部按 VELOCITY_LOOP_DIV / POSITION_LOOP_DIV 自行降频。
 */
void motor_fast_loop(motor_t *m);

/** @brief 1kHz 慢任务：故障判定、温度、统计汇总。由同一个 ISR 分频调用。 */
void motor_slow_task(motor_t *m);

/* ---- 上位机命令入口（CAN 层解析后调用） ---- */
void motor_set_mode(motor_t *m, motor_mode_t mode);
void motor_set_target(motor_t *m, float setpoint, float limit, uint8_t flags);
void motor_enable(motor_t *m, bool en);
void motor_start_calibration(motor_t *m);
void motor_emergency_stop(motor_t *m);

/* ---- 参数读写（供 CAN 参数服务调用） ---- */
bool  motor_param_read(motor_t *m, uint16_t param_id, float *out);
bool  motor_param_write(motor_t *m, uint16_t param_id, float val);
void  motor_save_encoder_offset(motor_t *m);

/** @brief 快照输出：把环路边界的一致值拷进 motor_t::fb_*。ISR 安全。 */
void motor_latch_feedback(motor_t *m);

#endif /* MOTOR_H */
