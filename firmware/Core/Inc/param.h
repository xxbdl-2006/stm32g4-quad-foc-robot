/**
 * @file    param.h
 * @brief   运行期可调参数集合 + Flash 持久化布局。
 *
 * 三类参数：
 *   1) 标定值（encoder offset / 电流偏置）—— 出厂标定一次，随板卡走，写在 Flash 独立扇区；
 *   2) 整定值（PID 增益、限幅）—— 现场可调，保存后写 Flash param 扇区；
 *   3) 保护阈值 —— 只允许上位机在线改，掉电不保存（安全考虑：异常阈值不应被持久化）。
 */
#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_config.h"

#define PARAM_MAGIC         0x4D435034U   /* "MCP4" */
#define PARAM_VERSION       3U
#define PARAM_FLASH_ADDR    0x0803E000UL  /* 16KB 扇区，G474 的 sector 3 */
#define PARAM_FLASH_SECTOR  FLASH_SECTOR_3

typedef struct {
    /* ---- 电机级，按 4 路展开 ---- */
    struct {
        float pid_vel_kp, pid_vel_ki, pid_vel_kd;
        float pid_pos_kp, pid_pos_ki, pid_pos_kd;
        float pid_cur_kp, pid_cur_ki;
        float limit_current_a;
        float limit_velocity;       /* rad/s */
        float limit_position_lo, limit_position_hi;
        float enc_elec_offset;      /* 电角度零点，标定得到 */
        float i_offset[3];          /* 电流偏置码值 */
        float encoder_direction;    /* +1 / -1 */
        float gear_ratio;
        float pole_pairs;
    } motor[MOTOR_COUNT];

    /* ---- 板级 ---- */
    struct {
        float vbus_ovp, vbus_uvp;
        float otp_mcu, otp_mos;
        float can_timeout_ms;
        float vbus_div_ratio;
    } board;

    /* ---- 气泵 ---- */
    struct {
        float pwm_freq_hz;
        float ramp_ms;
        float kp, ki;               /* 压力闭环 */
        float target_kpa;
    } pump;

    /* ---- 相机 ---- */
    struct {
        float trig_rate_hz;
        float trig_pulse_us;
        float enable;
        float sync_timeout_ms;
    } cam;
} hw_params_t;

extern hw_params_t g_params;

/** @brief 从 Flash 载入；校验失败则填充默认值并返回 false。 */
bool param_load(void);
/** @brief 把当前 g_params 整体写入 Flash（会短暂关中断 ~20ms）。 */
bool param_save(void);
/** @brief 恢复出厂默认（不写 Flash，需再调 param_save）。 */
void param_load_defaults(void);
/** @brief 把当前参数快照通过 CAN 的 CLS_DIAG 回传（上位机 ros2 param 同步用）。 */
void param_dump_summary(void);

/** @brief 板级参数读（IDX 不在 0..3 时走的通路）。 */
bool param_read_board(uint16_t param_id, float *out);
/** @brief 板级参数写。 */
bool param_write_board(uint16_t param_id, float val);

#endif /* PARAM_H */
