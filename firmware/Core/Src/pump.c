/**
 * @file    pump.c
 * @brief   气泵控制实现。
 */
#include "pump.h"
#include "board.h"
#include "param.h"
#include "current_sense.h"
#include "foc_math.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  状态                                                              */
/* ------------------------------------------------------------------ */
static struct {
    pump_state_t state;
    uint8_t  fault;

    uint16_t duty_target;      /* 千分比目标 */
    uint16_t duty_current;     /* 千分比当前（斜坡中间值） */
    uint16_t duty_ramp_step;   /* 每 1ms 的步进量 */

    uint8_t  mode;             /* 0=开环占空比 1=压力闭环 */
    float    target_kpa;
    float    pressure_kpa;
    float    pressure_filt;

    uint32_t stall_timer_ms;
    uint32_t run_timer_ms;
    uint32_t softstop_timer_ms;

    float    pi_integ;
    bool     cmd_run;
} s_pump;

/* ------------------------------------------------------------------ */
/*  内部                                                              */
/* ------------------------------------------------------------------ */
static inline void pump_set_pwm(uint16_t permille)
{
    if (permille > PUMP_DUTY_MAX) { permille = PUMP_DUTY_MAX; }
    /* TIM1_CH4 占空比 = permille/1000 * ARR。
     * 注意 TIM1 是中心对齐，ARR 已由 CubeMX 配成 2399，与三相共用时基。 */
    const uint32_t ccr = ((uint32_t)(SVPWM_ARR + 1U) * permille) / 1000U;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr);
}

static void pump_apply_output(void)
{
    pump_set_pwm(s_pump.duty_current);
    HAL_GPIO_WritePin(PUMP_EN_PORT, PUMP_EN_PIN,
                      (s_pump.duty_current > 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @brief 压力采样：0-100kPa 传感器，4-20mA 版本已由板载 165Ω 转成电压。 */
static void pump_sample_pressure(void)
{
    const float raw = current_sense_get_pressure_raw();
    /* 0kPa -> 0.5V(620LSB)，100kPa -> 4.5V(5584LSB)，线性内插 */
    const float v = raw * (ADC_VREF_V / ADC_FULL_SCALE);
    float kpa = (v - 0.5f) / (4.5f - 0.5f) * PUMP_PRESSURE_FS_KPA;

    /* 传感器断线：电压 < 0.3V 视为开路 */
    if (v < 0.3f) {
        s_pump.fault |= PUMP_FAULT_SENSOR_OPEN;
        kpa = 0.0f;
    } else {
        s_pump.fault &= (uint8_t)~PUMP_FAULT_SENSOR_OPEN;
    }
    if (kpa < 0.0f) { kpa = 0.0f; }

    /* 一阶低通，泵的脉动频率约 50Hz（隔膜往复），滤波到 5Hz 足够 */
    s_pump.pressure_filt = 0.85f * s_pump.pressure_filt + 0.15f * kpa;
    s_pump.pressure_kpa = s_pump.pressure_filt;
}

/* ------------------------------------------------------------------ */
/*  公开接口                                                          */
/* ------------------------------------------------------------------ */
void pump_init(void)
{
    memset(&s_pump, 0, sizeof(s_pump));
    s_pump.state = PUMP_STATE_IDLE;
    s_pump.mode = 0;
    s_pump.duty_target = 0;
    s_pump.duty_current = 0;
    s_pump.target_kpa = g_params.pump.target_kpa;

    /* 斜坡步进：250ms 从 0 到 1000‰ -> 每 ms 走 4‰ */
    s_pump.duty_ramp_step = (uint16_t)(PUMP_DUTY_MAX /
                             ((PUMP_RAMP_MS > 0U) ? PUMP_RAMP_MS : 1U));

    /* 方向引脚固定为单向（IN2=0） */
    HAL_GPIO_WritePin(PUMP_DIR_PORT, PUMP_DIR_PIN, GPIO_PIN_RESET);
    pump_apply_output();
}

void pump_handle_cmd(const can_pump_cmd_t *cmd)
{
    s_pump.mode = cmd->mode;
    if (cmd->target_kpa10 > 0U) {
        s_pump.target_kpa = (float)cmd->target_kpa10 / 10.0f;
    }
    if (cmd->duty_permille > PUMP_DUTY_MAX) {
        s_pump.duty_target = PUMP_DUTY_MAX;
    } else {
        s_pump.duty_target = cmd->duty_permille;
    }

    switch (cmd->cmd) {
        case 0: pump_stop(false); break;
        case 2: pump_stop(true);  break;
        default:
            s_pump.cmd_run = true;
            if (s_pump.state == PUMP_STATE_IDLE || s_pump.state == PUMP_STATE_SOFTSTOP) {
                s_pump.state = PUMP_STATE_RAMPING;
                s_pump.stall_timer_ms = 0;
                s_pump.run_timer_ms = 0;
                s_pump.pi_integ = 0.0f;
            }
            break;
    }
}

void pump_stop(bool soft)
{
    s_pump.cmd_run = false;
    if (soft) {
        s_pump.state = PUMP_STATE_SOFTSTOP;
        s_pump.softstop_timer_ms = 0;
    } else {
        s_pump.duty_target = 0;
        s_pump.duty_current = 0;
        s_pump.pi_integ = 0.0f;
        s_pump.state = PUMP_STATE_IDLE;
        pump_apply_output();
    }
}

void pump_get_state(can_pump_state_t *out)
{
    out->state = (uint8_t)s_pump.state;
    out->duty_permille = s_pump.duty_current;
    out->pressure_kpa10 = (uint16_t)(s_pump.pressure_kpa * 10.0f);
    out->fault = s_pump.fault;
    out->reserved = 0;
}

float pump_get_pressure_kpa(void) { return s_pump.pressure_kpa; }

/* ------------------------------------------------------------------ */
/*  1kHz 任务                                                         */
/* ------------------------------------------------------------------ */
void pump_tick_1k(void)
{
    pump_sample_pressure();

    switch (s_pump.state) {
        case PUMP_STATE_RAMPING: {
            if (s_pump.duty_current + s_pump.duty_ramp_step >= s_pump.duty_target) {
                s_pump.duty_current = s_pump.duty_target;
                s_pump.state = PUMP_STATE_RUNNING;
                s_pump.run_timer_ms = 0;
            } else {
                s_pump.duty_current += s_pump.duty_ramp_step;
            }
            pump_apply_output();
            break;
        }

        case PUMP_STATE_RUNNING: {
            s_pump.run_timer_ms++;

            if (s_pump.mode == 1) {
                /* ---- 压力闭环：PI 作用在占空比上 ----
                 * 被控对象是"泵-管路-容腔"的一阶惯性，带宽很低（~2Hz），
                 * 因此 PI 增益必须很小，否则一定振荡。 */
                const float err = s_pump.target_kpa - s_pump.pressure_kpa;
                s_pump.pi_integ += g_params.pump.ki * err * 0.001f;

                float d = g_params.pump.kp * err + s_pump.pi_integ;
                /* 限幅 + 反算抗饱和 */
                const float d_max = (float)s_pump.duty_target;
                if (d > d_max) { d = d_max; s_pump.pi_integ -= g_params.pump.ki * err * 0.001f; }
                if (d < 0.0f)  { d = 0.0f;  s_pump.pi_integ -= g_params.pump.ki * err * 0.001f; }
                s_pump.duty_current = (uint16_t)d;
            }

            /* ---- 堵转/空转检测 ----
             * 指令占空比已给满，但压力长时间上不去 -> 管路脱落或泵膜破损；
             * 反过来若电流异常高且压力不涨 -> 堵转。两者都判为 STALL。 */
            if (s_pump.duty_current > (PUMP_DUTY_MAX * 9U / 10U) &&
                s_pump.pressure_kpa < 2.0f) {
                s_pump.stall_timer_ms++;
                if (s_pump.stall_timer_ms > PUMP_STALL_TIMEOUT_MS) {
                    s_pump.fault |= PUMP_FAULT_STALL;
                    s_pump.state = PUMP_STATE_FAULT;
                    pump_stop(false);
                }
            } else {
                s_pump.stall_timer_ms = 0;
            }

            /* ---- 保护母线：泵连续工作限 5 分钟，避免与电机抢电流 ---- */
            if (s_pump.run_timer_ms > 300000U) {
                pump_stop(true);
            }
            pump_apply_output();
            break;
        }

        case PUMP_STATE_SOFTSTOP: {
            s_pump.softstop_timer_ms++;
            if (s_pump.duty_current > s_pump.duty_ramp_step * 2U) {
                s_pump.duty_current -= (uint16_t)(s_pump.duty_ramp_step * 2U);
            } else {
                s_pump.duty_current = 0;
                s_pump.state = PUMP_STATE_IDLE;
            }
            pump_apply_output();
            break;
        }

        case PUMP_STATE_FAULT:
        case PUMP_STATE_IDLE:
        default:
            break;
    }
}
