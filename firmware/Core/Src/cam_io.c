/**
 * @file    cam_io.c
 *
 * 定时器分配：TIM15 是 16bit 定时器，挂在 APB2（170MHz）。之所以不用 TIM1 的剩余通道：
 * TIM1 已经在跑 20kHz 中心对齐的三相 PWM，改它的 ARR 会同时改掉电流环时基。
 * TIM15 独立，可以自由设 ARR；缺点是只有 16bit，因此用 1MHz 计数频率时
 * 最慢只能到 1MHz/65535 ≈ 15Hz —— 恰好覆盖本项目 1~120Hz 的需求下沿。
 * 若要支持更低的触发频率（<15Hz），需要改成分频 + 软件计数的方式。
 */
#include "cam_io.h"
#include "foc_math.h"   /* clampf / wrap_2pi 等内联数学 */
#include "board.h"
#include "motor.h"
#include "can_node.h"
#include "param.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  DWT 周期计数器（Cortex-M4 的系统周期计数，1 周期 ≈ 5.88ns @170MHz） */
/* ------------------------------------------------------------------ */
static inline void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t dwt_now(void) { return DWT->CYCCNT; }
static inline uint32_t dwt_diff_us(uint32_t now, uint32_t prev)
{
    return (uint32_t)((float)(now - prev) / (SystemCoreClock / 1000000.0f));
}

/* ------------------------------------------------------------------ */
/*  状态                                                              */
/* ------------------------------------------------------------------ */
static struct {
    volatile uint32_t frame_id;
    volatile uint32_t last_cyc;
    volatile uint32_t period_us;
    volatile uint16_t dropped_window;
    volatile uint32_t cumulative_dropped;

    volatile bool     latch_pending;
    cam_latch_t       latch;         /* 双缓冲：ISR 写 A，20k 读 B */
    cam_latch_t       latch_shadow;

    bool     trig_enabled;
    uint16_t trig_rate_hz;
    uint16_t trig_pulse_us;

    uint32_t last_sync_tick_1k;      /* 1kHz 时基，超时判定用 */
    uint32_t tick_1k;
    bool     online;

    uint8_t  single_shot_left;       /* 单次触发剩余脉冲数 */
} s_cam;

/* ------------------------------------------------------------------ */
/*  配置                                                              */
/* ------------------------------------------------------------------ */
void cam_io_init(void)
{
    memset(&s_cam, 0, sizeof(s_cam));
    dwt_init();

    s_cam.trig_rate_hz  = (uint16_t)g_params.cam.trig_rate_hz;
    s_cam.trig_pulse_us = (uint16_t)g_params.cam.trig_pulse_us;
    s_cam.online = false;

    if (g_params.cam.enable > 0.5f) {
        cam_trigger_enable(s_cam.trig_rate_hz, s_cam.trig_pulse_us);
    } else {
        cam_trigger_disable();
    }
}

void cam_trigger_enable(uint16_t rate_hz, uint16_t pulse_us)
{
    if (rate_hz < CAM_TRIG_RATE_HZ_MIN) { rate_hz = CAM_TRIG_RATE_HZ_MIN; }
    if (rate_hz > CAM_TRIG_RATE_HZ_MAX) { rate_hz = CAM_TRIG_RATE_HZ_MAX; }
    if (pulse_us < CAM_TRIG_PULSE_US_MIN) { pulse_us = CAM_TRIG_PULSE_US_MIN; }
    if (pulse_us > CAM_TRIG_PULSE_US_MAX) { pulse_us = CAM_TRIG_PULSE_US_MAX; }

    s_cam.trig_rate_hz = rate_hz;
    s_cam.trig_pulse_us = pulse_us;

    /* TIM15 时钟 170MHz，双缓冲预分频：
     *   prescaler 固定 170-1 -> 计数频率 1MHz，1 计数 = 1us
     *   ARR = 1e6 / rate_hz - 1，CCR1 = pulse_us
     * 这样脉宽分辨率恒为 1us，与触发频率无关，最慢 1Hz 时 ARR=999999 仍在 16bit 外，
     * 因此 CAM_TRIG_RATE_HZ_MIN 被设为 1：低于 15Hz 的触发需要另想办法。 */
    __HAL_TIM_SET_PRESCALER(&htim15, 170U - 1U);
    __HAL_TIM_SET_AUTORELOAD(&htim15, (uint32_t)(1000000U / rate_hz) - 1U);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, (uint32_t)pulse_us);

    s_cam.single_shot_left = 0;
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
    s_cam.trig_enabled = true;
}

void cam_trigger_disable(void)
{
    HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
    /* 拉低输出，避免相机把悬空引脚上的毛刺当成触发 */
    HAL_GPIO_WritePin(CAM_TRIG_PORT, CAM_TRIG_PIN, GPIO_PIN_RESET);
    s_cam.trig_enabled = false;
}

void cam_single_shot(void)
{
    /* 单次触发：把 ARR 设成一个很小的一次性值 —— 用 One-Pulse 模式，
     * 硬件在产生一个完整脉冲后自动停止，不需要软件掐表。 */
    cam_trigger_enable(s_cam.trig_rate_hz, s_cam.trig_pulse_us);
    __HAL_TIM_SET_AUTORELOAD(&htim15, (uint32_t)s_cam.trig_pulse_us * 2U);
    HAL_TIM_OnePulse_Start(&htim15, TIM_CHANNEL_1);
}

void cam_handle_cfg(const can_cam_cfg_t *cfg)
{
    if (cfg->enable == 0U) {
        cam_trigger_disable();
        g_params.cam.enable = 0.0f;
        return;
    }
    if (cfg->pulse_us > 0U) { s_cam.trig_pulse_us = cfg->pulse_us; }
    if (cfg->rate_hz > 0U)  { s_cam.trig_rate_hz  = cfg->rate_hz; }

    switch (cfg->mode) {
        case 1: cam_single_shot(); break;
        case 2: /* 外部触发透传：不输出，只计数 SYNC */
            cam_trigger_disable();
            s_cam.trig_enabled = true;   /* 逻辑上"在跑"，但不产生脉冲 */
            break;
        default:
            cam_trigger_enable(s_cam.trig_rate_hz, s_cam.trig_pulse_us);
            break;
    }
    g_params.cam.enable = 1.0f;
    g_params.cam.trig_rate_hz = (float)s_cam.trig_rate_hz;
    g_params.cam.trig_pulse_us = (float)s_cam.trig_pulse_us;
}

/* ------------------------------------------------------------------ */
/*  SYNC 中断（PB11 上升沿）                                          */
/* ------------------------------------------------------------------ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != CAM_SYNC_PIN) { return; }

    const uint32_t now = dwt_now();
    const uint32_t dt_us = (s_cam.last_cyc == 0U) ? 0U : dwt_diff_us(now, s_cam.last_cyc);
    s_cam.last_cyc = now;

    /* ---- 丢帧判定：标称周期 T，实测 > 1.6T 即判为丢帧（留 60% 抖动余量） ---- */
    if (dt_us > 0U) {
        const uint32_t nominal = 1000000U / (s_cam.trig_rate_hz ? s_cam.trig_rate_hz : 1U);
        if (dt_us > (nominal * 16U / 10U)) {
            const uint32_t lost = (dt_us + nominal / 2U) / nominal;
            if (lost > 1U) {
                s_cam.dropped_window += (uint16_t)(lost - 1U);
                s_cam.cumulative_dropped += (lost - 1U);
            }
        }
    }
    s_cam.period_us = dt_us;
    s_cam.frame_id++;
    s_cam.online = true;

    /* ---- 关键：立刻锁存四轴位置 ----
     * float 在 Cortex-M4 上是单周期对齐写/读，不存在撕裂读，
     * 因此无需关中断。这里读的是 motor_latch_feedback() 刚写好的 fb_position，
     * 与被 SYNC 打断的控制周期最多相差 50us（一个 PWM 周期）。 */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        s_cam.latch.pos[i] = g_motors[i].fb_position;
        s_cam.latch.vel[i] = g_motors[i].fb_velocity;
    }
    s_cam.latch.frame_id = s_cam.frame_id;
    s_cam.latch.t_capture_cyc = now;
    s_cam.latch_pending = true;
}

/* ------------------------------------------------------------------ */
/*  20kHz 环路任务                                                    */
/* ------------------------------------------------------------------ */
void cam_io_tick_20k(void)
{
    /* 把 EXTI 里锁存的结果搬到影子缓冲，再发 CAN。
     * 之所以要搬到影子：latch 会被下一个 SYNC 覆盖，而 CAN 发送可能因为
     * TX FIFO 满而延后 —— 直接发会读到被改写的帧号，上位机就会把
     * 位置和图对错。 */
    if (s_cam.latch_pending) {
        s_cam.latch_pending = false;

        memcpy(&s_cam.latch_shadow, (const void *)&s_cam.latch, sizeof(cam_latch_t));

        can_node_send_cam_sync(s_cam.latch_shadow.frame_id,
                               (uint16_t)(s_cam.period_us > 65535U ? 65535U : s_cam.period_us),
                               s_cam.dropped_window);

        /* 4 帧位置锁存，跟着 SYNC 帧后面发 */
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            can_node_send_cam_latch(i,
                                    (int32_t)(s_cam.latch_shadow.pos[i] * 1000.0f),
                                    clamp_i16((int32_t)(s_cam.latch_shadow.vel[i] * 1000.0f)),
                                    s_cam.latch_shadow.frame_id);
        }
        s_cam.dropped_window = 0;
    }
}

/* ------------------------------------------------------------------ */
/*  1kHz 任务                                                         */
/* ------------------------------------------------------------------ */
void cam_io_tick_1k(void)
{
    s_cam.tick_1k++;

    /* 相机掉线：使能了触发但长时间收不到 SYNC */
    if (s_cam.trig_enabled) {
        const uint32_t silence = s_cam.tick_1k - s_cam.last_sync_tick_1k;
        if (s_cam.online && silence > CAM_SYNC_TIMEOUT_MS) {
            s_cam.online = false;
            can_node_report_event(CAN_EVT_CAM_SYNC_LOST, 0xFFU, (float)s_cam.cumulative_dropped);
        }
    }
    if (s_cam.online) {
        s_cam.last_sync_tick_1k = s_cam.tick_1k;
    }
}

/* ------------------------------------------------------------------ */
/*  查询                                                              */
/* ------------------------------------------------------------------ */
bool     cam_is_online(void)   { return s_cam.online; }
uint32_t cam_frame_count(void) { return s_cam.frame_id; }
uint32_t cam_last_period_us(void) { return s_cam.period_us; }
uint16_t cam_dropped_in_window(void) { return s_cam.dropped_window; }
const cam_latch_t *cam_get_last_latch(void) { return &s_cam.latch_shadow; }
