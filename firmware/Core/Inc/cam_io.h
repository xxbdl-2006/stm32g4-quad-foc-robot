/**
 * @file    cam_io.h
 * @brief   相机模块的 IO 侧：触发脉冲输出 + 帧同步中断输入 + 多轴位置锁存。
 *
 * 相机模块本身只引出一根 TRIG（输入，外部触发曝光）和一根 SYNC（输出，
 * 帧同步/曝光开始标志）。因此这块的全部逻辑就是：
 *
 *   TRIG  : TIM8_CH1 PWM 输出，频率 1~120Hz 可配，脉宽 5~10000us。
 *           相机设为"外部触发模式"，每个上升沿曝光一帧。
 *
 *   SYNC  : PB11 EXTI 上升沿。每来一个沿做四件事：
 *             ① 记 DWT 周期数，与上一沿求差，得到帧间隔；
 *             ② 帧号自增；若间隔 > 1.6 倍标称周期，判定丢了 1 帧或多帧；
 *             ③ 立刻锁存 4 台电机的当前位置（float 写是原子的，无需关中断）；
 *             ④ 置 latch_pending 标志，由 20kHz 环路去组 CAN 帧发出去。
 *
 *   "锁存"是整个设计里最关键的 5 行代码 —— 它保证了上位机拿到的
 *   图像和四轴位置严格属于同一时刻。如果等到上位机收到 SYNC 帧再去问
 *   电机位置，电机已经又转了几毫秒，四路视觉伺服全部对不齐。
 */
#ifndef CAM_IO_H
#define CAM_IO_H

#include <stdint.h>
#include <stdbool.h>
#include "can_proto.h"

typedef struct {
    uint32_t frame_id;
    float    pos[MOTOR_COUNT];
    float    vel[MOTOR_COUNT];
    uint32_t t_capture_cyc;     /* DWT 周期计数，供上位机做亚毫秒对齐 */
} cam_latch_t;

void cam_io_init(void);
/** @brief 处理上位机的相机配置帧。ISR 上下文。 */
void cam_handle_cfg(const can_cam_cfg_t *cfg);
/** @brief 停止触发输出（急停/E-Stop 时调用）。 */
void cam_trigger_disable(void);
/** @brief 使能触发输出到指定频率。 */
void cam_trigger_enable(uint16_t rate_hz, uint16_t pulse_us);

/** @brief 20kHz 环路里调用：处理待发的锁存结果与超时判定。 */
void cam_io_tick_20k(void);
/** @brief 1kHz 调用：统计与超时。 */
void cam_io_tick_1k(void);

/* ---- 状态查询 ---- */
bool     cam_is_online(void);
uint32_t cam_frame_count(void);
uint32_t cam_last_period_us(void);
uint16_t cam_dropped_in_window(void);
/** @brief 取最近一次锁存（供本地日志/调试）。 */
const cam_latch_t *cam_get_last_latch(void);
/** @brief 寄存器配置：单次触发（曝光一帧后自动停）。 */
void cam_single_shot(void);

#endif /* CAM_IO_H */
