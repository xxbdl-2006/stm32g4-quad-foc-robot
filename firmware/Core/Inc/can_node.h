/**
 * @file    can_node.h
 * @brief   CAN 节点：收发调度、周期帧生成、上位机超时看门狗、事件上抛。
 *
 * 线程模型（在本工程里是"中断优先级模型"）：
 *   - FDCAN1 RX FIFO0 新消息中断（优先级 3）：只做解析 + 置位标志，不做耗时操作
 *   - TIM1_UP 20kHz 中断（优先级 1）：控制环 + 周期帧入 TX FIFO
 *   - 主循环 while(1)（最低优先级）：Flash 写、参数保存、诊断组帧
 *
 * TX 路径使用 FDCAN 的三个专用 TX FIFO/Queue，硬件自动按 ID 优先级仲裁，
 * 不会出现"高优先级反馈帧被参数帧堵住"的情况。
 */
#ifndef CAN_NODE_H
#define CAN_NODE_H

#include <stdint.h>
#include <stdbool.h>
#include "can_proto.h"

/* 事件类型，用于 can_node_report_event 上抛给上位机 */
typedef enum {
    CAN_EVT_NONE = 0,
    CAN_EVT_CALIB_DONE = 1,
    CAN_EVT_FAULT = 2,
    CAN_EVT_MODE_CHANGED = 3,
    CAN_EVT_PARAM_SAVED = 4,
    CAN_EVT_BOOT = 5,
    CAN_EVT_CAM_SYNC_LOST = 6
} can_event_t;

void can_node_init(void);
/** @brief 20kHz 环路里调用：按分频把反馈帧/心跳推进 TX 队列。 */
void can_node_tick_20k(void);
/** @brief 主循环里调用：处理待发送队列、Flash 保存请求。 */
void can_node_poll(void);
/** @brief 事件上抛（会在下一个 1kHz 时隙发出参数帧）。ISR 安全。 */
void can_node_report_event(can_event_t evt, uint8_t motor_id, float payload);

/** @brief 请求把参数写 Flash —— 只置标志，真正的写盘在主循环做。 */
void can_node_request_param_save(void);

/** @brief 处理 CLS_PARAM 帧（读/写/回包）。ISR 上下文。 */
void can_node_handle_param(uint8_t idx, const can_param_frame_t *pf);

/* ---- 供其他模块查询的状态 ---- */
bool     can_node_host_online(void);
uint32_t can_node_host_silence_ms(void);
bool     can_node_bus_off(void);
uint32_t can_node_rx_count(void);
uint32_t can_node_tx_count(void);
uint32_t can_node_err_count(void);

/* ---- 相机同步事件的发送接口（由 cam_io.c 调用，ISR 安全） ---- */
void can_node_send_cam_sync(uint32_t frame_id, uint16_t dt_us, uint16_t dropped);
void can_node_send_cam_latch(uint8_t motor_id, int32_t pos_mrad,
                             int16_t vel_mrads, uint32_t frame_id);
/** @brief 把 4 台电机在同步时刻的位置一次性锁存并发 4 帧。 */
void can_node_publish_latched_pose(uint32_t frame_id);

/* ---- 气泵状态上报 ---- */
void can_node_send_pump_state(void);

#endif /* CAN_NODE_H */
