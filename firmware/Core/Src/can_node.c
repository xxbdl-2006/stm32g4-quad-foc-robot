/**
 * @file    can_node.c
 * @brief   CAN 节点实现。
 */
#include "can_node.h"
#include "foc_math.h"   /* clampf / wrap_2pi 等内联数学 */
#include "motor.h"
#include "board.h"
#include "param.h"
#include "pump.h"
#include "cam_io.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  内部状态                                                          */
/* ------------------------------------------------------------------ */
static FDCAN_TxHeaderTypeDef tx_hdr;
static uint8_t  tx_buf[8];

static volatile uint32_t s_rx_count;
static volatile uint32_t s_tx_count;
static volatile uint32_t s_err_count;

static volatile uint32_t s_last_host_rx_tick;   /* 最后一次收到上位机帧的 tick(1kHz) */
static volatile uint32_t s_tick_1k;             /* 1kHz 时基，由 20kHz 分频得到 */

static volatile bool     s_bus_off;
static volatile bool     s_param_save_req;
static volatile bool     s_can_timeout_latched;

/* 待发事件队列（单生产者单消费者，20kHz 写 / 主循环读） */
#define EVENT_QUEUE_LEN 8U
typedef struct { can_event_t evt; uint8_t id; float payload; } evt_t;
static volatile evt_t     s_evt_q[EVENT_QUEUE_LEN];
static volatile uint8_t   s_evt_head, s_evt_tail;

/* 反馈帧分频计数 */
static uint32_t s_fb_div;
static uint32_t s_hb_div;
static uint32_t s_pump_div;

/* ------------------------------------------------------------------ */
/*  低层发送                                                          */
/* ------------------------------------------------------------------ */
static bool can_send(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (s_bus_off) { return false; }

    tx_hdr.Identifier          = id;
    tx_hdr.IdType              = FDCAN_STANDARD_ID;
    tx_hdr.TxFrameType         = FDCAN_DATA_FRAME;
    tx_hdr.DataLength          = (len <= 8U) ? len : 8U;
    tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_hdr.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_hdr.MessageMarker       = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_hdr, (uint8_t *)data) != HAL_OK) {
        s_err_count++;
        return false;
    }
    s_tx_count++;
    return true;
}

/* ------------------------------------------------------------------ */
/*  标识符辅助                                                        */
/* ------------------------------------------------------------------ */
#define MKBROADCAST(cls, idx)   CAN_MKID(CAN_NODE_ID_MAINBOARD, (cls), (idx))
#define REQ_MOTOR_ID(id)        ((CAN_ID_IDX(id) < MOTOR_COUNT) ? CAN_ID_IDX(id) : 0xFFU)

/* ------------------------------------------------------------------ */
/*  初始化                                                            */
/* ------------------------------------------------------------------ */
void can_node_init(void)
{
    FDCAN_FilterTypeDef f;

    /* --- 接收过滤器：只收上位机(0x00)发的帧 + 广播(扩展到 0x7FF) --- */
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = CAN_MKID(CAN_NODE_ID_HOST, 0, 0);
    f.FilterID2    = CAN_MKID(CAN_NODE_MASK, 0, 0);   /* mask：只比对 NODE 段 */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &f);

    /* 全局过滤：不匹配的帧丢弃（总线上没有第三方节点，不需要组播） */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    /* 自动重传：由硬件保证，CAN 无 ACK 时会重试，避免丢关键指令 */
    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 0U, 0U);
    HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                   FDCAN_IT_BUS_OFF |
                                   FDCAN_IT_ERROR_PASSIVE |
                                   FDCAN_IT_TX_FIFO_EMPTY,
                                   0);

    s_last_host_rx_tick = 0;
    s_bus_off = false;
}

/* ------------------------------------------------------------------ */
/*  20kHz 周期帧                                                      */
/* ------------------------------------------------------------------ */
void can_node_tick_20k(void)
{
    s_fb_div++;
    s_hb_div++;
    s_pump_div++;

    /* ---- 1kHz 时基 ---- */
    if (s_fb_div >= (CURRENT_LOOP_HZ / 1000U)) {
        s_fb_div = 0;
        s_tick_1k++;

        /* 上位机静默看门狗 */
        if (!s_can_timeout_latched) {
            const uint32_t silence = s_tick_1k - s_last_host_rx_tick;
            if (silence > g_params.board.can_timeout_ms) {
                s_can_timeout_latched = true;
                /* 所有电机安全停机 —— 失联时不能继续按最后的指令转下去 */
                for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
                    motor_emergency_stop(&g_motors[i]);
                    g_motors[i].fb_fault |= FAULT_CAN_TIMEOUT;
                }
                can_node_report_event(CAN_EVT_FAULT, 0xFFU, (float)FAULT_CAN_TIMEOUT);
            }
        }
    }

    /* ---- 200Hz/电机 高速反馈 ---- */
    if (s_fb_div == 0U || s_fb_div == 1U || s_fb_div == 2U || s_fb_div == 3U) {
        /* 用连续 4 拍分别发 M0..M3，避免同一拍里 4 帧挤在一起 */
    }
    static uint32_t fb_phase = 0;
    fb_phase++;
    if (fb_phase >= (CURRENT_LOOP_HZ / (FEEDBACK_TX_HZ))) {
        fb_phase = 0;
        static uint8_t fb_next = 0;
        can_motor_feedback_t fb;

        motor_t *m = &g_motors[fb_next];
        fb.position_mrad = (int32_t)(m->fb_position * 1000.0f);
        fb.velocity_mrads = clamp_i16((int32_t)(m->fb_velocity * 1000.0f));
        fb.current_ma     = clamp_i16((int32_t)(m->fb_current * 1000.0f));

        can_pack(tx_buf, &fb, sizeof(fb));
        can_send(MKBROADCAST(CLS_FAST, fb_next), tx_buf, 8U);

        /* M0 之后紧接一帧板级状态，占一个时隙 */
        if (fb_next == 0U) {
            can_board_status_t bs;
            const float vbus = board_read_vbus();
            float p = 0.0f;
            for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
                p += g_motors[i].fb_current * g_motors[i].fb_velocity;
            }
            bs.vbus_mv        = (uint16_t)clampf(vbus * 1000.0f, 0.0f, 65535.0f);
            bs.mcu_temp_c10   = (int16_t)(board_read_mcu_temp() * 10.0f);
            bs.power_w10      = (int16_t)clampf(p * 10.0f, -32768.0f, 32767.0f);
            bs.fault_flags_lo = (uint8_t)(m->fb_fault & 0xFFU);
            bs.state          = (uint8_t)(can_node_host_online() ? NODE_STATE_RUNNING
                                                                 : NODE_STATE_READY);
            can_pack(tx_buf, &bs, sizeof(bs));
            can_send(MKBROADCAST(CLS_FAST, 4U), tx_buf, 8U);
        }

        fb_next = (uint8_t)((fb_next + 1U) % MOTOR_COUNT);
    }

    /* ---- 10Hz 心跳 ---- */
    if (s_hb_div >= (CURRENT_LOOP_HZ / HEARTBEAT_TX_HZ)) {
        s_hb_div = 0;
        can_heartbeat_t hb;
        uint16_t fault_all = 0;
        uint8_t enabled_mask = 0;
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            fault_all |= g_motors[i].fb_fault;
            if (g_motors[i].state == MSTATE_RUNNING) { enabled_mask |= (uint8_t)(1U << i); }
        }
        hb.fw_version    = (uint16_t)((FW_VERSION_MAJOR << 12) | (FW_VERSION_MINOR << 8) | FW_VERSION_PATCH);
        hb.node_state    = (uint8_t)(fault_all ? NODE_STATE_FAULT
                                      : (enabled_mask ? NODE_STATE_RUNNING : NODE_STATE_READY));
        hb.fault_flags   = fault_all;
        hb.fault_index   = (uint8_t)__builtin_ctz((unsigned)(fault_all ? fault_all : 1U));
        hb.motors_enabled = enabled_mask;
        hb.uptime_s      = (uint8_t)(HAL_GetTick() / 1000U);
        can_pack(tx_buf, &hb, sizeof(hb));
        can_send(MKBROADCAST(CLS_HEARTBEAT, 0U), tx_buf, 8U);
    }

    /* ---- 50Hz 气泵状态 ---- */
    if (s_pump_div >= (CURRENT_LOOP_HZ / 50U)) {
        s_pump_div = 0;
        can_node_send_pump_state();
    }

    /* ---- 处理事件队列（1kHz 时隙） ---- */
    if (s_tick_1k % 5U == 0U && s_evt_head != s_evt_tail) {
        evt_t e = s_evt_q[s_evt_tail];
        s_evt_tail = (uint8_t)((s_evt_tail + 1U) % EVENT_QUEUE_LEN);
        can_param_frame_t pf;
        pf.param_id = (uint16_t)(0x8000U | (uint16_t)e.evt);   /* 0x8xxx = 事件通道 */
        pf.value    = e.payload;
        pf.op       = PARAM_OP_READ_RESP;
        pf.reserved = e.id;
        can_pack(tx_buf, &pf, sizeof(pf));
        can_send(MKBROADCAST(CLS_DIAG, DIAG_IDX_UPTIME), tx_buf, 8U);
    }
}

/* ------------------------------------------------------------------ */
/*  主循环任务                                                        */
/* ------------------------------------------------------------------ */
void can_node_poll(void)
{
    /* Flash 保存：会关中断约 20ms，所以只在主循环做 */
    if (s_param_save_req) {
        s_param_save_req = false;
        HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        const bool ok = param_save();
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        can_node_report_event(ok ? CAN_EVT_PARAM_SAVED : CAN_EVT_FAULT, 0xFFU,
                              ok ? 1.0f : 0.0f);
    }

    /* 总线恢复：处于 bus-off 时尝试在静默 100ms 后重启 */
    if (s_bus_off) {
        static uint32_t t0 = 0;
        if (HAL_GetTick() - t0 > 100U) {
            HAL_FDCAN_Start(&hfdcan1);
            s_bus_off = false;
        } else if (t0 == 0U) {
            t0 = HAL_GetTick();
        }
    }
}

void can_node_request_param_save(void) { s_param_save_req = true; }

void can_node_report_event(can_event_t evt, uint8_t motor_id, float payload)
{
    const uint8_t next = (uint8_t)((s_evt_head + 1U) % EVENT_QUEUE_LEN);
    if (next == s_evt_tail) { return; }     /* 队列满，丢弃最旧语义：宁丢事件不阻塞 ISR */
    s_evt_q[s_evt_head].evt = evt;
    s_evt_q[s_evt_head].id = motor_id;
    s_evt_q[s_evt_head].payload = payload;
    s_evt_head = next;
}

/* ------------------------------------------------------------------ */
/*  接收解析                                                          */
/* ------------------------------------------------------------------ */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) { return; }

    FDCAN_RxHeaderTypeDef rx_hdr;
    uint8_t rx[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_hdr, rx) != HAL_OK) { break; }
        s_rx_count++;
        s_last_host_rx_tick = s_tick_1k;
        s_can_timeout_latched = false;

        const uint8_t cls = CAN_ID_CLASS(rx_hdr.Identifier);
        const uint8_t idx = CAN_ID_IDX(rx_hdr.Identifier);

        switch (cls) {
            case CLS_ESTOP:
                for (uint8_t i = 0; i < MOTOR_COUNT; i++) { motor_emergency_stop(&g_motors[i]); }
                pump_stop(true);
                cam_trigger_disable();
                break;

            case CLS_MOTION: {
                can_motion_cmd_t cmd;
                can_unpack(&cmd, rx, sizeof(cmd));
                if (idx < MOTOR_COUNT) {
                    motor_set_mode(&g_motors[idx], (motor_mode_t)cmd.mode);
                    motor_set_target(&g_motors[idx], cmd.setpoint,
                                     (float)cmd.limit_u16 / 1000.0f, cmd.flags);
                } else if (idx == CAN_MOTION_SYNC_IDX) {
                    can_motion_sync_half_t h;
                    can_unpack(&h, rx, sizeof(h));
                    for (uint8_t k = 0; k < 2U; k++) {
                        motor_set_mode(&g_motors[k], MOTOR_MODE_VELOCITY);
                        motor_set_target(&g_motors[k], q16_to_float(h.setpoint_q16[k]),
                                         (float)h.limit_ma[k] / 1000.0f,
                                         MOTION_FLAG_ENABLE);
                    }
                }
                break;
            }

            case CLS_PARAM: {
                can_param_frame_t pf;
                can_unpack(&pf, rx, sizeof(pf));
                can_node_handle_param(idx, &pf);
                break;
            }

            case CLS_IO: {
                switch (idx) {
                    case IO_IDX_PUMP_CMD: {
                        can_pump_cmd_t pc;
                        can_unpack(&pc, rx, sizeof(pc));
                        pump_handle_cmd(&pc);
                        break;
                    }
                    case IO_IDX_CAM_CFG: {
                        can_cam_cfg_t cc;
                        can_unpack(&cc, rx, sizeof(cc));
                        cam_handle_cfg(&cc);
                        break;
                    }
                    default: break;
                }
                break;
            }

            case CLS_FLASH: {
                can_param_frame_t pf;
                can_unpack(&pf, rx, sizeof(pf));
                if (pf.op == PARAM_OP_WRITE) {
                    if (pf.value > 0.5f) { s_param_save_req = true; }
                    else { param_load(); }
                }
                break;
            }

            case CLS_DIAG: {
                /* 上位机请求诊断：立即组一帧回 */
                can_diag_frame_t d;
                d.motor_id = idx;
                if (idx < MOTOR_COUNT) {
                    d.loop_overrun = (uint16_t)g_motors[idx].loop_overrun_cnt;
                    d.encoder_err  = (uint16_t)g_motors[idx].enc.crc_err_cnt;
                } else {
                    d.loop_overrun = 0; d.encoder_err = 0;
                }
                d.can_err = (uint16_t)(s_err_count & 0xFFFFU);
                d.reserved = 0;
                can_pack(tx_buf, &d, sizeof(d));
                can_send(MKBROADCAST(CLS_DIAG, DIAG_IDX_CAN_STATS), tx_buf, 8U);
                break;
            }

            default: break;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  参数读写处理                                                      */
/* ------------------------------------------------------------------ */
void can_node_handle_param(uint8_t idx, const can_param_frame_t *pf)
{
    can_param_frame_t resp = *pf;
    float v = 0.0f;

    if (pf->op == PARAM_OP_READ_REQ) {
        bool ok = false;
        if (idx < MOTOR_COUNT) {
            ok = motor_param_read(&g_motors[idx], pf->param_id, &v);
        } else {
            ok = param_read_board(pf->param_id, &v);
        }
        resp.value = v;
        resp.op = ok ? PARAM_OP_READ_RESP : PARAM_OP_ERR;
    } else if (pf->op == PARAM_OP_WRITE) {
        bool ok = false;
        if (idx < MOTOR_COUNT) {
            ok = motor_param_write(&g_motors[idx], pf->param_id, pf->value);
        } else {
            ok = param_write_board(pf->param_id, pf->value);
        }
        resp.op = ok ? PARAM_OP_WRITE_ACK : PARAM_OP_ERR;
    }

    can_pack(tx_buf, &resp, sizeof(resp));
    can_send(MKBROADCAST(CLS_PARAM, idx), tx_buf, 8U);
}

/* ------------------------------------------------------------------ */
/*  相机同步发帧                                                      */
/* ------------------------------------------------------------------ */
void can_node_send_cam_sync(uint32_t frame_id, uint16_t dt_us, uint16_t dropped)
{
    can_cam_sync_evt_t e;
    e.frame_id = frame_id;
    e.dt_us    = dt_us;
    e.dropped  = dropped;
    can_pack(tx_buf, &e, sizeof(e));
    can_send(MKBROADCAST(CLS_IO, IO_IDX_CAM_SYNC_EVT), tx_buf, 8U);
}

void can_node_send_cam_latch(uint8_t motor_id, int32_t pos_mrad,
                             int16_t vel_mrads, uint32_t frame_id)
{
    can_cam_latch_t l;
    l.position_mrad  = pos_mrad;
    l.velocity_mrads = vel_mrads;
    l.frame_id_lo    = (uint16_t)(frame_id & 0xFFFFU);
    can_pack(tx_buf, &l, sizeof(l));
    can_send(MKBROADCAST(CLS_IO, (uint8_t)(IO_IDX_CAM_LATCH0 + motor_id)), tx_buf, 8U);
}

void can_node_publish_latched_pose(uint32_t frame_id)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        can_node_send_cam_latch(i,
                                (int32_t)(g_motors[i].fb_position * 1000.0f),
                                clamp_i16((int32_t)(g_motors[i].fb_velocity * 1000.0f)),
                                frame_id);
    }
}

void can_node_send_pump_state(void)
{
    can_pump_state_t ps;
    pump_get_state(&ps);
    can_pack(tx_buf, &ps, sizeof(ps));
    can_send(MKBROADCAST(CLS_IO, IO_IDX_PUMP_STATE), tx_buf, 8U);
}

/* ------------------------------------------------------------------ */
/*  错误回调                                                          */
/* ------------------------------------------------------------------ */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    s_err_count++;
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U) {
        s_bus_off = true;
        /* 总线断开时把所有执行器停掉，避免"最后一个指令"被无限执行 */
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            motor_emergency_stop(&g_motors[i]);
            g_motors[i].fb_fault |= FAULT_CAN_BUSOFF;
        }
        pump_stop(true);
        cam_trigger_disable();
    }
}

/* ------------------------------------------------------------------ */
/*  查询接口                                                          */
/* ------------------------------------------------------------------ */
bool can_node_host_online(void)
{
    return (s_tick_1k - s_last_host_rx_tick) < g_params.board.can_timeout_ms;
}
uint32_t can_node_host_silence_ms(void) { return s_tick_1k - s_last_host_rx_tick; }
bool     can_node_bus_off(void)         { return s_bus_off; }
uint32_t can_node_rx_count(void)        { return s_rx_count; }
uint32_t can_node_tx_count(void)        { return s_tx_count; }
uint32_t can_node_err_count(void)       { return s_err_count; }
