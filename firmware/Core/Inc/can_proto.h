/**
 * @file    can_proto.h
 * @brief   自定义精简 CAN 应用层协议（经典 CAN 2.0B 数据帧，1 Mbps）。
 *
 * ── 为什么不用 CANopen ─────────────────────────────────────────────
 * 本项目总线上只有 2 个节点（上位机 + 主板），且帧率需求固定（4×200Hz 反馈）。
 * CANopen 的 SDO/PDO 映射、对象字典、NMT 状态机带来的代码量约 3000 行，
 * 而实际只用到其中的 5%。自定义协议的编解码 + 节点管理不到 600 行，
 * 且 11 位 ID 直接携带路由信息，上位机侧用 Wireshark/candump 一眼能读懂。
 * 代价是与第三方工业驱动器不可互换 —— 本项目无此需求。
 *
 * ── 11 位标识符布局 ───────────────────────────────────────────────
 *
 *     bit  10 9 8 | 7 6 5 4 | 3 2 1 0
 *           NODE  |  CLASS  |   IDX
 *
 *   NODE  3 bit：0=上位机，1=主板，2=扩展板（IMU）
 *   CLASS 4 bit：见 CLS_* 宏
 *   IDX   4 bit：CLASS 内的子索引，语义随 CLASS 变化
 *
 * ── 数据段约定 ───────────────────────────────────────────────────
 *   所有多字节整数一律小端（与 STM32 内存布局一致，memcpy 即可收发）。
 *   浮点：仅在 CLS_MOTION 的设定值字段使用 IEEE754 binary32，
 *   其余位置一律用定点整数，避免不同编译器 -ffast-math 带来的位模式差异。
 */
#ifndef CAN_PROTO_H
#define CAN_PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/*  标识符编解码                                                       */
/* ------------------------------------------------------------------ */
#define CAN_NODE_SHIFT      8U
#define CAN_CLASS_SHIFT     4U
#define CAN_NODE_MASK       0x7U
#define CAN_CLASS_MASK      0xFU
#define CAN_IDX_MASK        0xFU

#define CAN_MKID(node, cls, idx) \
    ((uint16_t)((((node) & CAN_NODE_MASK) << CAN_NODE_SHIFT) | \
                (((cls)  & CAN_CLASS_MASK) << CAN_CLASS_SHIFT) | \
                ((idx)   & CAN_IDX_MASK)))

#define CAN_ID_NODE(id)     (uint8_t)(((id) >> CAN_NODE_SHIFT) & CAN_NODE_MASK)
#define CAN_ID_CLASS(id)    (uint8_t)(((id) >> CAN_CLASS_SHIFT) & CAN_CLASS_MASK)
#define CAN_ID_IDX(id)      (uint8_t)((id) & CAN_IDX_MASK)

/* ------------------------------------------------------------------ */
/*  CLASS 定义                                                        */
/* ------------------------------------------------------------------ */
#define CLS_HEARTBEAT   0x0U    /* 主板->上位机，10Hz 心跳，含状态汇总 */
#define CLS_FAST        0x1U    /* 主板->上位机，200Hz/电机 高速反馈 */
#define CLS_MOTION      0x2U    /* 上位机->主板，运动指令 */
#define CLS_PARAM       0x3U    /* 双向，参数读写 */
#define CLS_IO          0x4U    /* 双向，气泵 / 相机 IO */
#define CLS_FLASH       0x5U    /* 上位机->主板，配置持久化 */
#define CLS_DIAG        0x6U    /* 双向，诊断与统计 */
#define CLS_ESTOP       0x7U    /* 上位机->主板，广播急停（IDX 忽略） */

/* ------------------------------------------------------------------ */
/*  CLS_HEARTBEAT (IDX = 0)                                           */
/* ------------------------------------------------------------------ */
typedef struct __attribute__((packed)) {
    uint16_t fw_version;    /* major<<12 | minor<<8 | patch */
    uint8_t  node_state;    /* 0=boot 1=idle 2=ready 3=running 4=fault */
    uint8_t  fault_index;   /* 首个置位的故障位序号，索引 FAULT_INDEX_* */
    uint16_t fault_flags;   /* 故障位图低 16 位 */
    uint8_t  motors_enabled;/* bit0..3 对应 M0..M3 */
    uint8_t  uptime_s;      /* 秒，8bit 循环 */
} can_heartbeat_t;

#define NODE_STATE_BOOT     0U
#define NODE_STATE_IDLE     1U
#define NODE_STATE_READY    2U
#define NODE_STATE_RUNNING  3U
#define NODE_STATE_FAULT    4U

/* ------------------------------------------------------------------ */
/*  CLS_FAST                                                          */
/* ------------------------------------------------------------------ */
/* IDX 0..3 : 电机反馈，8 字节 */
typedef struct __attribute__((packed)) {
    int32_t  position_mrad;     /* 多圈位置，毫弧度。±2.1e6 rad 量程足够 */
    int16_t  velocity_mrads;    /* 角速度 mrad/s，驱动侧钳到 ±32767 */
    int16_t  current_ma;        /* iq 电流 mA，驱动侧钳到 ±32767 (±32A) */
} can_motor_feedback_t;

#define CAN_VEL_CLAMP_MRADS     32767
#define CAN_CUR_CLAMP_MA        32767

/* IDX 4 : 板级状态 */
typedef struct __attribute__((packed)) {
    uint16_t vbus_mv;
    int16_t  mcu_temp_c10;      /* 0.1°C */
    int16_t  power_w10;         /* 0.1W，4 路电机有功之和 */
    uint8_t  fault_flags_lo;    /* FAULT_* 低 8 位 */
    uint8_t  state;             /* node_state */
} can_board_status_t;

/* ------------------------------------------------------------------ */
/*  CLS_MOTION (IDX = 电机号 0..3；IDX=15 表示 4 台同步广播)             */
/* ------------------------------------------------------------------ */
typedef struct __attribute__((packed)) {
    uint8_t mode;               /* motor_mode_t */
    float   setpoint;           /* rad/s 或 rad 或 A，随 mode */
    uint16_t limit_u16;         /* 电流上限 mA（velocity/position 模式） */
    uint8_t flags;              /* bit0=enable bit1=brake bit2=reset_fault bit3=estop */
} can_motion_cmd_t;

#define MOTION_FLAG_ENABLE      (1U << 0)
#define MOTION_FLAG_BRAKE       (1U << 1)
#define MOTION_FLAG_RESET_FAULT (1U << 2)
#define MOTION_FLAG_ESTOP       (1U << 3)

/* IDX=15 同步广播：4 组 float，用 CAN FD 或拆 2 帧。
 * 本项目走拆帧：第 1 帧装 M0/M1 的 q16.16 速度，第 2 帧装 M2/M3。
 * 之所以不用 float 而用 q16.16，是为了 8 字节塞下 4 个数。 */
typedef struct __attribute__((packed)) {
    int32_t setpoint_q16[2];    /* q16.16 rad/s */
    uint16_t limit_ma[2];
} can_motion_sync_half_t;

#define CAN_MOTION_SYNC_IDX     15U
#define CAN_MOTION_SYNC_HALF0   14U
#define CAN_MOTION_SYNC_HALF1   13U

static inline int32_t q16_from_float(float v) { return (int32_t)(v * 65536.0f); }
static inline float   q16_to_float(int32_t v) { return (float)v / 65536.0f; }

/* ------------------------------------------------------------------ */
/*  CLS_PARAM                                                         */
/* ------------------------------------------------------------------ */
/* 8 字节： [0..1] param_id(LE)  [2..5] float value  [6] op  [7] reserved
 * op: 0=read_req 1=write 2=write_ack 3=read_resp 4=err
 * IDX = 电机号（0..3），参数是否与电机绑定由 param_id 的定义决定 */
#define PARAM_OP_READ_REQ   0U
#define PARAM_OP_WRITE      1U
#define PARAM_OP_WRITE_ACK  2U
#define PARAM_OP_READ_RESP  3U
#define PARAM_OP_ERR        4U

typedef struct __attribute__((packed)) {
    uint16_t param_id;
    float    value;
    uint8_t  op;
    uint8_t  reserved;
} can_param_frame_t;

/* ---- 参数 ID 表 -------------------------------------------------- */
/* 电机级（IDX=0..3 分别对应 M0..M3） */
#define PID_VEL_KP          0x0100U
#define PID_VEL_KI          0x0101U
#define PID_VEL_KD          0x0102U
#define PID_POS_KP          0x0103U
#define PID_POS_KI          0x0104U
#define PID_POS_KD          0x0105U
#define PID_CUR_KP          0x0106U
#define PID_CUR_KI          0x0107U
#define LIMIT_CURRENT_A     0x0110U
#define LIMIT_VELOCITY      0x0111U
#define LIMIT_POSITION_LO   0x0112U
#define LIMIT_POSITION_HI   0x0113U
#define ENC_ELEC_OFFSET     0x0120U  /* 只读，由标定流程写入 */
#define ENCODER_DIRECTION   0x0121U
#define MOTOR_GEAR_RATIO    0x0122U
#define MOTOR_POLE_PAIRS    0x0123U

/* 板级（IDX 固定为 0） */
#define BOARD_VBUS_OVP      0x0200U
#define BOARD_VBUS_UVP      0x0201U
#define BOARD_OTP_MCU       0x0202U
#define BOARD_OTP_MOS       0x0203U
#define BOARD_CAN_TIMEOUT   0x0204U
#define PUMP_PWM_FREQ       0x0300U
#define PUMP_RAMP_MS        0x0301U
#define CAM_TRIG_RATE       0x0400U
#define CAM_TRIG_PULSE_US   0x0401U
#define CAM_TRIG_ENABLE     0x0402U

/* ------------------------------------------------------------------ */
/*  CLS_IO                                                            */
/* ------------------------------------------------------------------ */
#define IO_IDX_PUMP_CMD      0U
#define IO_IDX_PUMP_STATE    1U
#define IO_IDX_CAM_CFG       2U
#define IO_IDX_CAM_SYNC_EVT  3U
#define IO_IDX_CAM_LATCH0    5U   /* ..8 : 依次对应 M0..M3 的同步锁存位置 */
#define IO_IDX_BOARD_IO_STATE 4U

typedef struct __attribute__((packed)) {
    uint8_t  cmd;           /* 0=stop 1=run 2=soft_stop */
    uint16_t duty_permille; /* 0..1000 */
    uint8_t  mode;          /* 0=开环占空比 1=压力闭环 */
    uint16_t target_kpa10;  /* 压力闭环目标，0.1kPa */
    uint8_t  reserved;
} can_pump_cmd_t;

typedef struct __attribute__((packed)) {
    uint8_t  state;         /* 0=idle 1=ramping 2=running 3=fault */
    uint16_t duty_permille;
    uint16_t pressure_kpa10;
    uint8_t  fault;         /* bit0=stall bit1=sensor_open bit2=overcurrent */
    uint8_t  reserved;
} can_pump_state_t;

typedef struct __attribute__((packed)) {
    uint8_t  enable;
    uint16_t rate_hz;       /* 1..120 */
    uint16_t pulse_us;      /* 5..10000 */
    uint8_t  mode;          /* 0=连续定时 1=单次 2=外部触发透传 */
} can_cam_cfg_t;

typedef struct __attribute__((packed)) {
    uint32_t frame_id;      /* 单调递增，用于上位机检测丢帧 */
    uint16_t dt_us;         /* 与上一帧的间隔，微秒（16bit 满量程 65.5ms） */
    uint16_t dropped;       /* 窗口内丢失的同步脉冲数 */
} can_cam_sync_evt_t;

typedef struct __attribute__((packed)) {
    int32_t  position_mrad; /* 同步锁存时的多圈位置 */
    int16_t  velocity_mrads;/* 同步锁存时的机械角速度 */
    uint16_t frame_id_lo;   /* 帧号低 16 位；完整 32 位帧号在同批的 SYNC 事件帧里，
                             * 上位机按"紧随其后"的顺序即可无歧义还原 */
} can_cam_latch_t;

typedef struct __attribute__((packed)) {
    uint16_t pressure_kpa10;
    uint8_t  cam_online;
    uint8_t  cam_rate_hz;
    uint16_t cam_frames_low;  /* frame_id 低 16 位，便于 ROS 侧快速对齐 */
    uint16_t reserved;
} can_board_io_state_t;

/* ------------------------------------------------------------------ */
/*  CLS_DIAG                                                          */
/* ------------------------------------------------------------------ */
#define DIAG_IDX_LOOP_STATS  0U
#define DIAG_IDX_ENC_STATS   1U
#define DIAG_IDX_CAN_STATS   2U
#define DIAG_IDX_UPTIME      3U

typedef struct __attribute__((packed)) {
    uint16_t loop_overrun[MOTOR_COUNT]; /* 每台电机电流环超时次数 */
    uint16_t can_tx_err;
    uint16_t can_rx_err;
    /* 总计 12 字节，超 8 —— 因此本项目实际把它拆成两个 IDX 发，
     * 见 can_node.c 的 diag_pack()。此处结构体按逻辑整体定义。 */
} can_diag_loop_t;

/* 实际发送用的 8 字节版本 */
typedef struct __attribute__((packed)) {
    uint8_t  motor_id;
    uint16_t loop_overrun;
    uint16_t encoder_err;
    uint16_t can_err;
    uint8_t  reserved;
} can_diag_frame_t;

/* ------------------------------------------------------------------ */
/*  工具函数                                                          */
/* ------------------------------------------------------------------ */
static inline int16_t clamp_i16(int32_t v)
{
    if (v > 32767) { return 32767; }
    if (v < -32768) { return -32768; }
    return (int16_t)v;
}

/** 限幅函数统一用 foc_math.h 的 clampf()，这里不再重复定义。
 *  之所以删掉同名的 clamp_f：两个功能完全相同的函数放在两个头文件里，
 *  一旦某个 .c 只包含了其中一个，就会出现"看不见的隐式声明"。 */
/** @brief 便捷打包：把结构体写进 CAN 数据段，用完补 0。 */
static inline void can_pack(uint8_t data[8], const void *src, uint8_t len)
{
    memset(data, 0, 8);
    memcpy(data, src, (len > 8U) ? 8U : len);
}

/** @brief 便捷解包。 */
static inline void can_unpack(void *dst, const uint8_t data[8], uint8_t len)
{
    memcpy(dst, data, (len > 8U) ? 8U : len);
}

/* ------------------------------------------------------------------ */
/*  协议层辅助（实现在 can_proto.c）                                    */
/* ------------------------------------------------------------------ */
/** @brief 故障位序号 -> 可读名称，用于诊断/日志。 */
const char *can_proto_fault_name(uint8_t bit);
/** @brief 节点状态值 -> 可读名称。 */
const char *can_proto_state_name(uint8_t state);

/**
 * @brief 开机的协议自检：校验线格式结构体长度/偏移与 ID 编解码一致性。
 * @return true 全部通过。**返回 false 时必须进 Error_Handler** ——
 *         带着错位的帧格式跑起来，比直接不启动危险得多。
 */
bool can_proto_self_check(void);

/** @brief 构造急停广播帧的数据段（ID 固定 0x070，见 can_proto_estop_id）。 */
void can_proto_build_estop(uint8_t motor_mask, uint8_t data[8]);
uint16_t can_proto_estop_id(void);

#endif /* CAN_PROTO_H */
