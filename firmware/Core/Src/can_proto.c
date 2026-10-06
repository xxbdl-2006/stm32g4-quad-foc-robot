/**
 * @file    can_proto.c
 * @brief   协议层的运行时辅助：结构体长度自检、故障位名称表、急停帧构造。
 *
 * 为什么需要一个 .c 文件而不是全放头文件里：
 *   1) 故障位名称表（13 条字符串）只被主循环里的诊断组帧用到，
 *      放头文件会被复制到每个包含 can_proto.h 的编译单元，白烧 Flash；
 *   2) 结构体长度自检必须在**运行时**也做一遍 —— `__attribute__((packed))`
 *      在 GCC 上没问题，但如果将来有人用 IAR 编译固件，打包规则可能不同，
 *      而这个错误会导致"C 端和 C++ 端都编过了，但字段全错位"。
 *      在开机时断言一次，比在现场排查三天便宜太多。
 */
#include "can_proto.h"
#include <stddef.h>

/* ------------------------------------------------------------------ */
/*  名称表                                                            */
/* ------------------------------------------------------------------ */
static const char *const k_fault_names[16] = {
    "UNDERVOLTAGE",     /* bit0  */
    "OVERVOLTAGE",      /* bit1  */
    "OVERCURRENT",      /* bit2  */
    "MCU_OVERTEMP",     /* bit3  */
    "MOS_OVERTEMP",     /* bit4  */
    "ENCODER_SPI",      /* bit5  */
    "ENCODER_JUMP",     /* bit6  */
    "DRV_FAULT_N",      /* bit7  */
    "CAN_BUSOFF",       /* bit8  */
    "CAN_TIMEOUT",      /* bit9  */
    "WATCHDOG",         /* bit10 */
    "CTRL_LATE",        /* bit11 */
    "PUMP_STALL",       /* bit12 */
    "CAM_SYNC_LOST",    /* bit13 */
    "reserved14",       /* bit14 */
    "reserved15"        /* bit15 */
};

static const char *const k_state_names[] = {
    "BOOT", "IDLE", "READY", "RUNNING", "FAULT"
};

const char *can_proto_fault_name(uint8_t bit)
{
    return (bit < 16U) ? k_fault_names[bit] : "?";
}

const char *can_proto_state_name(uint8_t state)
{
    return (state < 5U) ? k_state_names[state] : "?";
}

/* ------------------------------------------------------------------ */
/*  布局自检                                                          */
/* ------------------------------------------------------------------ */
/**
 * @brief 校验线格式结构体的长度与字段偏移。开机调用一次，失败即 Error_Handler。
 *
 * 这是本项目里唯一一个"编译期 + 运行期双重保险"的地方，理由是：
 * 上位机侧的 C++ 用 #pragma pack(push,1)，固件侧用 __attribute__((packed))，
 * 两套打包机制的语义在 GCC 下一致，但在其他编译器上未必。一旦不一致，
 * 症状是"通信一切正常但数值全乱" —— 极难定位。所以宁可开机自杀。
 */
bool can_proto_self_check(void)
{
    /* 长度 */
    if (sizeof(can_heartbeat_t)     != 8U) { return false; }
    if (sizeof(can_motor_feedback_t) != 8U) { return false; }
    if (sizeof(can_board_status_t)  != 8U) { return false; }
    if (sizeof(can_motion_cmd_t)    != 8U) { return false; }
    if (sizeof(can_param_frame_t)   != 8U) { return false; }
    if (sizeof(can_pump_cmd_t)      != 8U) { return false; }
    if (sizeof(can_pump_state_t)    != 8U) { return false; }
    if (sizeof(can_cam_cfg_t)       != 8U) { return false; }
    if (sizeof(can_cam_sync_evt_t)  != 8U) { return false; }
    if (sizeof(can_cam_latch_t)     != 8U) { return false; }

    /* 关键字段偏移（抓的是最容易因为对齐而跑偏的位置） */
    if (offsetof(can_motor_feedback_t, velocity_mrads) != 4U) { return false; }
    if (offsetof(can_motion_cmd_t, setpoint)           != 1U) { return false; }
    if (offsetof(can_motion_cmd_t, limit_u16)          != 5U) { return false; }
    if (offsetof(can_param_frame_t, value)             != 2U) { return false; }
    if (offsetof(can_cam_latch_t, frame_id_lo)         != 6U) { return false; }

    /* 标识符编解码的往返一致性 */
    for (uint16_t node = 0; node < 8U; node++) {
        for (uint16_t cls = 0; cls < 16U; cls++) {
            for (uint16_t idx = 0; idx < 16U; idx++) {
                const uint16_t id = CAN_MKID(node, cls, idx);
                if (id > 0x7FFU)                          { return false; }
                if (CAN_ID_NODE(id)  != node)             { return false; }
                if (CAN_ID_CLASS(id) != cls)              { return false; }
                if (CAN_ID_IDX(id)   != idx)              { return false; }
            }
        }
    }
    return true;
}

/* ------------------------------------------------------------------ */
/*  急停帧                                                           */
/* ------------------------------------------------------------------ */
/**
 * @brief 构造急停广播帧（CLS_ESTOP，IDX 忽略）。
 *
 * 上位机侧也要用同样的 ID 才能被固件接受（固件的接收过滤器只放行
 * NODE_HOST=0 的帧，因此急停帧的 ID 固定是 0x070）。
 * 把 ID 常量放在协议层而不是散落在调用点，是为了避免出现
 * "急停发到了 0x170（NODE=1）上，固件根本收不到"这种事。
 */
void can_proto_build_estop(uint8_t motor_mask, uint8_t data[8])
{
    memset(data, 0, 8U);
    /* data[0] = 目标电机掩码，0xFF 表示全部 */
    data[0] = motor_mask;
    /* data[1] = 停机方式：0=自由滑行 1=制动 2=受控减速 */
    data[1] = 2U;
    /* data[2..3] = 受控减速的减速度上限，0.01 rad/s² 为单位 */
    const uint16_t decel_centi_rads2 = 3000U;   /* 30 rad/s² */
    data[2] = (uint8_t)(decel_centi_rads2 & 0xFFU);
    data[3] = (uint8_t)(decel_centi_rads2 >> 8);
}

#define CAN_ESTOP_ID  0x070U

uint16_t can_proto_estop_id(void)
{
    return CAN_ESTOP_ID;
}
