/**
 * @file    foc_config.h
 * @brief   全板编译期常量：电气参数、控制频率、安全阈值、CAN 节点标识。
 *
 * 板卡型号：HWB-MC4G4 (STM32G474VET6 @ 170MHz)
 * 拓扑    ：单 MCU 集中式 —— 4 路 BLDC FOC + 1 路气泵 PWM + 1 路相机 IO 中断
 *
 * 修改本文件任意常量后必须同时更新 docs/03_hardware.md 的 BOM 参数表，
 * 因为上位机 hw_bringup/config/hardware.yaml 中保存的标定值与之强耦合。
 */
#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/* ------------------------------------------------------------------ */
/*  拓扑与节点身份                                                     */
/* ------------------------------------------------------------------ */
#define FW_VERSION_MAJOR            2U
#define FW_VERSION_MINOR            4U
#define FW_VERSION_PATCH            1U

#define CAN_NODE_ID_MAINBOARD       0x01U   /* 本板 */
#define CAN_NODE_ID_HOST            0x00U   /* Ubuntu 上位机 */
#define CAN_NODE_ID_EXPANSION       0x02U   /* 预留：IMU / 扩展 IO 板 */

#define MOTOR_COUNT                 4U      /* 本板驱动的无刷电机数量 */

/* ------------------------------------------------------------------ */
/*  控制频率链                                                         */
/* ------------------------------------------------------------------ */
#define PWM_FREQ_HZ                 20000U  /* 中心对齐，20kHz 可听域外 */
#define CURRENT_LOOP_HZ             PWM_FREQ_HZ
#define VELOCITY_LOOP_DIV           20U     /* 1 kHz 速度环 */
#define POSITION_LOOP_DIV           20U     /* 1 kHz 位置环 */
#define FEEDBACK_TX_HZ              200U    /* CAN 反馈帧频率（每电机） */
#define HEARTBEAT_TX_HZ             10U

#define CONTROL_LOOP_HZ             ((float)CURRENT_LOOP_HZ)
#define DT_CURRENT                  (1.0f / (float)CURRENT_LOOP_HZ)
#define DT_VELOCITY                 (1.0f / ((float)CURRENT_LOOP_HZ / (float)VELOCITY_LOOP_DIV))
#define DT_POSITION                 (1.0f / ((float)CURRENT_LOOP_HZ / (float)POSITION_LOOP_DIV))

/* ------------------------------------------------------------------ */
/*  电机电气参数（42BLF01-24V 云台/轮毂电机，4 台同型号）               */
/* ------------------------------------------------------------------ */
#define POLE_PAIRS                  7U      /* 14 极 12 槽 */
#define PHASE_R_OHM                 0.42f   /* 相电阻 */
#define PHASE_L_HENRY               0.00019f/* 相电感 190uH */
#define KV_RPM_PER_V                100.0f
#define FLUX_LINKAGE_WB             (1.0f / (KV_RPM_PER_V * 0.10472f * (float)POLE_PAIRS))
#define RATED_CURRENT_A             6.0f
#define PEAK_CURRENT_A              15.0f
#define MAX_MECH_SPEED_RAD_S        60.0f   /* 机械角速度上限 ≈ 573 rpm */

/* 采样链路 */
#define R_SHUNT_OHM                 0.005f  /* 5 mΩ 板载分流 */
#define AMP_GAIN                    20.0f   /* INA240A1 固定增益 */
#define ADC_VREF_V                  3.3f
#define ADC_FULL_SCALE              4096.0f
#define ADC_MID_CODE                2048.0f /* 偏置点，启动时在线标定 */
/* 每 LSB 对应安培数：3.3 / (4096 * 20 * 0.005) = 8.056 mA */
#define CURRENT_LSB_A               (ADC_VREF_V / (ADC_FULL_SCALE * AMP_GAIN * R_SHUNT_OHM))

/* 母线 */
#define VBUS_NOMINAL_V              24.0f
#define VBUS_OVP_V                  29.0f   /* 过压保护阈值 */
#define VBUS_UVP_V                  18.0f   /* 欠压保护阈值 */
#define VBUS_DIV_RATIO              (1.0f / 12.1f) /* 分压比，标定后写入 param */

/* 温度 */
#define MCU_OTP_C                   110.0f  /* MCU 过温关断 */
#define MCU_OTW_C                   95.0f   /* 过温预警 */
#define MOS_OTP_C                   120.0f
#define NTC_BETA                    3950.0f
#define NTC_R25_OHM                 10000.0f

/* ------------------------------------------------------------------ */
/*  编码器：AS5047P，14bit 绝对式，SPI1 四线                               */
/* ------------------------------------------------------------------ */
#define ENCODER_BITS                14U
#define ENCODER_CPR                 (1U << ENCODER_BITS)    /* 16384 */
#define ENCODER_MASK                (ENCODER_CPR - 1U)
#define ENCODER_RES_RAD             (6.28318530718f / (float)ENCODER_CPR)
#define ENCODER_SPI_TIMEOUT_MS      2U
/* 允许的单拍角度跳变（rad），超过判为 SPI 噪声 */
#define ENCODER_MAX_JUMP_RAD        1.2f

/* 电角度零点（每台电机单独标定，存于 Flash param 区） */
#define ENC_OFFSET_FLASH_BASE       0x0803F000UL

/* ------------------------------------------------------------------ */
/*  气泵：24V 隔膜泵，DRV8874 H 桥，TIM1_CH4 PWM 20kHz                    */
/* ------------------------------------------------------------------ */
#define PUMP_PWM_FREQ_HZ            20000U
#define PUMP_DUTY_MAX               1000U   /* 千分比 0..1000 */
#define PUMP_PRESSURE_ADC_CH        12U     /* ADC2_IN12, 0..100kPa 传感器 */
#define PUMP_PRESSURE_FS_KPA        100.0f
#define PUMP_RAMP_MS                250U    /* 软启动斜坡，防止浪涌拉垮母线 */
#define PUMP_STALL_TIMEOUT_MS       3000U

/* ------------------------------------------------------------------ */
/*  相机 IO：定时触发输出 + EXTI 帧同步输入                              */
/* ------------------------------------------------------------------ */
#define CAM_TRIG_TIM                8U      /* TIM8_CH1 -> PA15，50% 占空脉冲 */
#define CAM_TRIG_PULSE_US_MIN       5U
#define CAM_TRIG_PULSE_US_MAX       10000U
#define CAM_TRIG_RATE_HZ_MIN        1U
#define CAM_TRIG_RATE_HZ_MAX        120U
#define CAM_SYNC_EXTI_IRQ           EXTI15_10_IRQn
#define CAM_SYNC_PIN                GPIO_PIN_11 /* PB11，上升沿 */
#define CAM_SYNC_TIMEOUT_MS         500U    /* 超时判为相机掉线 */
#define CAM_FRAME_HISTORY           32U     /* 用于丢帧统计的环形窗口 */

/* ------------------------------------------------------------------ */
/*  安全与保护                                                         */
/* ------------------------------------------------------------------ */
#define FAULT_NONE                  0x0000U
#define FAULT_UNDERVOLTAGE          (1U << 0)
#define FAULT_OVERVOLTAGE           (1U << 1)
#define FAULT_OVERCURRENT           (1U << 2)
#define FAULT_MCU_OVERTEMP          (1U << 3)
#define FAULT_MOS_OVERTEMP          (1U << 4)
#define FAULT_ENCODER_SPI           (1U << 5)
#define FAULT_ENCODER_JUMP          (1U << 6)
#define FAULT_DRV_FAULT_N            (1U << 7)  /* nFAULT 引脚低 */
#define FAULT_CAN_BUSOFF            (1U << 8)
#define FAULT_CAN_TIMEOUT           (1U << 9)  /* 上位机静默 */
#define FAULT_WATCHDOG              (1U << 10)
#define FAULT_CTRL_LATE             (1U << 11) /* 环路抖动 > 10% 周期 */
#define FAULT_PUMP_STALL            (1U << 12)
#define FAULT_CAM_SYNC_LOST         (1U << 13)

#define CAN_RX_TIMEOUT_MS           300U    /* 超过此时长无上位机指令 -> 安全停机 */
#define CTRL_JITTER_LIMIT_CYC       (SystemCoreClock / CURRENT_LOOP_HZ / 10U)

/* ------------------------------------------------------------------ */
/*  控制模式枚举（与 CAN CLS_MOTION 的 mode 字段一一对应）                */
/* ------------------------------------------------------------------ */
typedef enum {
    MOTOR_MODE_IDLE      = 0,   /* 关断，三相桥全下管 */
    MOTOR_MODE_CURRENT   = 1,   /* 电流（转矩）闭环 */
    MOTOR_MODE_VELOCITY  = 2,   /* 速度闭环，内环电流 */
    MOTOR_MODE_POSITION  = 3,   /* 位置闭环，内环速度->电流 */
    MOTOR_MODE_OPENLOOP  = 4,   /* 开环电压矢量，仅调试用 */
    MOTOR_MODE_CALIBRATE = 5,   /* 电角度零点标定 */
    MOTOR_MODE_BRAKE     = 6    /* 三相下管短接制动 */
} motor_mode_t;

typedef enum {
    MSTATE_DISABLED = 0,
    MSTATE_READY    = 1,
    MSTATE_RUNNING  = 2,
    MSTATE_FAULT    = 3,
    MSTATE_CALIB    = 4
} motor_state_t;

#endif /* FOC_CONFIG_H */
