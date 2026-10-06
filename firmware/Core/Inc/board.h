/**
 * @file    board.h
 * @brief   板级引脚映射与外设句柄集中声明。
 *
 * 引脚分配表（STM32G474VET6, LQFP100）。改板时必须同步 docs/03_hardware.md。
 *
 *  ┌────────────┬────────────────────┬─────────────────────────────────┐
 *  │ 功能        │ 引脚                │ 外设 / 说明                      │
 *  ├────────────┼────────────────────┼─────────────────────────────────┤
 *  │ M0 三相      │ PA8 / PA9 / PA10   │ TIM1_CH1/2/3，中心对齐 20kHz     │
 *  │ M1 三相      │ PA0 / PA1 / PA2    │ TIM2_CH1/2/3                    │
 *  │ M2 三相      │ PC6 / PC7 / PC8    │ TIM8_CH1/2/3                    │
 *  │ M3 三相      │ PB6 / PB7 / PB8    │ TIM4_CH1/2/3                    │
 *  ├────────────┼────────────────────┼─────────────────────────────────┤
 *  │ 电流采样      │ PB2/PB3/PB4 + PA6/PA7 + PB9/PB10 + PC0/PC1        │
 *  │             │ 各接 ADC1..ADC4 注入组，由 TIM1_UP 触发            │
 *  │ 母线电压      │ PA3 (ADC1_IN4)     │ 分压 12.1:1，ADC5 规则组         │
 *  │ 泵 压力      │ PC4 (ADC5_IN12)    │ 0~100kPa 模拟输入               │
 *  │ NTC ×4      │ PC2/PC3/PC5/PC15   │ B=3950，10k 上拉，ADC5 规则组    │
 *  ├────────────┼────────────────────┼─────────────────────────────────┤
 *  │ 编码器 SPI    │ PA5 / PA6 / PA7    │ SPI1，10MHz，mode1              │
 *  │ 编码器 CS ×4  │ PA4 / PA15 / PB0 / PB1 │ GPIO 推挽                  │
 *  ├────────────┼────────────────────┼─────────────────────────────────┤
 *  │ CAN          │ PB11(RX) / PB9→复用 │ FDCAN1，1Mbps                   │
 *  │ 泵 PWM       │ PA11               │ TIM1_CH4 -> DRV8874 IN1         │
 *  │ 泵 方向       │ PA12               │ GPIO                            │
 *  │ 泵 使能       │ PB5                │ GPIO（DRV8874 nSLEEP，高有效）    │
 *  │ 相机 触发     │ PB14               │ TIM15_CH1，脉冲输出              │
 *  │ 相机 帧同步   │ PB13               │ EXTI13，上升沿                   │
 *  ├────────────┼────────────────────┼─────────────────────────────────┤
 *  │ 驱动使能 ×4   │ PB3 / PB4 / PB10 / PB12 │ GPIO，高有效（nSLEEP）      │
 *  │ 驱动 nFAULT×4 │ PC9 / PC10 / PC11 / PC12 │ GPIO，低有效 + EXTI 下降沿  │
 *  │ SWD          │ PA13(SWDIO) / PA14(SWCLK) │                          │
 *  │ LED 运行      │ PC13                │ 1Hz 闪烁                        │
 *  │ LED 故障      │ PC14                │ 常亮 = 有故障                    │
 *  └────────────┴────────────────────┴─────────────────────────────────┘
 *
 * 用 LQFP100 而不是 LQFP64 的原因：4 路电机各 3 个 PWM 输出 + 3 路电流采样，
 * 就已经吃掉 24 个引脚，而 G4 的 TIM 输出与 ADC 输入在多数引脚上是复用的 ——
 * 引脚复用冲突无法靠软件绕开。LQFP64 下必须砍到 2 电阻采样甚至 1 电阻采样，
 * 会牺牲低速电流精度。多出来的 36 个引脚换回完整的 3 电阻采样，划算。
 */
#ifndef BOARD_H
#define BOARD_H

#include "stm32g4xx_hal.h"
#include "foc_config.h"

/* ---------------- 编码器片选（SPI1 共用 SCK/MISO/MOSI） ----------------
 * 四片 AS5047P 的 CS 引脚分散在两个端口上，因此不能用单个 PORT 宏表示。
 * 低层由 board_encoder_cs() 封装，encoder.c 不直接碰引脚。 */
#define ENC_CS0_PORT        GPIOA
#define ENC_CS0_PIN         GPIO_PIN_4    /* PA4  */
#define ENC_CS1_PORT        GPIOA
#define ENC_CS1_PIN         GPIO_PIN_15   /* PA15 */
#define ENC_CS2_PORT        GPIOB
#define ENC_CS2_PIN         GPIO_PIN_0    /* PB0  */
#define ENC_CS3_PORT        GPIOB
#define ENC_CS3_PIN         GPIO_PIN_1    /* PB1  */

/** @brief 控制指定编码器的片选：active=true 拉低选中，false 拉高释放。 */
void board_encoder_cs(uint8_t idx, bool active);

/* ---------------- 驱动器 nFAULT ---------------- */
#define DRV_FAULT_PORT      GPIOC
#define DRV_FAULT0_PIN      GPIO_PIN_9
#define DRV_FAULT1_PIN      GPIO_PIN_10
#define DRV_FAULT2_PIN      GPIO_PIN_11
#define DRV_FAULT3_PIN      GPIO_PIN_12

/* ---------------- 驱动器使能（nSLEEP，高有效） ---------------- */
#define DRV_EN_PORT         GPIOB
#define DRV_EN0_PIN         GPIO_PIN_3
#define DRV_EN1_PIN         GPIO_PIN_4
#define DRV_EN2_PIN         GPIO_PIN_10
#define DRV_EN3_PIN         GPIO_PIN_12

/* ---------------- 气泵 ---------------- */
#define PUMP_DIR_PORT       GPIOA
#define PUMP_DIR_PIN        GPIO_PIN_12
#define PUMP_EN_PORT        GPIOB
#define PUMP_EN_PIN         GPIO_PIN_5

/* ---------------- 相机 ---------------- */
#define CAM_SYNC_PORT       GPIOB
#define CAM_SYNC_PIN        GPIO_PIN_13   /* EXTI13，属 EXTI15_10 组 */
#define CAM_TRIG_PORT       GPIOB
#define CAM_TRIG_PIN        GPIO_PIN_14   /* TIM15_CH1 */

/* ---------------- LED / 调试 ---------------- */
#define LED_RUN_PORT        GPIOC
#define LED_RUN_PIN         GPIO_PIN_13
#define LED_FAULT_PORT      GPIOC
#define LED_FAULT_PIN       GPIO_PIN_14

/* ---------------- 外设句柄（CubeMX 生成，定义在 main.c） ---------------- */
extern TIM_HandleTypeDef  htim1;    /* M0 三相 + 泵 PWM(CH4) */
extern TIM_HandleTypeDef  htim2;    /* M1 三相 */
extern TIM_HandleTypeDef  htim8;    /* M2 三相 */
extern TIM_HandleTypeDef  htim4;    /* M3 三相 */
extern TIM_HandleTypeDef  htim15;   /* 相机触发脉冲输出 */
/* 电流采样用 ADC1..ADC4 —— G474 有多达 5 个 ADC，正好一路电机一个 ADC，
 * 各自注入组 3 通道（ia/ib/ic），互不争抢 S/H，采样时刻由同一个 TIM1 触发，
 * 保证 4 台电机的电流在时间上严格对齐（做四轮力矩分配时很重要）。
 * ADC5 的规则组跑 DMA 循环，采母线电压与泵压力这类慢变量（1kHz 足够）。 */
extern ADC_HandleTypeDef  hadc1;    /* M0 三相电流 */
extern ADC_HandleTypeDef  hadc2;    /* M1 三相电流 */
extern ADC_HandleTypeDef  hadc3;    /* M2 三相电流 */
extern ADC_HandleTypeDef  hadc4;    /* M3 三相电流 */
extern ADC_HandleTypeDef  hadc5;    /* 规则组：vbus + pump pressure + 4 路 NTC */
extern SPI_HandleTypeDef  hspi1;
extern FDCAN_HandleTypeDef hfdcan1;
extern CORDIC_HandleTypeDef hcordic;

/* ---------------- 板级初始化 / 工具 ---------------- */
void board_clock_init(void);
void board_gpio_init(void);
void board_led_set(uint8_t which, bool on);
/** @brief 读 MCU 内部温度传感器，返回摄氏度。 */
float board_read_mcu_temp(void);
/** @brief 读某路 MOS 的 NTC 温度，返回摄氏度。 */
float board_read_mos_temp(uint8_t motor_idx);
/** @brief 母线电压，伏特。 */
float board_read_vbus(void);
/** @brief 立即封锁全部三相输出（下管短接），用于故障处理。ISR 安全。 */
void board_all_motors_off(void);
/** @brief 把 SVPWM 结果写到对应定时器的三个 CCR。ISR 安全，无锁。 */
void board_pwm_write(uint8_t motor_idx, const pwm_duty_t *duty);
/** @brief 使能/关断某路三相桥（关断 = 上下管全关，电机自由滑行）。 */
void board_pwm_enable(uint8_t motor_idx, bool en);
/** @brief 三相下管同时导通（制动），用于 MOTOR_MODE_BRAKE。 */
void board_pwm_brake(uint8_t motor_idx, bool en);
/** @brief 读某路驱动器 nFAULT 引脚电平（低有效）。 */
bool board_drv_fault_active(uint8_t motor_idx);
/** @brief 触发一次软件复位。 */
void board_system_reset(void);

#endif /* BOARD_H */
