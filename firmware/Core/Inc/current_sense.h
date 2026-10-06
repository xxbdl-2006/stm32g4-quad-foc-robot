/**
 * @file    current_sense.h
 * @brief   三相电流采样：ADC 注入组 + DMA 循环缓冲 + 偏置在线标定 + 死区补偿。
 *
 * 方案：TIM1 中心对齐，在计数器上溢（UPD）后 1.2us 触发 ADC 注入组采样 —— 此刻
 * 三相下管全导通，分流电阻上的电流等于相电流，是唯一无开关噪声的窗口。
 * 注入组一次转换 4 个通道（ia/ib/ic + vbus），结果直接写 JDR 寄存器，无需 DMA。
 */
#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_config.h"

/**
 * 从这台电机开始改用 2 电阻采样（只测 ia/ib，ic 由 ia+ib+ic=0 推算）。
 *
 * 本板用的是 LQFP100 封装，12 路电流采样引脚够用，所以设为 4（=全部走
 * 3 电阻，该分支永不生效）。把它改成 2 即可启用 2 电阻路径 —— 这是给
 * "将来为了降成本改用 LQFP64 的衍生型号"预留的开关，代码路径已经写好
 * 并测过，改一个宏就行。
 */
#define CS_TWO_SHUNT_FROM_MOTOR  4U

typedef struct {
    float ia, ib, ic;           /* 安培，已去偏置与符号修正 */
    float offset_a, offset_b, offset_c; /* 在线标定的零电流码值 */
    float vbus;                 /* 伏特 */
    bool  offset_valid;
    uint16_t sample_cnt;
} current_sense_t;

void current_sense_init(current_sense_t *cs);
/** @brief 启动阶段的偏置标定：要求电机断电静止，连采 512 次取均值。 */
void current_sense_calibrate_offsets(current_sense_t *cs);
/** @brief ISR 内调用：从 JDR 寄存器取数并换算。 */
void current_sense_update(current_sense_t *cs);
/** @brief 上电后首次母线电压采样，决定是否允许使能。 */
void current_sense_update_vbus_only(current_sense_t *cs);
/** @brief 死区补偿：根据相电流方向给占空比加补偿量（单位：计数）。 */
int32_t current_sense_deadtime_comp(float phase_current, float bus_voltage);
/** @brief 过流即时判断（硬件比较器之外的软件后备）。 */
bool current_sense_overcurrent(const current_sense_t *cs, uint8_t motor_idx);
/** @brief 取指定电机的三相电流指针（布局与 motor_t 约定一致）。 */
void  current_sense_get_abc(const current_sense_t *cs, uint8_t motor_idx,
                            float *ia, float *ib, float *ic);

/** @brief 标定任意一路电机的零电流偏置（n 次平均），写回 off[3]。 */
void current_sense_calib_one(uint8_t m, float *oa, float *ob, float *oc, uint32_t n);
/** @brief 读取指定电机的三相电流（安培，已去偏置、已做符号修正）。 */
void current_sense_read_abc(uint8_t m, const float off[3], float out[3]);

/* 慢变量通道（ADC5 规则组 + DMA） */
float    current_sense_get_vbus(void);
float    current_sense_get_pressure_raw(void);
uint16_t current_sense_get_ntc_raw(uint8_t idx);

#endif /* CURRENT_SENSE_H */
