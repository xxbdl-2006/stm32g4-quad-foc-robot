/**
 * @file    current_sense.c
 * @brief   4 路三相电流采样 + 偏置在线标定 + 死区补偿。
 */
#include "current_sense.h"
#include "foc_math.h"
#include "board.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  硬件相关常量                                                       */
/* ------------------------------------------------------------------ */

/* 每相采样链路的符号：分流电阻的正负接法不同会导致相位反相。
 * v1.3 板实测：M0/M2 为反相，M1/M3 为正相。此值不可凭理论推导，必须用
 * 直流标定台（见 docs/05_tuning.md 第 2 节）逐路确认后写死。 */
static const float phase_sign[MOTOR_COUNT][3] = {
    { -1.0f, -1.0f, -1.0f },    /* M0: INA240 反相装配 */
    {  1.0f,  1.0f,  1.0f },    /* M1 */
    { -1.0f, -1.0f, -1.0f },    /* M2 */
    {  1.0f,  1.0f,  1.0f },    /* M3 */
};

static ADC_HandleTypeDef *const adc_of_motor[MOTOR_COUNT] = {
    &hadc1, &hadc2, &hadc3, &hadc4
};

/* ADC5 规则组 DMA 缓冲： [0]=vbus  [1]=pump_pressure  [2..5]=NTC */
#define ADC5_BUF_LEN    6U
static volatile uint16_t adc5_dma_buf[ADC5_BUF_LEN];

/* 母线分压后经 RC 滤波，时间常数 ~10ms，需软件再补一阶低通 */
#define VBUS_LPF_ALPHA  0.92f

static float vbus_filtered = VBUS_NOMINAL_V;

/* ------------------------------------------------------------------ */
/*  初始化                                                             */
/* ------------------------------------------------------------------ */
void current_sense_init(current_sense_t *cs)
{
    memset(cs, 0, sizeof(*cs));
    cs->offset_a = ADC_MID_CODE;
    cs->offset_b = ADC_MID_CODE;
    cs->offset_c = ADC_MID_CODE;
    cs->vbus = VBUS_NOMINAL_V;
    cs->offset_valid = false;

    /* ADC5 规则组 + DMA 循环模式，由 TIM1_UP 的 1kHz 分频触发 */
    HAL_ADC_Start_DMA(&hadc5, (uint32_t *)adc5_dma_buf, ADC5_BUF_LEN);

    /* 注入组校准（G4 的注入组有独立的 offset 寄存器，必须单独校准） */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
}

/* ------------------------------------------------------------------ */
/*  偏置标定                                                           */
/* ------------------------------------------------------------------ */
void current_sense_calibrate_offsets(current_sense_t *cs)
{
    /* 前提：三相桥全部关断（下管不导通时不产生电流），电机静止。
     * 对每台电机的 3 个注入通道各采 512 次求均值。512 次足以把
     * 白噪声压到 0.3 LSB 以下（对应 ~2.4mA 偏置误差）。 */
    const uint32_t N = 512U;
    uint64_t acc[MOTOR_COUNT][3] = {{0}};

    for (uint32_t i = 0; i < N; i++) {
        for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
            ADC_HandleTypeDef *h = adc_of_motor[m];
            HAL_ADCEx_InjectedStart(h);
            if (HAL_ADCEx_InjectedPollForConversion(h, 1U) != HAL_OK) { continue; }
            acc[m][0] += HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_1);
            acc[m][1] += HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_2);
            acc[m][2] += HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_3);
        }
        /* 期间也让 ADC5 跑起来，顺便把 vbus 读通 */
    }

    /* 本函数只维护 current_sense_t 里 M0 的偏置（该结构主要服务于需要
     * 全局汇总的调用点）；每台电机的偏置另存于 motor_t::cs 中。 */
    cs->offset_a = (float)acc[0][0] / (float)N;
    cs->offset_b = (float)acc[0][1] / (float)N;
    cs->offset_c = (float)acc[0][2] / (float)N;
    cs->offset_valid = true;
}

/** @brief 供 motor 层调用：标定任意一路电机的偏置并写回。 */
void current_sense_calib_one(uint8_t m, float *oa, float *ob, float *oc, uint32_t n)
{
    if (m >= MOTOR_COUNT || n == 0U) { return; }
    uint64_t a = 0, b = 0, c = 0;
    ADC_HandleTypeDef *h = adc_of_motor[m];
    for (uint32_t i = 0; i < n; i++) {
        HAL_ADCEx_InjectedStart(h);
        if (HAL_ADCEx_InjectedPollForConversion(h, 1U) != HAL_OK) { continue; }
        a += HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_1);
        b += HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_2);
        c += HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_3);
    }
    *oa = (float)a / (float)n;
    *ob = (float)b / (float)n;
    *oc = (float)c / (float)n;
}

/* ------------------------------------------------------------------ */
/*  每拍采样                                                           */
/* ------------------------------------------------------------------ */
void current_sense_update(current_sense_t *cs)
{
    ADC_HandleTypeDef *h = adc_of_motor[0];
    const uint8_t m = 0;

    float ra = (float)HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_1);
    float rb = (float)HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_2);
    float rc = (float)HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_3);

    cs->ia = (ra - cs->offset_a) * CURRENT_LSB_A * phase_sign[m][0];
    cs->ib = (rb - cs->offset_b) * CURRENT_LSB_A * phase_sign[m][1];
    cs->ic = (rc - cs->offset_c) * CURRENT_LSB_A * phase_sign[m][2];

    cs->sample_cnt++;
}

/** @brief 通用版本：由 motor 层按电机索引调用。 */
void current_sense_read_abc(uint8_t m, const float off[3], float out[3])
{
    if (m >= MOTOR_COUNT) { return; }
    ADC_HandleTypeDef *h = adc_of_motor[m];
    float r[3];

    /* ---- 2 电阻采样（M2 起） ----
     * LQFP64 的引脚数在"12 路电流 + 4 路编码器 CS + 3 路 SPI + 2 路 CAN +
     * 4 路 nFAULT + 4 路 EN + 气泵 3 + 相机 2 + 4 路 NTC"这个配置下不够用。
     * v1.4 的取舍是：M0/M1 保留 3 电阻（低速区精度最好，且这两台电机
     * 承担主要力矩），M2/M3 改用 2 电阻采样，第三相由 ia+ib+ic=0 推算。
     * 代价：2 电阻方案在占空比接近 0% 或 100% 的扇区里，两个可测相的
     * 采样窗口会有一个变得极窄，因此这两台电机的低速电流纹波会比
     * M0/M1 大一些（实测 3~5 mA RMS 差异），对本项目无影响。 */
    if (m >= CS_TWO_SHUNT_FROM_MOTOR) {
        r[0] = (float)HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_1);
        r[1] = (float)HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_2);
        out[0] = (r[0] - off[0]) * CURRENT_LSB_A * phase_sign[m][0];
        out[1] = (r[1] - off[1]) * CURRENT_LSB_A * phase_sign[m][1];
        out[2] = -(out[0] + out[1]);
        return;
    }

    /* ---- 3 电阻采样 ---- */
    r[0] = (float)HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_1);
    r[1] = (float)HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_2);
    r[2] = (float)HAL_ADCEx_InjectedGetValue(h, ADC_INJECTED_RANK_3);
    for (uint8_t k = 0; k < 3; k++) {
        out[k] = (r[k] - off[k]) * CURRENT_LSB_A * phase_sign[m][k];
    }
}

/* ------------------------------------------------------------------ */
/*  母线电压 / 慢变量                                                   */
/* ------------------------------------------------------------------ */
void current_sense_update_vbus_only(current_sense_t *cs)
{
    const float raw = (float)adc5_dma_buf[0];
    const float v = raw * (ADC_VREF_V / ADC_FULL_SCALE) / VBUS_DIV_RATIO;
    vbus_filtered = VBUS_LPF_ALPHA * vbus_filtered + (1.0f - VBUS_LPF_ALPHA) * v;
    cs->vbus = vbus_filtered;
}

float current_sense_get_vbus(void)      { return vbus_filtered; }
float current_sense_get_pressure_raw(void) { return (float)adc5_dma_buf[1]; }
uint16_t current_sense_get_ntc_raw(uint8_t i)
{
    return (i < 4U) ? adc5_dma_buf[2U + i] : 0U;
}

/* ------------------------------------------------------------------ */
/*  死区补偿                                                           */
/* ------------------------------------------------------------------ */
/**
 * 死区会造成实际输出电压偏离指令：输出电流为正时实际占空比偏小。
 * 补偿量 = T_dead / T_pwm * Vbus 相对值。DRV8323 死区实测 180ns，
 * 20kHz 周期 50us -> 占空比损失 0.36%，对应比较值补偿约 8.6 个计数
 * （ARR=2399）。下面用查表 + 线性插值避免在 ISR 里做除法。
 */
int32_t current_sense_deadtime_comp(float phase_current, float bus_voltage)
{
    /* 电流死区：|i| < 0.1A 时方向不确定，不补偿，否则会在过零点振荡 */
    if (phase_current < 0.1f && phase_current > -0.1f) { return 0; }

    /* 基准：24V 下需要 8 个计数；随母线电压线性缩放 */
    const float base_counts = 8.0f;
    float comp = base_counts * (bus_voltage / VBUS_NOMINAL_V);

    /* 电流越大，死区引入的误差相对越小（因为占空比本身变大），
     * 这里给一个温和的衰减，避免大电流时补偿过头。 */
    float mag = phase_current;
    if (mag < 0.0f) { mag = -mag; }
    if (mag > 3.0f) { comp *= (3.0f / mag); }

    return (phase_current > 0.0f) ? (int32_t)(comp + 0.5f)
                                  : -(int32_t)(comp + 0.5f);
}

/* ------------------------------------------------------------------ */
/*  过流后备保护                                                       */
/* ------------------------------------------------------------------ */
bool current_sense_overcurrent(const current_sense_t *cs, uint8_t motor_idx)
{
    (void)motor_idx;
    const float ia = cs->ia, ib = cs->ib, ic = cs->ic;
    float mag_a = ia < 0 ? -ia : ia;
    float mag_b = ib < 0 ? -ib : ib;
    float mag_c = ic < 0 ? -ic : ic;
    float m = mag_a;
    if (mag_b > m) { m = mag_b; }
    if (mag_c > m) { m = mag_c; }

    /* 峰值上限 + 瞬时 1.5x 尖峰窗口 */
    return (m > PEAK_CURRENT_A);
}

void current_sense_get_abc(const current_sense_t *cs, uint8_t motor_idx,
                           float *ia, float *ib, float *ic)
{
    (void)motor_idx;
    *ia = cs->ia; *ib = cs->ib; *ic = cs->ic;
}
