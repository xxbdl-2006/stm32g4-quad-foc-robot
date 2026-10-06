/**
 * @file    foc_math.h
 * @brief   FOC 坐标变换与 SVPWM 的定点/浮点混合内核。
 *
 * 设计取舍：
 *  - 三角函数全部走 CORDIC 协处理器（STM32G4 硬件加速，92 cycle 出结果），
 *    不使用 math.h 的 sinf/cosf —— 后者在 170MHz 下约 120~200 cycle 且会拉 FPU 栈。
 *  - Clarke/Park 保持浮点，2/3 等幅值不变换，便于与 Simulink 模型逐点对齐。
 *  - SVPWM 输出为 1200 计数（TIM1 ARR=2399 中心对齐，半周期 1200）范围内的占空比。
 */
#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h>
#include <math.h>
#include "stm32g4xx_hal.h"
#include "foc_config.h"

#define SQRT3                       (1.7320508075688772f)
#define INV_SQRT3                   (0.5773502691896258f)
#define TWO_PI                      (6.283185307179586f)
#define ONE_BY_SQRT3_F              (0.5773502691896258f)

/* ---------------- 角度归一化 ---------------------------------------- */
/** @brief 把任意弧度折回 [0, 2π)，避免长时间运行后浮点精度退化。 */
static inline float wrap_2pi(float a)
{
    a = fmodf(a, TWO_PI);
    return (a < 0.0f) ? (a + TWO_PI) : a;
}

/** @brief 归一化到 (-π, π]，用于位置误差计算。 */
static inline float wrap_pm_pi(float a)
{
    a = fmodf(a + (float)M_PI, TWO_PI);
    if (a < 0.0f) { a += TWO_PI; }
    return a - (float)M_PI;
}

/* ---------------- CORDIC 三角函数 ----------------------------------- */
/* 这两支在 foc_math.c 中实现：优先调用 HAL 的 CORDIC 配置，失败时回退 libm。
 * 之所以封装而不是直接调 sinf，是因为代码要能在无 CORDIC 的 G431 上编译（单元测试）。 */
void  fast_math_init(void);
float fast_sin(float rad);
float fast_cos(float rad);
void  fast_sincos(float rad, float *s, float *c);

/* ---------------- Clarke / Park ------------------------------------- */
typedef struct { float alpha, beta; }   alphabeta_t;
typedef struct { float d, q; }          dq_t;

/** @brief Clarke 变换（等幅值）：iα = ia, iβ = (ia + 2ib)/√3。假定 ia+ib+ic=0。 */
static inline alphabeta_t clarke(float ia, float ib)
{
    alphabeta_t out;
    out.alpha = ia;
    out.beta  = (ia + 2.0f * ib) * ONE_BY_SQRT3_F;
    return out;
}

/** @brief Clarke 变换（三相版，用于非对称采样，保留 ic 以提高抗偏置能力）。 */
static inline alphabeta_t clarke_3ph(float ia, float ib, float ic)
{
    alphabeta_t out;
    out.alpha = (2.0f / 3.0f) * (ia - 0.5f * ib - 0.5f * ic);
    out.beta  = ONE_BY_SQRT3_F * (ib - ic);
    return out;
}

/** @brief Park 变换：把 αβ 旋转到随转子同步的 dq 系。 */
static inline dq_t park(alphabeta_t in, float theta_e)
{
    float s, c;
    fast_sincos(theta_e, &s, &c);
    dq_t out;
    out.d = in.alpha * c + in.beta * s;
    out.q = -in.alpha * s + in.beta * c;
    return out;
}

/** @brief 反 Park：把 dq 电压矢量旋回静止系。 */
static inline alphabeta_t inv_park(dq_t in, float theta_e)
{
    float s, c;
    fast_sincos(theta_e, &s, &c);
    alphabeta_t out;
    out.alpha = in.d * c - in.q * s;
    out.beta  = in.d * s + in.q * c;
    return out;
}

/* ---------------- SVPWM --------------------------------------------- */
/** 三相占空比，量纲为定时器比较值（0..SVPWM_ARR），已含死区补偿前的值。 */
typedef struct { uint16_t a, b, c; } pwm_duty_t;

#define SVPWM_ARR      2399U   /* TIM1 中心对齐 ARR，实际计数值域 0..2399 */
#define SVPWM_HALF     (SVPWM_ARR / 2U)  /* 1200，零矢量对应的中位点 */

/**
 * @brief 七段式 SVPWM，输出比较值。
 *
 * 采用"最小-最大注入"实现（等效于马鞍波调制），比查表法少 2 个扇区判断分支，
 * 且在过调制区（|v| 接近母线）行为连续，不会出现扇区切换毛刺。
 *
 * @param v_ab   αβ 系电压指令，单位伏特（已由电流环输出）
 * @param vbus   实测母线电压，伏特
 * @param duty   输出比较值
 */
static inline void svpwm(alphabeta_t v_ab, float vbus, pwm_duty_t *duty)
{
    /* 归一化到 [-0.5, 0.5] 的调制比 */
    const float inv_vbus = 1.0f / vbus;
    float ua = (v_ab.alpha) * inv_vbus;
    float ub = (-0.5f * v_ab.alpha + 0.5f * SQRT3 * v_ab.beta) * inv_vbus;
    float uc = (-0.5f * v_ab.alpha - 0.5f * SQRT3 * v_ab.beta) * inv_vbus;

    /* 最小-最大零序注入 */
    float umax = ua > ub ? ua : ub;
    if (uc > umax) { umax = uc; }
    float umin = ua < ub ? ua : ub;
    if (uc < umin) { umin = uc; }
    float uoff = -0.5f * (umax + umin);

    ua += uoff; ub += uoff; uc += uoff;

    /* 饱和到 [0,1]，再映射到比较值。留 2% 头部余量给死区与上升沿。 */
    const float lo = 0.02f, hi = 0.98f;
    ua = ua < lo ? lo : (ua > hi ? hi : ua);
    ub = ub < lo ? lo : (ub > hi ? hi : ub);
    uc = uc < lo ? lo : (uc > hi ? hi : uc);

    duty->a = (uint16_t)(ua * (float)SVPWM_ARR + 0.5f);
    duty->b = (uint16_t)(ub * (float)SVPWM_ARR + 0.5f);
    duty->c = (uint16_t)(uc * (float)SVPWM_ARR + 0.5f);
}

/* ---------------- 单位与限幅 ---------------------------------------- */
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float rad_s_to_rpm(float w)
{
    return w * 9.54929658551f;      /* 60 / 2π */
}

/** @brief 机械角 -> 电角度，含 7 对极折叠。 */
static inline float mech_to_elec(float theta_mech)
{
    return wrap_2pi(theta_mech * (float)POLE_PAIRS);
}

/** @brief 定子电压限幅：按母线电压的 95% 计算可输出矢量幅值上限（SVPWM 线性区）。 */
static inline float voltage_limit(float vbus)
{
    return 0.95f * vbus * ONE_BY_SQRT3_F;
}

#endif /* FOC_MATH_H */
