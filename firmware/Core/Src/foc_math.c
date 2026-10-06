/**
 * @file    foc_math.c
 * @brief   CORDIC 三角函数的封装与回退实现。
 *
 * STM32G4 的 CORDIC 协处理器支持"余弦+正弦同时出"（FUNC=COSINE，一次 2 个结果），
 * 192 个周期出结果，且不占用 FPU。相比 sinf()+cosf() 的约 400 周期有数量级收益。
 *
 * 注意 CORDIC 的输入是 q1.31 归一化到 [-1, 1) 表示的角度（-π..π），
 * 输出是 q1.31 的 cos/sin 值。这个归一化容易搞错 —— 单位是"圈"，不是弧度，
 * 即 angle_q31 = rad / π * 2^31。下面严格按照这个关系换算。
 */
#include "foc_math.h"

extern CORDIC_HandleTypeDef hcordic;

/* CORDIC 在 G4 的句柄；未初始化时（例如在电机使能之前）回退到 libm。
 * 这里用一个简单的标志而不是查 HAL 状态，因为 HAL 的 CORDIC 状态位在
 * 每次计算后会被清零，不能作为"已配置"的依据。 */
static bool s_cordic_ready = false;

void fast_math_init(void)
{
    CORDIC_ConfigTypeDef cfg = {0};
    cfg.Function         = CORDIC_FUNCTION_COSINE;
    cfg.Precision        = CORDIC_PRECISION_6CYCLES;
    cfg.Scale            = CORDIC_SCALE_0;
    cfg.NbWrite          = CORDIC_NBWRITE_1;
    cfg.NbRead           = CORDIC_NBREAD_2;     /* 一次读 cos 与 sin */
    cfg.InSize           = CORDIC_INSIZE_32BITS;
    cfg.OutSize          = CORDIC_OUTSIZE_32BITS;

    if (HAL_CORDIC_Configure(&hcordic, &cfg) != HAL_OK) {
        s_cordic_ready = false;
        return;
    }
    s_cordic_ready = true;
}

/* ------------------------------------------------------------------ */
/*  CORDIC 批量计算：一次配置算一对 (sin, cos)                          */
/* ------------------------------------------------------------------ */
void fast_sincos(float rad, float *s, float *c)
{
    if (s_cordic_ready) {
        int32_t out[2];
        float a = wrap_pm_pi(rad);
        int32_t arg = (int32_t)(a * (1.0f / (float)M_PI) * 2147483648.0f);

        if (HAL_CORDIC_Calculate(&hcordic, &arg, out, 1U, 2U) == HAL_OK) {
            /* HAL 按 CORDIC 结果寄存器顺序返回：RES1 = cos, RES2 = sin */
            const float cosv = (float)out[0] / 2147483648.0f;
            const float sinv = (float)out[1] / 2147483648.0f;
            *s = sinv;
            *c = cosv;
            return;
        }
    }
    /* 回退：CORDIC 未就绪（上电初期 / 单测环境） */
    *s = sinf(rad);
    *c = cosf(rad);
}

float fast_sin(float rad)
{
    float s, c;
    fast_sincos(rad, &s, &c);
    return s;
}

float fast_cos(float rad)
{
    float s, c;
    fast_sincos(rad, &s, &c);
    return c;
}
