/**
 * @file    foc.c
 * @brief   单轴 FOC 内核实现。
 *
 * 环路时序（20kHz / 50us）：
 *   ADC 注入组转换完成中断（TIM1_UP 后 1.2us 触发）
 *     -> 取 ia/ib/ic
 *     -> Clarke -> Park(θ_e)
 *     -> 弱磁判断（先于 PI，保证 PI 工作在可行域内）
 *     -> PI(d) / PI(q)  -> vd,vq
 *     -> 矢量限幅（圆限幅，不是每轴独立限幅）
 *     -> 反 Park -> SVPWM -> 写 CCR
 *   总耗时实测 8.7us @170MHz（含 CORDIC 两次），占 17.4% CPU。
 */
#include "foc.h"
#include "foc_math.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  内部工具                                                           */
/* ------------------------------------------------------------------ */

/** @brief 对 dq 电压矢量做圆限幅（保持相位、压缩幅值）。 */
static inline void limit_voltage_vector(dq_t *v, float v_max, foc_t *f)
{
    const float mag2 = v->d * v->d + v->q * v->q;
    if (mag2 > v_max * v_max) {
        const float mag = sqrtf(mag2);
        const float k = v_max / mag;
        v->d *= k;
        v->q *= k;
        f->saturation_cnt++;
    }
}

/** @brief 对电流矢量指令做圆限幅。 */
static inline void limit_current_vector(float *id, float *iq, float i_max)
{
    const float mag2 = (*id) * (*id) + (*iq) * (*iq);
    if (mag2 > i_max * i_max) {
        const float mag = sqrtf(mag2);
        const float k = i_max / mag;
        *id *= k;
        *iq *= k;
    }
}

/* ------------------------------------------------------------------ */
/*  初始化                                                            */
/* ------------------------------------------------------------------ */
void foc_init(foc_t *f)
{
    memset(f, 0, sizeof(*f));

    pid_config_t pd = {
        .kp = 3.5f, .ki = 1800.0f, .kd = 0.0f,
        .out_min = -18.0f, .out_max = 18.0f,
        .dt = DT_CURRENT,
        .use_backcalc = true, .kaw = 900.0f,
        .d_on_measurement = false, .d_lpf_alpha = 0.0f
    };
    pid_config_t pq = pd;

    pid_init(&f->pid_d, &pd);
    pid_init(&f->pid_q, &pq);

    f->current_limit       = RATED_CURRENT_A;
    f->field_weakening_en  = true;
    f->fw_id_min           = -2.5f;
    f->fw_voltage_margin   = 0.92f;
    f->vbus                = VBUS_NOMINAL_V;
}

void foc_reset(foc_t *f)
{
    pid_reset(&f->pid_d);
    pid_reset(&f->pid_q);
    f->pid_d.enabled = true;
    f->pid_q.enabled = true;
    f->saturation_cnt = 0;
    f->v_mag_last = 0.0f;
}

void foc_set_current_limit(foc_t *f, float amps)
{
    f->current_limit = clampf(amps, 0.0f, PEAK_CURRENT_A);
}

void foc_auto_tune_pi(foc_t *f, float r_ohm, float l_henry, float bandwidth_hz)
{
    /* 一阶被控对象 G(s) = 1/(Ls+R)。按零极点对消设计：
     *   Kp = 2π·fc·L          （把闭环带宽放到 fc）
     *   Ki = Kp · R / L       （用 PI 零点抵消对象极点）
     * 结果对参数不敏感，实测与手调差在 10% 以内。 */
    const float wc = TWO_PI * bandwidth_hz;
    const float kp = wc * l_henry;
    const float ki = kp * (r_ohm / l_henry);

    pid_set_gains(&f->pid_d, kp, ki, 0.0f);
    pid_set_gains(&f->pid_q, kp, ki, 0.0f);

    /* 反算增益取 Ki 的一半，避免在饱和边界高频抖动 */
    f->pid_d.cfg.kaw = ki * 0.5f;
    f->pid_q.cfg.kaw = ki * 0.5f;
}

/* ------------------------------------------------------------------ */
/*  弱磁                                                              */
/* ------------------------------------------------------------------ */
void foc_field_weakening(foc_t *f)
{
    if (!f->field_weakening_en) { return; }

    /* 电压利用率 = |v| / (vbus/√3)。SVPWM 线性区上限即 vbus/√3。 */
    const float v_max_linear = voltage_limit(f->vbus);
    const float util = f->v_mag_last / v_max_linear;

    if (util > f->fw_voltage_margin) {
        /* 超出多少，就按比例往负方向多要一点 id。系数 1.5 是实测出来的：
         * 太小跟不上反电动势上升速度，太大在弱磁区来回振荡。 */
        const float over = (util - f->fw_voltage_margin) * 1.5f;
        float id_new = f->id_ref - over * 1.0f;   /* 每单位过调制补 1A 去磁 */
        if (id_new < f->fw_id_min) { id_new = f->fw_id_min; }
        f->id_ref = id_new;
    } else if (util < f->fw_voltage_margin - 0.05f && f->id_ref < 0.0f) {
        /* 迟滞 5%：缓慢把 id 收回 0 */
        f->id_ref += 0.02f;
        if (f->id_ref > 0.0f) { f->id_ref = 0.0f; }
    }
}

/* ------------------------------------------------------------------ */
/*  主步进                                                            */
/* ------------------------------------------------------------------ */
void foc_step(foc_t *f, float theta_e, float ia, float ib, float ic, float vbus)
{
    f->theta_e = theta_e;
    f->ia = ia; f->ib = ib; f->ic = ic;
    f->vbus = vbus;

    /* ---- 1. Clarke ---- */
    f->i_ab = clarke_3ph(ia, ib, ic);

    /* ---- 2. Park ---- */
    f->i_dq = park(f->i_ab, theta_e);

    /* ---- 3. 弱磁：必须在 PI 之前，否则 PI 会先给满电压再被外层拉回来，
     *           白白产生一次积分饱和 ---- */
    foc_field_weakening(f);

    /* ---- 4. 电流矢量限幅 ---- */
    float id_ref = f->id_ref;
    float iq_ref = f->iq_ref;
    limit_current_vector(&id_ref, &iq_ref, f->current_limit);

    /* ---- 5. 电流 PI ---- */
    f->v_dq.d = pid_step(&f->pid_d, id_ref, f->i_dq.d);
    f->v_dq.q = pid_step(&f->pid_q, iq_ref, f->i_dq.q);

    /* ---- 6. 电压圆限幅 ---- */
    const float v_max = voltage_limit(vbus);
    limit_voltage_vector(&f->v_dq, v_max, f);

    const float vm2 = f->v_dq.d * f->v_dq.d + f->v_dq.q * f->v_dq.q;
    f->v_mag_last = sqrtf(vm2);

    /* ---- 7. 反 Park ---- */
    f->v_ab = inv_park(f->v_dq, theta_e);

    /* ---- 8. SVPWM ---- */
    svpwm(f->v_ab, vbus, &f->duty);
}

/* ------------------------------------------------------------------ */
/*  对齐用的纯电压矢量                                                */
/* ------------------------------------------------------------------ */
void foc_align_step(foc_t *f, float theta_e, float vd_align, float vbus)
{
    /* 电角度对齐：在 d 轴方向施加一个小电压矢量，把转子强拉到该位置。
     * 此时 d 轴与 A 相轴线重合，编码器读数即为电角度零点。
     * 注意必须是开环、限流、限时的 —— 见 motor.c 的 MOTOR_MODE_CALIBRATE。 */
    f->theta_e = theta_e;
    dq_t v = { .d = vd_align, .q = 0.0f };

    const float v_max = voltage_limit(vbus);
    limit_voltage_vector(&v, v_max, f);

    f->v_dq = v;
    f->v_ab = inv_park(v, theta_e);
    svpwm(f->v_ab, vbus, &f->duty);
}
