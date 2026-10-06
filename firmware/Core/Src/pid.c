/**
 * @file    pid.c
 */
#include "pid.h"
#include <string.h>

static inline float clampf_local(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void pid_init(pid_t *p, const pid_config_t *cfg)
{
    memset(p, 0, sizeof(*p));
    p->cfg = *cfg;
    p->enabled = true;
}

void pid_reset(pid_t *p)
{
    p->integ     = 0.0f;
    p->d_state   = 0.0f;
    p->err       = 0.0f;
    p->err_prev  = 0.0f;
    p->fdb_prev  = p->fdb;
    p->out       = 0.0f;
    p->out_unsat = 0.0f;
}

void pid_freeze(pid_t *p)
{
    /* 保留 integ / d_state，只停止累加。用于故障恢复时避免"积分清零 -> 冲击"。 */
    p->enabled = false;
}

void pid_set_gains(pid_t *p, float kp, float ki, float kd)
{
    p->cfg.kp = kp;
    p->cfg.ki = ki;
    p->cfg.kd = kd;
}

float pid_step(pid_t *p, float ref, float fdb)
{
    return pid_step_ff(p, ref, fdb, 0.0f);
}

float pid_step_ff(pid_t *p, float ref, float fdb, float ff)
{
    const pid_config_t *c = &p->cfg;

    if (!p->enabled) {
        p->fdb = fdb;
        p->ref = ref;
        p->out = clampf_local(ff, c->out_min, c->out_max);
        return p->out;
    }

    p->ref = ref;
    p->fdb = fdb;

    const float err = ref - fdb;
    p->err = err;

    /* ---- 比例 ---- */
    float up = c->kp * err;

    /* ---- 积分（先算，饱和后回退） ---- */
    const float dt = c->dt;
    p->integ += c->ki * err * dt;

    /* ---- 微分 ---- */
    float ud = 0.0f;
    if (c->kd != 0.0f) {
        const float d_src = c->d_on_measurement ? -fdb : err;
        const float d_raw = (d_src - (c->d_on_measurement ? -p->fdb_prev : p->err_prev)) / dt;
        /* 一阶低通：d_state = a*d_state + (1-a)*d_raw */
        const float a = c->d_lpf_alpha;
        p->d_state = a * p->d_state + (1.0f - a) * d_raw;
        ud = c->kd * p->d_state;
    }

    float out_unsat = up + p->integ + ud + ff;
    float out = clampf_local(out_unsat, c->out_min, c->out_max);

    /* ---- 抗饱和 ---- */
    const float sat_err = out - out_unsat;      /* 被削掉的量，非零即进入饱和 */
    if (sat_err != 0.0f) {
        if (c->use_backcalc && c->kaw > 0.0f) {
            /* 反算：把超出的部分按 kaw 速率倒灌回积分器，使积分器跟踪实际输出 */
            p->integ += c->kaw * sat_err * dt;
            /* 再次限位，避免反算本身把积分器推到反向饱和 */
            p->integ = clampf_local(p->integ, c->out_min - ff, c->out_max - ff);
        } else {
            /* 条件积分：只在误差会把输出推离饱和区时才累加 —— 由于已经累加过了，
             * 这里做一次回退即可。 */
            if ((out_unsat > c->out_max && err > 0.0f) ||
                (out_unsat < c->out_min && err < 0.0f)) {
                p->integ -= c->ki * err * dt;
            }
        }
    }

    p->err_prev = err;
    p->fdb_prev = fdb;
    p->out_unsat = out_unsat;
    p->out = out;
    return out;
}
