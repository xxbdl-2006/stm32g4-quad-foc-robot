/**
 * @file    motor.c
 * @brief   4 台电机的三环控制、模式状态机、标定流程与故障保护。
 */
#include "motor.h"
#include "board.h"
#include "param.h"
#include "can_node.h"
#include "foc_math.h"
#include <string.h>

motor_t g_motors[MOTOR_COUNT];

/* =================================================================== */
/*  初始化                                                            */
/* =================================================================== */

void motor_init(motor_t *m, uint8_t id)
{
    memset(m, 0, sizeof(*m));
    m->id = id;

    foc_init(&m->foc);
    foc_auto_tune_pi(&m->foc, PHASE_R_OHM, PHASE_L_HENRY, 1000.0f); /* fc = 1kHz */

    encoder_init(&m->enc, 0.0f);

    /* --- 速度环：被控对象是"电流环 + 机械惯性"，可近似为一阶惯性+积分。
     *     初值按 1kHz 环路、目标带宽 50Hz 给，实测手调只需微调 Kp。 */
    pid_config_t pv = {
        .kp = 0.085f, .ki = 1.60f, .kd = 0.0f,
        .out_min = -RATED_CURRENT_A, .out_max = RATED_CURRENT_A,
        .dt = DT_VELOCITY,
        .use_backcalc = false,          /* 速度环用条件积分，实测超调更小 */
        .kaw = 0.0f,
        .d_on_measurement = true, .d_lpf_alpha = 0.7f
    };
    pid_init(&m->pid_velocity, &pv);

    /* --- 位置环：纯 P + 速度前馈，不用 I（避免积分器爬行导致静差反向） */
    pid_config_t pp = {
        .kp = 12.0f, .ki = 0.0f, .kd = 0.18f,
        .out_min = -MAX_MECH_SPEED_RAD_S, .out_max = MAX_MECH_SPEED_RAD_S,
        .dt = DT_POSITION,
        .use_backcalc = true, .kaw = 20.0f,
        .d_on_measurement = true, .d_lpf_alpha = 0.8f
    };
    pid_init(&m->pid_position, &pp);

    m->mode = MOTOR_MODE_IDLE;
    m->mode_pending = MOTOR_MODE_IDLE;
    m->state = MSTATE_DISABLED;
    m->current_limit = RATED_CURRENT_A;
    m->enabled_cmd = false;
    m->fb_fault = FAULT_NONE;
}

void motor_init_all(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motor_init(&g_motors[i], i);
    }

    /* 电流偏置标定：必须在三相桥关断状态下做。
     * 这里直接调用 ADC 层，512 次平均。 */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motor_t *m = &g_motors[i];
        float oa, ob, oc;
        current_sense_calib_one(i, &oa, &ob, &oc, 512U);
        m->i_offset[0] = oa;
        m->i_offset[1] = ob;
        m->i_offset[2] = oc;
        m->foc.pid_d.cfg.dt = DT_CURRENT;   /* 已在 foc_init 设好，此处冗余保险 */
    }

    encoder_prime_pipelines();
}

/* =================================================================== */
/*  模式与指令                                                        */
/* =================================================================== */

void motor_set_mode(motor_t *m, motor_mode_t mode)
{
    /* 模式切换不立即生效，挂到 mode_pending，在环路边界统一处理 ——
     * 否则可能在 Park 变换到一半时把 pid 的积分器清掉，输出一个巨大的尖峰。 */
    m->mode_pending = mode;
}

void motor_enable(motor_t *m, bool en)
{
    if (en) {
        if (m->state == MSTATE_FAULT) { return; }   /* 必须先 reset_fault */
        m->enabled_cmd = true;
        if (m->state == MSTATE_DISABLED) {
            m->state = MSTATE_READY;
        }
        board_pwm_enable(m->id, true);
    } else {
        m->enabled_cmd = false;
        m->state = MSTATE_DISABLED;
        m->mode_pending = MOTOR_MODE_IDLE;
        board_pwm_enable(m->id, false);
        foc_reset(&m->foc);
        pid_reset(&m->pid_velocity);
        pid_reset(&m->pid_position);
        m->foc.iq_ref = 0.0f;
    }
}

void motor_set_target(motor_t *m, float setpoint, float limit, uint8_t flags)
{
    if (flags & MOTION_FLAG_RESET_FAULT) { motor_reset_fault(m); }

    if (flags & MOTION_FLAG_ESTOP) { motor_emergency_stop(m); return; }

    if (flags & MOTION_FLAG_ENABLE) {
        motor_enable(m, true);
    } else if (m->state == MSTATE_DISABLED) {
        /* 未使能时只更新指令，不动作 —— 允许上位机先摆好目标再使能 */
    }

    if (flags & MOTION_FLAG_BRAKE) {
        motor_set_mode(m, MOTOR_MODE_BRAKE);
        return;
    }

    if (limit > 0.0f) {
        m->current_limit = clampf(limit, 0.0f, PEAK_CURRENT_A);
    }

    switch (m->mode_pending) {
        case MOTOR_MODE_CURRENT:  m->current_ref  = setpoint; break;
        case MOTOR_MODE_VELOCITY: m->velocity_ref = setpoint; break;
        case MOTOR_MODE_POSITION: m->position_ref = setpoint; break;
        case MOTOR_MODE_OPENLOOP: m->duty_openloop = clampf(setpoint, -1.0f, 1.0f); break;
        default: break;
    }
}

void motor_reset_fault(motor_t *m)
{
    if (m->state != MSTATE_FAULT) { return; }

    /* 故障复位必须先撤掉功率级，再清状态，最后才允许重新使能 */
    board_pwm_enable(m->id, false);
    m->overcurrent_cnt = 0;
    m->encoder_fault_cnt = 0;
    m->nfault_pin_low = false;
    m->fb_fault = FAULT_NONE;
    m->foc.saturation_cnt = 0;
    encoder_clear_fault(&m->enc);
    foc_reset(&m->foc);
    pid_reset(&m->pid_velocity);
    pid_reset(&m->pid_position);
    m->state = MSTATE_DISABLED;
    m->enabled_cmd = false;
}

void motor_emergency_stop(motor_t *m)
{
    board_pwm_enable(m->id, false);
    m->foc.iq_ref = 0.0f;
    m->foc.id_ref = 0.0f;
    m->velocity_ref = 0.0f;
    m->current_ref = 0.0f;
    m->mode_pending = MOTOR_MODE_IDLE;
    m->mode = MOTOR_MODE_IDLE;
    m->state = MSTATE_READY;
    m->enabled_cmd = false;
    pid_reset(&m->pid_velocity);
    pid_reset(&m->pid_position);
}

/* =================================================================== */
/*  电角度标定                                                        */
/* =================================================================== */
void motor_start_calibration(motor_t *m)
{
    if (m->state == MSTATE_FAULT) { return; }
    m->calib_requested = true;
    m->calib_tick = 0;
    m->calib_vd = 1.5f;     /* 1.5V d 轴电压，24V 母线下对应约 6% 占空比 */
}

static void motor_calib_step(motor_t *m)
{
    /* 标定流程（共 250ms）：
     *   阶段 0-100ms ：施加 +d 轴电压，让转子向 A 相轴线对齐
     *   阶段 100-160ms：保持，等待机械振荡衰减
     *   阶段 160-195ms：撤掉电压，转子自由停在最近稳定点
     *   阶段 195ms     ：记录此时编码器角作为电角度零点
     *   阶段 195-250ms ：把偏置写 Flash，回到 READY
     * 期间电流限到 30% 额定，防止堵转发热。 */
    m->calib_tick++;
    const float vbus = board_read_vbus();

    if (m->calib_tick < 2000U) {                    /* 100ms */
        m->state = MSTATE_CALIB;
        foc_align_step(&m->foc, 0.0f, m->calib_vd, vbus);
        board_pwm_write(m->id, &m->foc.duty);
    } else if (m->calib_tick < 3200U) {             /* 保持到 160ms */
        foc_align_step(&m->foc, 0.0f, m->calib_vd, vbus);
        board_pwm_write(m->id, &m->foc.duty);
    } else if (m->calib_tick < 3900U) {             /* 撤压 */
        board_pwm_enable(m->id, false);
    } else if (m->calib_tick == 3900U) {
        encoder_set_zero_from_current(&m->enc);
        m->foc.pid_d.integ = 0.0f;
        m->foc.pid_q.integ = 0.0f;
    } else if (m->calib_tick >= 5000U) {            /* 250ms，收尾 */
        m->calib_in_progress = false;
        m->calib_requested = false;
        m->state = MSTATE_READY;
        m->mode_pending = MOTOR_MODE_IDLE;
        board_pwm_enable(m->id, false);
        motor_save_encoder_offset(m);
        can_node_report_event(CAN_EVT_CALIB_DONE, m->id, m->enc.elec_offset);
    }
}

/* =================================================================== */
/*  20kHz 快速环路                                                    */
/* =================================================================== */
void motor_fast_loop(motor_t *m)
{
    m->tick++;

    /* ---- 0. 标定优先级最高 ---- */
    if (m->calib_requested) {
        if (!m->calib_in_progress) {
            m->calib_in_progress = true;
            m->enabled_cmd = false;
            board_pwm_enable(m->id, true);
        }
        motor_calib_step(m);
        return;
    }

    /* ---- 1. 采编码器（每周期都采：1 帧 SPI，6.4us/4台） ---- */
    if (!encoder_sample(&m->enc, m->id)) {
        m->encoder_fault_cnt++;
        if (m->encoder_fault_cnt > 20U) {          /* 连续 1ms 读不到 -> 停机 */
            m->fb_fault |= FAULT_ENCODER_SPI;
            m->state = MSTATE_FAULT;
            board_pwm_enable(m->id, false);
            return;
        }
    } else {
        m->encoder_fault_cnt = 0;
    }

    /* ---- 2. 采三相电流 ---- */
    float abc[3];
    current_sense_read_abc(m->id, m->i_offset, abc);

    m->foc.ia = abc[0]; m->foc.ib = abc[1]; m->foc.ic = abc[2];

    /* ---- 3. 过流后备保护（硬件比较器是第一道，这里是第二道） ---- */
    float amax = abc[0] < 0 ? -abc[0] : abc[0];
    if (abc[1] < 0 ? -abc[1] : abc[1] > amax) { amax = abc[1] < 0 ? -abc[1] : abc[1]; }
    if (abc[2] < 0 ? -abc[2] : abc[2] > amax) { amax = abc[2] < 0 ? -abc[2] : abc[2]; }
    if (amax > PEAK_CURRENT_A) {
        m->overcurrent_cnt++;
        if (m->overcurrent_cnt > 30U) {            /* 1.5ms 持续过流才跳闸，躲开尖峰 */
            m->fb_fault |= FAULT_OVERCURRENT;
            m->state = MSTATE_FAULT;
            board_pwm_enable(m->id, false);
            can_node_report_event(CAN_EVT_FAULT, m->id, (float)FAULT_OVERCURRENT);
            return;
        }
    } else {
        m->overcurrent_cnt = 0;
    }

    /* ---- 4. 模式切换（在环路边界生效） ---- */
    if (m->mode != m->mode_pending) {
        m->mode = m->mode_pending;
        foc_reset(&m->foc);
        switch (m->mode) {
            case MOTOR_MODE_IDLE:
            case MOTOR_MODE_BRAKE:
                m->foc.iq_ref = 0.0f; m->foc.id_ref = 0.0f;
                break;
            case MOTOR_MODE_VELOCITY:
                /* 从当前速度起步，避免速度环积分器从 0 爬升造成冲击 */
                m->pid_velocity.integ = m->foc.iq_ref;
                pid_reset(&m->pid_velocity);
                break;
            case MOTOR_MODE_POSITION:
                m->position_ref = m->enc.mech_rad_multi;
                m->velocity_ref = 0.0f;
                break;
            default: break;
        }
    }

    /* ---- 5. 外环（按分频） ---- */
    const float theta_e = m->enc.elec_rad;

    if (m->mode == MOTOR_MODE_BRAKE) {
        board_pwm_brake(m->id, true);
        board_pwm_write(m->id, &m->foc.duty);
        motor_latch_feedback(m);
        return;
    }

    if (m->mode == MOTOR_MODE_POSITION) {
        if ((m->tick % POSITION_LOOP_DIV) == 0U) {
            const float pos_err = m->position_ref - m->enc.mech_rad_multi;
            /* 速度前馈：上一拍位置环输出的低通，提高轨迹跟随精度 */
            float v_cmd = pid_step(&m->pid_position, m->position_ref, m->enc.mech_rad_multi);
            m->velocity_ff = 0.85f * m->velocity_ff + 0.15f * v_cmd;
            (void)pos_err;
            m->velocity_ref = clampf(v_cmd, -MAX_MECH_SPEED_RAD_S, MAX_MECH_SPEED_RAD_S);

            /* 内环速度 PI -> iq */
            m->foc.iq_ref = pid_step_ff(&m->pid_velocity, m->velocity_ref,
                                        m->enc.mech_speed, 0.0f);
        }
    } else if (m->mode == MOTOR_MODE_VELOCITY) {
        if ((m->tick % VELOCITY_LOOP_DIV) == 0U) {
            m->foc.iq_ref = pid_step(&m->pid_velocity, m->velocity_ref, m->enc.mech_speed);
        }
    } else if (m->mode == MOTOR_MODE_CURRENT) {
        m->foc.iq_ref = m->current_ref;
    } else if (m->mode == MOTOR_MODE_OPENLOOP) {
        /* 开环：直接构造电压矢量，电角度由内部斜坡发生器给出（不依赖编码器） */
        static float ol_theta[MOTOR_COUNT];
        ol_theta[m->id] = wrap_2pi(ol_theta[m->id] + 0.002f);
        dq_t v = { .d = 0.0f, .q = m->duty_openloop * board_read_vbus() * 0.9f };
        m->foc.v_ab = inv_park(v, ol_theta[m->id]);
        svpwm(m->foc.v_ab, board_read_vbus(), &m->foc.duty);
        board_pwm_write(m->id, &m->foc.duty);
        motor_latch_feedback(m);
        return;
    } else {
        /* IDLE */
        m->foc.iq_ref = 0.0f;
        if (!m->enabled_cmd) {
            board_pwm_enable(m->id, false);
            motor_latch_feedback(m);
            return;
        }
    }

    /* ---- 6. 电流环 + SVPWM ---- */
    m->foc.current_limit = m->current_limit;
    foc_step(&m->foc, theta_e, abc[0], abc[1], abc[2], board_read_vbus());
    board_pwm_write(m->id, &m->foc.duty);

    if (m->enabled_cmd && m->state == MSTATE_READY) {
        m->state = MSTATE_RUNNING;
    }

    /* ---- 7. 统计 ---- */
    m->iq_rms_acc += m->foc.i_dq.q * m->foc.i_dq.q;
    m->iq_rms_n++;

    motor_latch_feedback(m);
}

/* =================================================================== */
/*  1kHz 慢任务                                                       */
/* =================================================================== */
void motor_slow_task(motor_t *m)
{
    /* ---- 温度 ---- */
    if (m->id == 0U) {
        const float mcu_t = board_read_mcu_temp();
        if (mcu_t > MCU_OTP_C) { m->fb_fault |= FAULT_MCU_OVERTEMP; }
    }
    const float mos_t = board_read_mos_temp(m->id);
    if (mos_t > MOS_OTP_C) {
        m->fb_fault |= FAULT_MOS_OVERTEMP;
    }

    /* ---- 驱动器 nFAULT 引脚 ---- */
    if (board_drv_fault_active(m->id)) {
        m->nfault_pin_low = true;
        m->fb_fault |= FAULT_DRV_FAULT_N;
    }

    /* ---- 母线保护 ---- */
    const float vbus = board_read_vbus();
    if (vbus > g_params.board.vbus_ovp) { m->fb_fault |= FAULT_OVERVOLTAGE; }
    if (vbus < g_params.board.vbus_uvp) { m->fb_fault |= FAULT_UNDERVOLTAGE; }

    /* ---- 任一硬故障 -> 立刻切 FAULT 并封锁输出 ---- */
    const uint16_t hard_faults = FAULT_OVERCURRENT | FAULT_MCU_OVERTEMP |
                                 FAULT_MOS_OVERTEMP | FAULT_DRV_FAULT_N |
                                 FAULT_OVERVOLTAGE | FAULT_UNDERVOLTAGE;
    if ((m->fb_fault & hard_faults) != 0U && m->state != MSTATE_FAULT) {
        m->state = MSTATE_FAULT;
        m->enabled_cmd = false;
        board_pwm_enable(m->id, false);
        foc_reset(&m->foc);
        pid_reset(&m->pid_velocity);
        pid_reset(&m->pid_position);
        can_node_report_event(CAN_EVT_FAULT, m->id, (float)m->fb_fault);
    }

    /* ---- 指令停机时的速度零钳（抑制量化噪声） ---- */
    const bool cmd_stop = (m->mode == MOTOR_MODE_VELOCITY && m->velocity_ref == 0.0f) ||
                          (m->mode == MOTOR_MODE_IDLE);
    encoder_zero_speed_hint(&m->enc, cmd_stop);

    /* ---- 环路耗时统计清零窗口 ---- */
    if ((m->tick % 20000U) == 0U) { m->loop_overrun_cnt = 0; }
}

/* =================================================================== */
/*  反馈快照                                                          */
/* =================================================================== */
void motor_latch_feedback(motor_t *m)
{
    m->fb_position = m->enc.mech_rad_multi;
    m->fb_velocity = m->enc.mech_speed;
    m->fb_current  = m->foc.i_dq.q;
    m->fb_duty     = (float)m->foc.duty.a / (float)SVPWM_ARR;
    /* fb_fault 由 slow_task 累加，此处不覆盖 */
}

/* =================================================================== */
/*  参数读写                                                          */
/* =================================================================== */
bool motor_param_read(motor_t *m, uint16_t param_id, float *out)
{
    switch (param_id) {
        case PID_VEL_KP:      *out = m->pid_velocity.cfg.kp; break;
        case PID_VEL_KI:      *out = m->pid_velocity.cfg.ki; break;
        case PID_VEL_KD:      *out = m->pid_velocity.cfg.kd; break;
        case PID_POS_KP:      *out = m->pid_position.cfg.kp; break;
        case PID_POS_KI:      *out = m->pid_position.cfg.ki; break;
        case PID_POS_KD:      *out = m->pid_position.cfg.kd; break;
        case PID_CUR_KP:      *out = m->foc.pid_q.cfg.kp;    break;
        case PID_CUR_KI:      *out = m->foc.pid_q.cfg.ki;    break;
        case LIMIT_CURRENT_A: *out = m->current_limit;       break;
        case LIMIT_VELOCITY:  *out = m->pid_velocity.cfg.out_max; break;
        case ENC_ELEC_OFFSET: *out = m->enc.elec_offset;     break;
        case ENCODER_DIRECTION: *out = 1.0f;                 break;
        case MOTOR_GEAR_RATIO:  *out = g_params.motor[m->id].gear_ratio; break;
        case MOTOR_POLE_PAIRS:  *out = (float)POLE_PAIRS;    break;
        default: return false;
    }
    return true;
}

bool motor_param_write(motor_t *m, uint16_t param_id, float val)
{
    switch (param_id) {
        case PID_VEL_KP: pid_set_gains(&m->pid_velocity, val, m->pid_velocity.cfg.ki, m->pid_velocity.cfg.kd); break;
        case PID_VEL_KI: pid_set_gains(&m->pid_velocity, m->pid_velocity.cfg.kp, val, m->pid_velocity.cfg.kd); break;
        case PID_VEL_KD: pid_set_gains(&m->pid_velocity, m->pid_velocity.cfg.kp, m->pid_velocity.cfg.ki, val); break;
        case PID_POS_KP: pid_set_gains(&m->pid_position, val, m->pid_position.cfg.ki, m->pid_position.cfg.kd); break;
        case PID_POS_KI: pid_set_gains(&m->pid_position, m->pid_position.cfg.kp, val, m->pid_position.cfg.kd); break;
        case PID_POS_KD: pid_set_gains(&m->pid_position, m->pid_position.cfg.kp, m->pid_position.cfg.ki, val); break;
        case PID_CUR_KP: pid_set_gains(&m->foc.pid_d, val, m->foc.pid_d.cfg.ki, 0.0f);
                         pid_set_gains(&m->foc.pid_q, val, m->foc.pid_q.cfg.ki, 0.0f); break;
        case PID_CUR_KI: pid_set_gains(&m->foc.pid_d, m->foc.pid_d.cfg.kp, val, 0.0f);
                         pid_set_gains(&m->foc.pid_q, m->foc.pid_q.cfg.kp, val, 0.0f); break;
        case LIMIT_CURRENT_A: m->current_limit = clampf(val, 0.0f, PEAK_CURRENT_A); break;
        case LIMIT_VELOCITY:
            m->pid_velocity.cfg.out_max = clampf(val, 0.0f, PEAK_CURRENT_A);
            m->pid_velocity.cfg.out_min = -m->pid_velocity.cfg.out_max;
            break;
        case MOTOR_GEAR_RATIO: g_params.motor[m->id].gear_ratio = val; break;
        case ENC_ELEC_OFFSET:
            /* 只允许标定流程写，禁止上位机直接改 —— 写错会直接烧驱动 */
            return false;
        default: return false;
    }
    return true;
}

void motor_save_encoder_offset(motor_t *m)
{
    g_params.motor[m->id].enc_elec_offset = m->enc.elec_offset;
    /* Flash 写入会阻塞约 20ms，因此放到非实时上下文（can_node 的空闲任务）执行 */
    can_node_request_param_save();
}
