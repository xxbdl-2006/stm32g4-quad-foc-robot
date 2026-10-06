/**
 * @file    main.c
 * @brief   HWB-MC4G4 固件入口：初始化顺序、20kHz 控制环、主循环任务调度。
 *
 * 启动顺序（顺序本身是设计的一部分，不能随意调换）：
 *   1. HAL / 时钟 170MHz
 *   2. 参数从 Flash 载入（失败则用默认值）
 *   3. 三相桥全部关断（此刻还不知编码器零点，贸然开桥 = 飞车）
 *   4. ADC 校准 + 电流偏置标定（必须在桥关断时做）
 *   5. 编码器 SPI 流水线灌满
 *   6. 电角度零点从参数恢复；若无效则置"需要标定"标志
 *   7. 气泵 / 相机 IO 初始化
 *   8. CAN 节点启动 —— 放在最后，保证上电瞬间不会接受指令去驱动电机
 *   9. 开 TIM1 中断，开始 20kHz 环路
 */
#include "board.h"
#include "motor.h"
#include "current_sense.h"
#include "encoder.h"
#include "can_node.h"
#include "param.h"
#include "pump.h"
#include "cam_io.h"
#include "foc_math.h"

/* ---- HAL 句柄定义（CubeMX 生成的头文件里只声明，定义在此） ---- */
TIM_HandleTypeDef   htim1, htim2, htim4, htim8, htim15;
ADC_HandleTypeDef   hadc1, hadc2, hadc3, hadc4, hadc5;
SPI_HandleTypeDef   hspi1;
FDCAN_HandleTypeDef hfdcan1;
CORDIC_HandleTypeDef hcordic;

static current_sense_t g_cs;
static volatile uint32_t s_ctrl_tick;

/* ------------------------------------------------------------------ */
/*  20kHz 控制中断（TIM1 更新事件）                                    */
/* ------------------------------------------------------------------ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) { return; }

    /* 环路抖动监测：两次中断间隔偏离标称值超过 10% 计一次超时。
     * 用 DWT 而不是 HAL_GetTick()，后者分辨率只有 1ms。 */
    static uint32_t prev_cyc = 0;
    const uint32_t now_cyc = DWT->CYCCNT;
    const uint32_t nominal  = SystemCoreClock / CURRENT_LOOP_HZ;
    if (prev_cyc != 0U) {
        const uint32_t interval = now_cyc - prev_cyc;
        if (interval > nominal + (nominal / 10U)) {
            for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
                g_motors[i].loop_overrun_cnt++;
            }
        }
    }
    prev_cyc = now_cyc;

    s_ctrl_tick++;

    /* ---- 母线电压 / 慢量：每 20 拍（1kHz）更新一次 ---- */
    if ((s_ctrl_tick % (CURRENT_LOOP_HZ / 1000U)) == 0U) {
        current_sense_update_vbus_only(&g_cs);
    }

    /* ---- 四台电机电流环 ---- */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motor_fast_loop(&g_motors[i]);
    }

    /* ---- 相机锁存结果下发 ---- */
    cam_io_tick_20k();

    /* ---- CAN 周期帧 ---- */
    can_node_tick_20k();
}

/* ------------------------------------------------------------------ */
/*  1kHz 慢任务（由主循环里的软定时驱动，不占中断）                     */
/* ------------------------------------------------------------------ */
static void tasks_1k(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motor_slow_task(&g_motors[i]);
    }
    pump_tick_1k();
    cam_io_tick_1k();

    /* 运行指示灯：心跳 1Hz */
    static uint32_t led_div = 0;
    if (++led_div >= 1000U) {
        led_div = 0;
        HAL_GPIO_TogglePin(LED_RUN_PORT, LED_RUN_PIN);
    }

    /* 故障灯：任一电机故障常亮 */
    bool any_fault = false;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (g_motors[i].state == MSTATE_FAULT) { any_fault = true; break; }
    }
    board_led_set(1U, any_fault);
}

/* ------------------------------------------------------------------ */
/*  主函数                                                            */
/* ------------------------------------------------------------------ */
int main(void)
{
    HAL_Init();
    board_clock_init();
    board_gpio_init();
    fast_math_init();          /* CORDIC 就绪；失败会自动回退 libm，不致命 */

    /* 1.5 协议自检：线格式结构体的长度与字段偏移必须与上位机一侧一致。
     * 检查失败直接进 Error_Handler —— 带着错位的帧格式运行，
     * 表现是"通信正常但数值全乱"，那种故障在现场要查好几天。 */
    if (!can_proto_self_check()) {
        Error_Handler();
    }

    /* 2. 参数载入 */
    if (!param_load()) {
        param_load_defaults();
        /* 首次上电：Flash 里没有标定值，全部电机标记为待标定 */
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            g_params.motor[i].enc_elec_offset = 0.0f;
        }
    }

    /* 3. 先封桥 —— 在任何情况下都不允许上电瞬间有输出 */
    board_all_motors_off();

    /* 4. ADC 校准 + 电流偏置标定 */
    current_sense_init(&g_cs);
    current_sense_calibrate_offsets(&g_cs);
    current_sense_update_vbus_only(&g_cs);

    /* 5~6. 电机子系统（含编码器流水线、偏置写入、PI 参数） */
    motor_init_all();
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        g_motors[i].enc.elec_offset = g_params.motor[i].enc_elec_offset;
        g_motors[i].current_limit   = g_params.motor[i].limit_current_a;
        g_motors[i].foc.pid_q.cfg.kp = g_params.motor[i].pid_cur_kp;
        g_motors[i].foc.pid_q.cfg.ki = g_params.motor[i].pid_cur_ki;
        g_motors[i].foc.pid_d.cfg.kp = g_params.motor[i].pid_cur_kp;
        g_motors[i].foc.pid_d.cfg.ki = g_params.motor[i].pid_cur_ki;
        pid_set_gains(&g_motors[i].pid_velocity,
                      g_params.motor[i].pid_vel_kp,
                      g_params.motor[i].pid_vel_ki,
                      g_params.motor[i].pid_vel_kd);
        pid_set_gains(&g_motors[i].pid_position,
                      g_params.motor[i].pid_pos_kp,
                      g_params.motor[i].pid_pos_ki,
                      g_params.motor[i].pid_pos_kd);
    }

    /* 7. 外围 */
    pump_init();
    cam_io_init();

    /* 8. CAN 最后启动 */
    can_node_init();
    can_node_report_event(CAN_EVT_BOOT, 0xFFU, (float)(FW_VERSION_MAJOR * 100 + FW_VERSION_MINOR));

    /* 9. 开环控制中断 */
    HAL_TIM_Base_Start_IT(&htim1);

    /* ------------------------------------------------------------------ */
    /*  主循环                                                            */
    /* ------------------------------------------------------------------ */
    uint32_t next_1k = HAL_GetTick();
    for (;;) {
        can_node_poll();

        const uint32_t now = HAL_GetTick();
        if ((int32_t)(now - next_1k) >= 0) {
            next_1k = now + 1U;
            tasks_1k();
        }

        /* 空闲时进低功耗等待，中断一来立刻醒 —— 把 CPU 占用让给环路抖动余量 */
        __WFI();
    }
}

/* ------------------------------------------------------------------ */
/*  故障处理                                                          */
/* ------------------------------------------------------------------ */
void Error_Handler(void)
{
    board_all_motors_off();
    pump_stop(false);
    cam_trigger_disable();
    __disable_irq();
    for (;;) {
        board_led_set(1U, true);
        HAL_Delay(100);
        board_led_set(1U, false);
        HAL_Delay(100);
    }
}
