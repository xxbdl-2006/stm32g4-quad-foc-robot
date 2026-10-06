/**
 * @file    board.c
 * @brief   板级底层：时钟、GPIO、PWM 落点、温度、母线电压、复位。
 *
 * 驱动器模式说明（很重要，决定了 PWM 段数）：
 *   四路电机各用一片 DRV8323，配置为 **3x PWM 模式**（MODE 引脚 1/0/0）。
 *   此模式下 MCU 只需 3 路 PWM 送进 INHx，芯片内部
 *      PWM=1 -> 上管开、下管关；PWM=0 -> 上管关、下管开
 *   并自带 180ns 死区。因此：
 *     - 不需要 MCU 产生互补通道与死区寄存器；
 *     - 三相 CCR 同时写 0 即等于"下管全开 = 制动"，这就是 board_pwm_brake 的实现；
 *     - 死区引起的电压畸变仍需软件补偿（见 current_sense_deadtime_comp）。
 */
#include "board.h"
#include "foc_math.h"
#include "param.h"

/* =================================================================== */
/*  前向声明：故障兜底（定义在 main.c）                                 */
/* =================================================================== */
extern void Error_Handler(void);

/* =================================================================== */
/*  PWM 落点：4 路电机 -> 4 个定时器各 3 个通道                          */
/* =================================================================== */
static TIM_HandleTypeDef *const pwm_tim[MOTOR_COUNT] = {
    &htim1,     /* M0: TIM1_CH1/2/3 (PA8/PA9/PA10) */
    &htim2,     /* M1: TIM2_CH1/2/3 (PA0/PA1/PA2) */
    &htim8,     /* M2: TIM8_CH1/2/3 (PC6/PC7/PC8) */
    &htim4      /* M3: TIM4_CH1/2/3 (PB6/PB7/PB8) */
};
static const uint32_t pwm_ch[3] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3 };

/* 驱动使能引脚（nSLEEP），高有效 */
static const uint16_t drv_en_pin[MOTOR_COUNT] = {
    DRV_EN0_PIN, DRV_EN1_PIN, DRV_EN2_PIN, DRV_EN3_PIN
};

static volatile bool s_pwm_enabled[MOTOR_COUNT];
static volatile bool s_brake[MOTOR_COUNT];

void board_pwm_write(uint8_t idx, const pwm_duty_t *d)
{
    if (idx >= MOTOR_COUNT) { return; }

    if (s_brake[idx]) {
        /* 制动态：下管常通，不走 SVPWM */
        for (uint8_t k = 0; k < 3U; k++) {
            __HAL_TIM_SET_COMPARE(pwm_tim[idx], pwm_ch[k], 0U);
        }
        return;
    }
    if (!s_pwm_enabled[idx]) { return; }

    __HAL_TIM_SET_COMPARE(pwm_tim[idx], pwm_ch[0], d->a);
    __HAL_TIM_SET_COMPARE(pwm_tim[idx], pwm_ch[1], d->b);
    __HAL_TIM_SET_COMPARE(pwm_tim[idx], pwm_ch[2], d->c);
}

void board_pwm_enable(uint8_t idx, bool en)
{
    if (idx >= MOTOR_COUNT) { return; }

    if (en) {
        s_brake[idx] = false;
        /* 顺序：先把 CCR 写成 50% 中位，再开驱动使能。
         * 反过来的话，使能瞬间 CCR 还是上次的残值，会有一拍满压输出。 */
        const uint16_t mid = SVPWM_HALF;
        __HAL_TIM_SET_COMPARE(pwm_tim[idx], pwm_ch[0], mid);
        __HAL_TIM_SET_COMPARE(pwm_tim[idx], pwm_ch[1], mid);
        __HAL_TIM_SET_COMPARE(pwm_tim[idx], pwm_ch[2], mid);
        HAL_GPIO_WritePin(DRV_EN_PORT, drv_en_pin[idx], GPIO_PIN_SET);
        if (!s_pwm_enabled[idx]) {
            HAL_TIM_PWM_Start(pwm_tim[idx], TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(pwm_tim[idx], TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(pwm_tim[idx], TIM_CHANNEL_3);
        }
        s_pwm_enabled[idx] = true;
    } else {
        /* 关断 = 六管全关，电机自由滑行。先把 CCR 清掉再撤使能，
         * 避免使能撤掉时引脚被拉低而实际是"下管开"（取决于驱动器的失效行为）。 */
        for (uint8_t k = 0; k < 3U; k++) {
            __HAL_TIM_SET_COMPARE(pwm_tim[idx], pwm_ch[k], 0U);
        }
        HAL_GPIO_WritePin(DRV_EN_PORT, drv_en_pin[idx], GPIO_PIN_RESET);
        s_pwm_enabled[idx] = false;
        s_brake[idx] = false;
    }
}

void board_pwm_brake(uint8_t idx, bool en)
{
    if (idx >= MOTOR_COUNT) { return; }
    if (en) {
        if (!s_pwm_enabled[idx]) { board_pwm_enable(idx, true); }
        s_brake[idx] = true;
        for (uint8_t k = 0; k < 3U; k++) {
            __HAL_TIM_SET_COMPARE(pwm_tim[idx], pwm_ch[k], 0U);
        }
    } else {
        s_brake[idx] = false;
    }
}

void board_all_motors_off(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        board_pwm_enable(i, false);
    }
}

bool board_drv_fault_active(uint8_t idx)
{
    const uint16_t pin[4] = { DRV_FAULT0_PIN, DRV_FAULT1_PIN, DRV_FAULT2_PIN, DRV_FAULT3_PIN };
    if (idx >= MOTOR_COUNT) { return false; }
    /* nFAULT 低有效 */
    return HAL_GPIO_ReadPin(DRV_FAULT_PORT, pin[idx]) == GPIO_PIN_RESET;
}

/* =================================================================== */
/*  编码器片选：四片 AS5047P 分散在两个端口上，集中在这里处理            */
/* =================================================================== */
void board_encoder_cs(uint8_t idx, bool active)
{
    static GPIO_TypeDef *const port[MOTOR_COUNT] = {
        ENC_CS0_PORT, ENC_CS1_PORT, ENC_CS2_PORT, ENC_CS3_PORT
    };
    static const uint16_t pin[MOTOR_COUNT] = {
        ENC_CS0_PIN, ENC_CS1_PIN, ENC_CS2_PIN, ENC_CS3_PIN
    };
    if (idx >= MOTOR_COUNT) { return; }
    HAL_GPIO_WritePin(port[idx], pin[idx], active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* =================================================================== */
/*  模拟量                                                            */
/* =================================================================== */
float board_read_vbus(void)
{
    extern float current_sense_get_vbus(void);
    return current_sense_get_vbus();
}

float board_read_mcu_temp(void)
{
    /* STM32G4 内部温度传感器：TS_CAL1/TS_CAL2 出厂校准点，
     * 分别在 30°C / 130°C 下测得，斜率 = (130-30)/(TS_CAL2-TS_CAL1)。 */
    static bool inited = false;
    static float slope = 0.0f, offset = 0.0f;

    if (!inited) {
        const uint16_t cal1 = *(uint16_t *)0x1FFF75A8UL;   /* TS_CAL1 @30C */
        const uint16_t cal2 = *(uint16_t *)0x1FFF75CAUL;   /* TS_CAL2 @130C */
        if (cal2 > cal1) {
            slope  = 100.0f / (float)(cal2 - cal1);
            offset = 30.0f - slope * (float)cal1;
        }
        inited = true;
    }

    HAL_ADC_Start(&hadc5);
    if (HAL_ADC_PollForConversion(&hadc5, 2U) != HAL_OK) { return 25.0f; }
    const float raw = (float)HAL_ADC_GetValue(&hadc5);
    HAL_ADC_Stop(&hadc5);

    float t = slope * raw + offset;

    /* 内部传感器精度差（±5°C），只用它做过温保护，不做显示主值 */
    return t;
}

float board_read_mos_temp(uint8_t idx)
{
    extern uint16_t current_sense_get_ntc_raw(uint8_t i);
    const uint16_t raw = current_sense_get_ntc_raw(idx);
    if (raw == 0U || raw >= 4095U) { return -40.0f; }   /* 传感器异常 */

    /* NTC 分压：Vcc=3.3V，上拉 R25=10k —— R_ntc = R25 * raw/(4095-raw) */
    const float r_ntc = NTC_R25_OHM * (float)raw / (4095.0f - (float)raw);
    /* B 参数方程：1/T = 1/T0 + ln(R/R0)/B */
    const float inv_t = (1.0f / 298.15f) + logf(r_ntc / NTC_R25_OHM) / NTC_BETA;
    const float kelvin = 1.0f / inv_t;
    return kelvin - 273.15f;
}

/* =================================================================== */
/*  GPIO / LED / 复位                                                 */
/* =================================================================== */
void board_led_set(uint8_t which, bool on)
{
    if (which == 0U) {
        HAL_GPIO_WritePin(LED_RUN_PORT, LED_RUN_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(LED_FAULT_PORT, LED_FAULT_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void board_system_reset(void)
{
    board_all_motors_off();
    HAL_Delay(5);
    NVIC_SystemReset();
}

/* =================================================================== */
/*  时钟与 GPIO（CubeMX 生成的 SystemClock_Config / MX_GPIO_Init 包装）  */
/* =================================================================== */
extern void SystemClock_Config(void);   /* CubeMX 生成 */
extern void MX_GPIO_Init(void);         /* CubeMX 生成 */

void board_clock_init(void)
{
    SystemClock_Config();
    /* 断言：实际主频必须是 170MHz，否则所有环路定时都会错 */
    if (SystemCoreClock != 170000000UL) {
        Error_Handler();
    }
}

void board_gpio_init(void)
{
    MX_GPIO_Init();
    /* 上电默认：所有驱动使能拉低 */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        HAL_GPIO_WritePin(DRV_EN_PORT, drv_en_pin[i], GPIO_PIN_RESET);
    }
}
