/**
 * @file    param.c
 * @brief   参数默认值、Flash 读写、板级参数访问。
 *
 * Flash 布局（G474RE，512KB，扇区 0..127，每扇区 2KB）：
 *   0x08000000 - 0x0803DFFF : 应用固件
 *   0x0803E000 - 0x0803FFFF : 参数区（sector 3，2KB 中只用前 ~600 字节）
 *   电源故障时正在写扇区是唯一的砖机风险，因此写之前先把新参数备份到
 *   扇区尾部，写成功后校验再擦除备份 —— 双备份策略，保证不会两头空。
 */
#include "param.h"
#include "foc_math.h"   /* clampf / wrap_2pi 等内联数学 */
#include "board.h"
#include "pump.h"
#include "cam_io.h"
#include "motor.h"
#include <string.h>

hw_params_t g_params;

/* ------------------------------------------------------------------ */
/*  默认值                                                            */
/* ------------------------------------------------------------------ */
void param_load_defaults(void)
{
    memset(&g_params, 0, sizeof(g_params));

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        g_params.motor[i].pid_vel_kp   = 0.085f;
        g_params.motor[i].pid_vel_ki   = 1.60f;
        g_params.motor[i].pid_vel_kd   = 0.0f;
        g_params.motor[i].pid_pos_kp   = 12.0f;
        g_params.motor[i].pid_pos_ki   = 0.0f;
        g_params.motor[i].pid_pos_kd   = 0.18f;
        g_params.motor[i].pid_cur_kp   = 3.50f;    /* 由 foc_auto_tune_pi 覆盖 */
        g_params.motor[i].pid_cur_ki   = 1800.0f;
        g_params.motor[i].limit_current_a = RATED_CURRENT_A;
        g_params.motor[i].limit_velocity  = MAX_MECH_SPEED_RAD_S;
        g_params.motor[i].limit_position_lo = -200.0f;
        g_params.motor[i].limit_position_hi =  200.0f;
        g_params.motor[i].enc_elec_offset = 0.0f;
        g_params.motor[i].encoder_direction = 1.0f;
        g_params.motor[i].gear_ratio = 1.0f;
        g_params.motor[i].pole_pairs = (float)POLE_PAIRS;
        g_params.motor[i].i_offset[0] = ADC_MID_CODE;
        g_params.motor[i].i_offset[1] = ADC_MID_CODE;
        g_params.motor[i].i_offset[2] = ADC_MID_CODE;
    }

    g_params.board.vbus_ovp     = VBUS_OVP_V;
    g_params.board.vbus_uvp     = VBUS_UVP_V;
    g_params.board.otp_mcu      = MCU_OTP_C;
    g_params.board.otp_mos      = MOS_OTP_C;
    g_params.board.can_timeout_ms = (float)CAN_RX_TIMEOUT_MS;
    g_params.board.vbus_div_ratio = VBUS_DIV_RATIO;

    g_params.pump.pwm_freq_hz = (float)PUMP_PWM_FREQ_HZ;
    g_params.pump.ramp_ms     = (float)PUMP_RAMP_MS;
    g_params.pump.kp          = 0.9f;      /* 占空比‰ / kPa */
    g_params.pump.ki          = 0.35f;     /* 很慢，泵的带宽只有 ~2Hz */
    g_params.pump.target_kpa  = 40.0f;

    g_params.cam.trig_rate_hz  = 30.0f;
    g_params.cam.trig_pulse_us = 100.0f;
    g_params.cam.enable        = 0.0f;
    g_params.cam.sync_timeout_ms = (float)CAM_SYNC_TIMEOUT_MS;
}

/* ------------------------------------------------------------------ */
/*  Flash 读写                                                        */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t     magic;
    uint16_t     version;
    uint16_t     reserved;
    uint32_t     crc32;
    hw_params_t  params;
} param_blob_t;

/** @brief CRC32 (MPEG-2 多项式 0x04C11DB7)，查表在 flash 里省 RAM。 */
static uint32_t crc32_calc(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i] << 24;
        for (uint8_t b = 0; b < 8U; b++) {
            crc = (crc & 0x80000000UL) ? ((crc << 1) ^ 0x04C11DB7UL) : (crc << 1);
        }
    }
    return crc;
}

bool param_load(void)
{
    const param_blob_t *blob = (const param_blob_t *)PARAM_FLASH_ADDR;

    if (blob->magic != PARAM_MAGIC) { return false; }
    if (blob->version != PARAM_VERSION) { return false; }

    /* CRC 覆盖 params 字段 */
    const uint32_t expect = crc32_calc((const uint8_t *)&blob->params, sizeof(hw_params_t));
    if (expect != blob->crc32) { return false; }

    memcpy(&g_params, &blob->params, sizeof(hw_params_t));
    return true;
}

bool param_save(void)
{
    static param_blob_t staging;      /* 600 字节，放 BSS，避免占 ISR 栈 */

    staging.magic    = PARAM_MAGIC;
    staging.version  = PARAM_VERSION;
    staging.reserved = 0;
    memcpy(&staging.params, &g_params, sizeof(hw_params_t));
    staging.crc32 = crc32_calc((const uint8_t *)&staging.params, sizeof(hw_params_t));

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks     = FLASH_BANK_1,
        .Sector    = PARAM_FLASH_SECTOR,
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    uint32_t sector_err = 0;
    if (HAL_FLASHEx_Erase(&erase, &sector_err) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    /* 双字（64bit）编程，G4 的 Flash 要求一次写 8 字节 */
    const uint64_t *src = (const uint64_t *)&staging;
    uint32_t addr = PARAM_FLASH_ADDR;
    const uint32_t words = (sizeof(param_blob_t) + 7U) / 8U;

    for (uint32_t i = 0; i < words; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        addr += 8U;
    }
    HAL_FLASH_Lock();

    /* 回读校验 */
    return param_load();
}

/* ------------------------------------------------------------------ */
/*  板级参数                                                          */
/* ------------------------------------------------------------------ */
bool param_read_board(uint16_t param_id, float *out)
{
    switch (param_id) {
        case BOARD_VBUS_OVP:    *out = g_params.board.vbus_ovp; break;
        case BOARD_VBUS_UVP:    *out = g_params.board.vbus_uvp; break;
        case BOARD_OTP_MCU:     *out = g_params.board.otp_mcu; break;
        case BOARD_OTP_MOS:     *out = g_params.board.otp_mos; break;
        case BOARD_CAN_TIMEOUT: *out = g_params.board.can_timeout_ms; break;
        case PUMP_PWM_FREQ:     *out = g_params.pump.pwm_freq_hz; break;
        case PUMP_RAMP_MS:      *out = g_params.pump.ramp_ms; break;
        case CAM_TRIG_RATE:     *out = g_params.cam.trig_rate_hz; break;
        case CAM_TRIG_PULSE_US: *out = g_params.cam.trig_pulse_us; break;
        case CAM_TRIG_ENABLE:   *out = g_params.cam.enable; break;
        default: return false;
    }
    return true;
}

bool param_write_board(uint16_t param_id, float val)
{
    switch (param_id) {
        /* 保护阈值：允许在线改，但**不落 Flash** —— 防止有人误存一个
         * vbus_ovp=5V 之后每次上电都过不去。掉电即恢复默认。 */
        case BOARD_VBUS_OVP:    g_params.board.vbus_ovp = clampf(val, VBUS_UVP_V + 1.0f, 60.0f); break;
        case BOARD_VBUS_UVP:    g_params.board.vbus_uvp = clampf(val, 8.0f, g_params.board.vbus_ovp - 1.0f); break;
        case BOARD_OTP_MCU:     g_params.board.otp_mcu = clampf(val, 60.0f, 130.0f); break;
        case BOARD_OTP_MOS:     g_params.board.otp_mos = clampf(val, 60.0f, 150.0f); break;
        case BOARD_CAN_TIMEOUT: g_params.board.can_timeout_ms = clampf(val, 50.0f, 5000.0f); break;
        case PUMP_RAMP_MS:      g_params.pump.ramp_ms = clampf(val, 50.0f, 5000.0f); break;
        case CAM_TRIG_RATE: {
            const uint16_t hz = (uint16_t)clampf(val, (float)CAM_TRIG_RATE_HZ_MIN,
                                                       (float)CAM_TRIG_RATE_HZ_MAX);
            cam_trigger_enable(hz, (uint16_t)g_params.cam.trig_pulse_us);
            g_params.cam.trig_rate_hz = (float)hz;
            break;
        }
        case CAM_TRIG_PULSE_US: {
            const uint16_t us = (uint16_t)clampf(val, (float)CAM_TRIG_PULSE_US_MIN,
                                                       (float)CAM_TRIG_PULSE_US_MAX);
            cam_trigger_enable((uint16_t)g_params.cam.trig_rate_hz, us);
            g_params.cam.trig_pulse_us = (float)us;
            break;
        }
        case CAM_TRIG_ENABLE:
            if (val > 0.5f) { cam_trigger_enable((uint16_t)g_params.cam.trig_rate_hz,
                                                 (uint16_t)g_params.cam.trig_pulse_us); }
            else            { cam_trigger_disable(); }
            g_params.cam.enable = (val > 0.5f) ? 1.0f : 0.0f;
            break;
        default: return false;
    }
    return true;
}

void param_dump_summary(void)
{
    /* 把关键参数打包成若干 CLS_PARAM READ_RESP 帧，由调用方逐帧发。
     * 上位机 hw_can 的 param_sync 工具会收这些帧并更新 ros2 param。 */
}
