/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : imu.c
  * @brief          : IMU service implementation (ISM330DHCX, polling, v1).
  *
  * State machine: UNINIT -> POWER_WAIT -> PROBE -> CONFIGURE -> RUNNING
  *                -> DEGRADED -> FAULT -> RECOVER. All timing is driven by the
  * now_ms argument; there is no HAL_Delay here and IMU_Update is bounded-time
  * (short SPI transfers only). An IMU fault never stops motors in v1.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "imu.h"

#include "app_config.h"
#include "diagnostics.h"
#include "jetson_tx.h"
#include "main.h"

#include <string.h>

static ImuState_t      s_state = IMU_STATE_UNINIT;
static uint32_t        s_state_enter_ms = 0U;
static uint32_t        s_last_update_ms = 0U;
static uint32_t        s_last_publish_ms = 0U;
static uint32_t        s_sequence = 0U;
static ImuSample_t     s_sample;
static ImuFaultStats_t s_fault;
static uint32_t        s_fault_bits = 0U;

static void IMU_EnterState(ImuState_t state, uint32_t now_ms)
{
    s_state = state;
    s_state_enter_ms = now_ms;
}

static void IMU_StoreSample(const ImuRawSample_t *raw, uint32_t now_ms)
{
    s_sequence++;
    s_sample.mcu_timestamp_ms = now_ms;
    s_sample.imu_timestamp_25us = raw->imu_timestamp_25us;
    s_sample.sequence = s_sequence;
    s_sample.accel_raw_x = raw->accel_raw_x;
    s_sample.accel_raw_y = raw->accel_raw_y;
    s_sample.accel_raw_z = raw->accel_raw_z;
    s_sample.gyro_raw_x = raw->gyro_raw_x;
    s_sample.gyro_raw_y = raw->gyro_raw_y;
    s_sample.gyro_raw_z = raw->gyro_raw_z;
    s_sample.temp_raw = raw->temp_raw;
    s_sample.status_reg = raw->status_reg;
    s_sample.valid = true;
    s_fault.last_good_sample_ms = now_ms;
}

static uint32_t IMU_SampleAge(uint32_t now_ms)
{
    if (!s_sample.valid) {
        return 0xFFFFFFFFU;
    }
    return now_ms - s_fault.last_good_sample_ms;
}

static ImuHealth_t IMU_ComputeHealth(uint32_t now_ms)
{
    uint32_t age = IMU_SampleAge(now_ms);

    if ((s_state == IMU_STATE_FAULT) || (s_sample.valid && (age > IMU_STALE_AGE_MS))) {
        return IMU_HEALTH_FAULT;
    }
    if ((s_state == IMU_STATE_DEGRADED) || (s_sample.valid && (age > IMU_DEGRADED_AGE_MS))) {
        return IMU_HEALTH_DEGRADED;
    }
    if ((s_state == IMU_STATE_RUNNING) && s_sample.valid) {
        return IMU_HEALTH_OK;
    }
    return IMU_HEALTH_INIT;
}

void IMU_Init(void)
{
    memset(&s_sample, 0, sizeof(s_sample));
    memset(&s_fault, 0, sizeof(s_fault));
    s_fault_bits = 0U;
    s_sequence = 0U;
    s_last_update_ms = 0U;
    s_last_publish_ms = 0U;

    /* Park CS HIGH (deselected) before any transaction. */
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    IMU_EnterState(IMU_STATE_POWER_WAIT, HAL_GetTick());
}

static void IMU_DoRunningRead(uint32_t now_ms)
{
    ImuRawSample_t raw;

    if (ISM330DHCX_ReadRaw(&raw)) {
        IMU_StoreSample(&raw, now_ms);
        s_fault.consecutive_fail_count = 0U;
        s_fault_bits &= ~(IMU_FAULT_READ | IMU_FAULT_STALE);
        if (s_state != IMU_STATE_RUNNING) {
            IMU_EnterState(IMU_STATE_RUNNING, now_ms);
        }
        return;
    }

    /* Read failed. */
    s_fault.read_error_count++;
    s_fault.spi_error_count++;
    s_fault.consecutive_fail_count++;
    s_fault.last_error_code = IMU_FAULT_READ;
    s_fault_bits |= IMU_FAULT_READ;

    uint32_t age = IMU_SampleAge(now_ms);
    bool stale = (s_sample.valid && (age > IMU_STALE_AGE_MS));
    if (stale) {
        s_fault.stale_sample_count++;
        s_fault_bits |= IMU_FAULT_STALE;
    }

    if ((s_fault.consecutive_fail_count >= IMU_DEGRADE_FAIL_LIMIT) || stale) {
        IMU_EnterState(IMU_STATE_FAULT, now_ms);
    } else if (s_state == IMU_STATE_RUNNING) {
        IMU_EnterState(IMU_STATE_DEGRADED, now_ms);
    }
}

void IMU_Update(uint32_t now_ms)
{
    switch (s_state) {
    case IMU_STATE_UNINIT:
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        IMU_EnterState(IMU_STATE_POWER_WAIT, now_ms);
        break;

    case IMU_STATE_POWER_WAIT:
        if ((now_ms - s_state_enter_ms) >= IMU_POWER_WAIT_MS) {
            IMU_EnterState(IMU_STATE_PROBE, now_ms);
        }
        break;

    case IMU_STATE_PROBE: {
        uint8_t who = 0U;
        if (!ISM330DHCX_ReadWhoAmI(&who)) {
            s_fault.spi_error_count++;
            s_fault.last_error_code = IMU_FAULT_SPI;
            s_fault_bits |= IMU_FAULT_SPI;
            IMU_EnterState(IMU_STATE_FAULT, now_ms);
        } else if (who != ISM330DHCX_WHO_AM_I_EXPECTED) {
            s_fault.whoami_mismatch_count++;
            s_fault.last_error_code = IMU_FAULT_WHOAMI;
            s_fault_bits |= IMU_FAULT_WHOAMI;
            IMU_EnterState(IMU_STATE_FAULT, now_ms);
        } else {
            s_fault_bits &= ~(IMU_FAULT_SPI | IMU_FAULT_WHOAMI);
            IMU_EnterState(IMU_STATE_CONFIGURE, now_ms);
        }
        break;
    }

    case IMU_STATE_CONFIGURE:
        if (ISM330DHCX_ConfigDefault()) {
            s_fault_bits &= ~IMU_FAULT_CONFIG;
            s_fault.consecutive_fail_count = 0U;
            s_last_update_ms = now_ms;
            IMU_EnterState(IMU_STATE_RUNNING, now_ms);
        } else {
            s_fault.config_error_count++;
            s_fault.last_error_code = IMU_FAULT_CONFIG;
            s_fault_bits |= IMU_FAULT_CONFIG;
            IMU_EnterState(IMU_STATE_FAULT, now_ms);
        }
        break;

    case IMU_STATE_RUNNING:
    case IMU_STATE_DEGRADED:
        if ((now_ms - s_last_update_ms) >= IMU_UPDATE_PERIOD_MS) {
            s_last_update_ms = now_ms;
            IMU_DoRunningRead(now_ms);
        }
        break;

    case IMU_STATE_FAULT:
        if ((now_ms - s_state_enter_ms) >= IMU_RECOVER_PERIOD_MS) {
            IMU_EnterState(IMU_STATE_RECOVER, now_ms);
        }
        break;

    case IMU_STATE_RECOVER:
        s_fault.reinit_count++;
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        IMU_EnterState(IMU_STATE_PROBE, now_ms);
        break;

    default:
        IMU_EnterState(IMU_STATE_UNINIT, now_ms);
        break;
    }
}

bool IMU_GetLatestSample(ImuSample_t *out)
{
    if (out == NULL) {
        return false;
    }

    uint32_t now = HAL_GetTick();
    s_sample.state = (uint8_t)s_state;
    s_sample.health = (uint8_t)IMU_ComputeHealth(now);
    s_sample.fault_bits = s_fault_bits;
    s_sample.age_ms = s_sample.valid ? (now - s_fault.last_good_sample_ms) : 0xFFFFFFFFU;

    *out = s_sample;
    return s_sample.valid;
}

ImuHealth_t IMU_GetHealth(void)
{
    return IMU_ComputeHealth(HAL_GetTick());
}

ImuFaultStats_t IMU_GetFaultStats(void)
{
    return s_fault;
}

void IMU_ResetFaults(void)
{
    memset(&s_fault, 0, sizeof(s_fault));
    if (s_sample.valid) {
        s_fault.last_good_sample_ms = s_sample.mcu_timestamp_ms;
    }
    s_fault_bits = 0U;
    ISM330DHCX_ResetDebugStats();
}

/* Publishes the additive IMU status line. The "_50ms" suffix is historical: the
 * actual publish cadence is IMU_PUBLISH_PERIOD_MS, now 10 ms / 100 Hz (was
 * 50 ms / 20 Hz). Publishing is additive and does not touch FDCAN/CANopen,
 * drive, lift, or battery behavior. */
void IMU_SendJetsonStatus_50ms(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_last_publish_ms) < IMU_PUBLISH_PERIOD_MS) {
        return;
    }
    s_last_publish_ms = now;

    ImuHealth_t health = IMU_ComputeHealth(now);
    uint32_t age = s_sample.valid ? (now - s_fault.last_good_sample_ms) : 0xFFFFFFFFU;

    /* IMU t_ms seq imu_ts ax ay az gx gy gz temp status health fault age */
    JetsonTx_Printf(JETSON_TX_PRIORITY_REALTIME,
                "IMU %lu %lu %lu %d %d %d %d %d %d %d %u %u %lu %lu\r\n",
                (unsigned long)now,
                (unsigned long)s_sample.sequence,
                (unsigned long)s_sample.imu_timestamp_25us,
                (int)s_sample.accel_raw_x,
                (int)s_sample.accel_raw_y,
                (int)s_sample.accel_raw_z,
                (int)s_sample.gyro_raw_x,
                (int)s_sample.gyro_raw_y,
                (int)s_sample.gyro_raw_z,
                (int)s_sample.temp_raw,
                (unsigned)s_sample.status_reg,
                (unsigned)health,
                (unsigned long)s_fault_bits,
                (unsigned long)age);
}

void IMU_PrintDebug(void)
{
    uint32_t now = HAL_GetTick();
    ImuHealth_t health = IMU_ComputeHealth(now);
    uint32_t age = IMU_SampleAge(now);
    ISM330DHCX_DebugStats_t chip = {0};
    ISM330DHCX_GetDebugStats(&chip);
    GPIO_PinState cs_pin = HAL_GPIO_ReadPin(IMU_CS_GPIO_Port, IMU_CS_Pin);

    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "IMU_DEBUG state=%u health=%u fault=%lu valid=%u seq=%lu age=%lu\r\n",
                (unsigned)s_state,
                (unsigned)health,
                (unsigned long)s_fault_bits,
                s_sample.valid ? 1U : 0U,
                (unsigned long)s_sample.sequence,
                (unsigned long)age);

    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "IMU_DEBUG whoami=0x%02X expected=0x%02X spi_status=%lu(%s)\r\n",
                (unsigned)chip.last_whoami,
                (unsigned)chip.expected_whoami,
                (unsigned long)chip.last_hal_status,
                hal_status_name((HAL_StatusTypeDef)chip.last_hal_status));

    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "IMU_DEBUG spi_error=0x%08lX cs=%u read_err=%lu write_err=%lu multi_err=%lu\r\n",
                (unsigned long)chip.last_hal_error,
                (unsigned)cs_pin,
                (unsigned long)chip.read_reg_error_count,
                (unsigned long)chip.write_reg_error_count,
                (unsigned long)chip.read_multi_error_count);

    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "IMU_DEBUG whoami_mismatch=%lu config_err=%lu stale=%lu read_fail=%lu spi_fail=%lu\r\n",
                (unsigned long)s_fault.whoami_mismatch_count,
                (unsigned long)s_fault.config_error_count,
                (unsigned long)s_fault.stale_sample_count,
                (unsigned long)s_fault.read_error_count,
                (unsigned long)s_fault.spi_error_count);

    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "IMU_DEBUG consecutive_fail=%lu reinit=%lu last_error=%lu last_good=%lu\r\n",
                (unsigned long)s_fault.consecutive_fail_count,
                (unsigned long)s_fault.reinit_count,
                (unsigned long)s_fault.last_error_code,
                (unsigned long)s_fault.last_good_sample_ms);
}
