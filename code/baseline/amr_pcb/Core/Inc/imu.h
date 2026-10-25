/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : imu.h
  * @brief          : IMU service (state machine, timing, latest sample, faults).
  *
  * Owns IMU state/timing/fault/latest sample and the Jetson IMU telemetry line.
  * Uses imu_ism330dhcx for all register access. Performs no fusion in v1 and
   * never controls motors. See docs/system_truth.md.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "imu_ism330dhcx.h"

typedef enum
{
    IMU_HEALTH_INIT = 0,
    IMU_HEALTH_OK,
    IMU_HEALTH_DEGRADED,
    IMU_HEALTH_FAULT
} ImuHealth_t;

typedef enum
{
    IMU_STATE_UNINIT = 0,
    IMU_STATE_POWER_WAIT,
    IMU_STATE_PROBE,
    IMU_STATE_CONFIGURE,
    IMU_STATE_RUNNING,
    IMU_STATE_DEGRADED,
    IMU_STATE_FAULT,
    IMU_STATE_RECOVER
} ImuState_t;

/* Fault bitmask values reported in ImuSample_t.fault_bits. */
#define IMU_FAULT_SPI        (1U << 0)
#define IMU_FAULT_WHOAMI     (1U << 1)
#define IMU_FAULT_CONFIG     (1U << 2)
#define IMU_FAULT_READ       (1U << 3)
#define IMU_FAULT_STALE      (1U << 4)

typedef struct
{
    uint32_t mcu_timestamp_ms;
    uint32_t imu_timestamp_25us;
    uint32_t sequence;
    int16_t  accel_raw_x;
    int16_t  accel_raw_y;
    int16_t  accel_raw_z;
    int16_t  gyro_raw_x;
    int16_t  gyro_raw_y;
    int16_t  gyro_raw_z;
    int16_t  temp_raw;
    uint8_t  status_reg;
    uint8_t  state;
    uint8_t  health;
    uint32_t fault_bits;
    uint32_t age_ms;
    bool     valid;
} ImuSample_t;

typedef struct
{
    uint32_t spi_error_count;
    uint32_t whoami_mismatch_count;
    uint32_t config_error_count;
    uint32_t read_error_count;
    uint32_t stale_sample_count;
    uint32_t consecutive_fail_count;
    uint32_t reinit_count;
    uint32_t last_good_sample_ms;
    uint32_t last_error_code;
} ImuFaultStats_t;

void           IMU_Init(void);
void           IMU_Update(uint32_t now_ms);
bool           IMU_GetLatestSample(ImuSample_t *out);
ImuHealth_t    IMU_GetHealth(void);
ImuFaultStats_t IMU_GetFaultStats(void);
void           IMU_ResetFaults(void);
/* Historical name; actual publish period is IMU_PUBLISH_PERIOD_MS (now 10 ms / 100 Hz). */
void           IMU_SendJetsonStatus_50ms(void);
void           IMU_PrintDebug(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
