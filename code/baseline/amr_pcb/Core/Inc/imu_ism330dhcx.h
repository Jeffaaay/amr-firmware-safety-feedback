/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : imu_ism330dhcx.h
  * @brief          : ISM330DHCX chip driver (SPI4 register access + registers).
  *
  * Low-level driver only. Owns SPI4 transactions, manual CS (PE11), and the
  * ISM330DHCX register map. Contains no service policy, timing, or scheduling.
   * See docs/system_truth.md.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef IMU_ISM330DHCX_H
#define IMU_ISM330DHCX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ISM330DHCX register addresses. */
#define ISM330DHCX_REG_WHO_AM_I      0x0FU
#define ISM330DHCX_REG_CTRL1_XL      0x10U
#define ISM330DHCX_REG_CTRL2_G       0x11U
#define ISM330DHCX_REG_CTRL3_C       0x12U
#define ISM330DHCX_REG_CTRL4_C       0x13U
#define ISM330DHCX_REG_CTRL9_XL      0x18U
#define ISM330DHCX_REG_CTRL10_C      0x19U
#define ISM330DHCX_REG_STATUS_REG    0x1EU
#define ISM330DHCX_REG_OUT_TEMP_L    0x20U
#define ISM330DHCX_REG_OUTX_L_G      0x22U
#define ISM330DHCX_REG_OUTX_L_A      0x28U
#define ISM330DHCX_REG_TIMESTAMP0    0x40U

#define ISM330DHCX_WHO_AM_I_EXPECTED 0x6BU

/* SPI read/write command bit on the first transferred byte. */
#define ISM330DHCX_SPI_READ_BIT      0x80U

/* Raw sensor sample, little-endian two's complement as read from the chip. */
typedef struct
{
    uint8_t  status_reg;
    int16_t  temp_raw;
    int16_t  gyro_raw_x;
    int16_t  gyro_raw_y;
    int16_t  gyro_raw_z;
    int16_t  accel_raw_x;
    int16_t  accel_raw_y;
    int16_t  accel_raw_z;
    uint32_t imu_timestamp_25us;
} ImuRawSample_t;

typedef struct
{
    uint8_t  last_whoami;
    uint8_t  expected_whoami;
    uint32_t last_hal_status;
    uint32_t last_hal_error;
    uint32_t read_reg_error_count;
    uint32_t write_reg_error_count;
    uint32_t read_multi_error_count;
} ISM330DHCX_DebugStats_t;

bool ISM330DHCX_ReadReg(uint8_t reg, uint8_t *value);
bool ISM330DHCX_WriteReg(uint8_t reg, uint8_t value);
bool ISM330DHCX_ReadMulti(uint8_t start_reg, uint8_t *buf, uint16_t len);
bool ISM330DHCX_ReadWhoAmI(uint8_t *id);
bool ISM330DHCX_ConfigDefault(void);
bool ISM330DHCX_ReadRaw(ImuRawSample_t *sample);
bool ISM330DHCX_ReadTimestamp(uint32_t *timestamp_25us);
void ISM330DHCX_GetDebugStats(ISM330DHCX_DebugStats_t *out);
void ISM330DHCX_ResetDebugStats(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ISM330DHCX_H */
