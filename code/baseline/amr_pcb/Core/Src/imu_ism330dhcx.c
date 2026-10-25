/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : imu_ism330dhcx.c
  * @brief          : ISM330DHCX chip driver implementation (SPI4 + manual CS).
  *
  * 4-wire SPI on SPI4. Manual chip select on PE11 (IMU_CS), idle HIGH, driven
  * LOW for each transaction and HIGH again on every exit path including errors.
  * Read command: reg | 0x80. Write command: reg & 0x7F. Multi-byte read relies
  * on CTRL3_C.IF_INC = 1. All transfers use a short bounded timeout; no HAL_Delay.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "imu_ism330dhcx.h"

#include "app_config.h"
#include "main.h"

#include <string.h>

extern SPI_HandleTypeDef hspi4;

static ISM330DHCX_DebugStats_t s_debug = {
    .last_whoami = 0xFFU,
    .expected_whoami = ISM330DHCX_WHO_AM_I_EXPECTED,
    .last_hal_status = (uint32_t)HAL_OK,
    .last_hal_error = 0U,
};

static inline void ISM330DHCX_CS_Low(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static inline void ISM330DHCX_CS_High(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static void ISM330DHCX_RecordHalStatus(HAL_StatusTypeDef status)
{
    s_debug.last_hal_status = (uint32_t)status;
    s_debug.last_hal_error = HAL_SPI_GetError(&hspi4);
}

bool ISM330DHCX_ReadReg(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        s_debug.last_hal_status = (uint32_t)HAL_ERROR;
        s_debug.last_hal_error = HAL_SPI_GetError(&hspi4);
        s_debug.read_reg_error_count++;
        return false;
    }

    uint8_t tx[2] = { (uint8_t)(reg | ISM330DHCX_SPI_READ_BIT), 0x00U };
    uint8_t rx[2] = { 0U, 0U };

    ISM330DHCX_CS_Low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi4, tx, rx, 2U, IMU_SPI_TIMEOUT_MS);
    ISM330DHCX_CS_High();
    ISM330DHCX_RecordHalStatus(st);

    if (st != HAL_OK) {
        s_debug.read_reg_error_count++;
        return false;
    }

    *value = rx[1];
    return true;
}

bool ISM330DHCX_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), value };

    ISM330DHCX_CS_Low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi4, tx, 2U, IMU_SPI_TIMEOUT_MS);
    ISM330DHCX_CS_High();
    ISM330DHCX_RecordHalStatus(st);

    if (st != HAL_OK) {
        s_debug.write_reg_error_count++;
    }
    return (st == HAL_OK);
}

bool ISM330DHCX_ReadMulti(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    /* Single full-duplex transaction (command byte + len read bytes) avoids
     * RX-FIFO ordering issues from split transmit/receive on STM32H7. */
    if ((buf == NULL) || (len == 0U) || (len > 32U)) {
        s_debug.last_hal_status = (uint32_t)HAL_ERROR;
        s_debug.last_hal_error = HAL_SPI_GetError(&hspi4);
        s_debug.read_multi_error_count++;
        return false;
    }

    uint8_t tx[33] = { 0 };
    uint8_t rx[33] = { 0 };
    tx[0] = (uint8_t)(start_reg | ISM330DHCX_SPI_READ_BIT);

    ISM330DHCX_CS_Low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi4, tx, rx,
                                                   (uint16_t)(len + 1U),
                                                   IMU_SPI_TIMEOUT_MS);
    ISM330DHCX_CS_High();
    ISM330DHCX_RecordHalStatus(st);

    if (st != HAL_OK) {
        s_debug.read_multi_error_count++;
        return false;
    }

    for (uint16_t i = 0U; i < len; i++) {
        buf[i] = rx[i + 1U];
    }
    return true;
}

bool ISM330DHCX_ReadWhoAmI(uint8_t *id)
{
    bool ok = ISM330DHCX_ReadReg(ISM330DHCX_REG_WHO_AM_I, id);
    if (ok) {
        s_debug.last_whoami = *id;
    }
    return ok;
}

/* Read-modify-write a single bitmask, preserving the chip's other/default bits. */
static bool ISM330DHCX_SetBits(uint8_t reg, uint8_t set_mask, uint8_t clear_mask)
{
    uint8_t value = 0U;

    if (!ISM330DHCX_ReadReg(reg, &value)) {
        return false;
    }

    value = (uint8_t)((value & (uint8_t)~clear_mask) | set_mask);
    return ISM330DHCX_WriteReg(reg, value);
}

bool ISM330DHCX_ConfigDefault(void)
{
    uint8_t who = 0U;
    bool ok = true;

    if (!ISM330DHCX_ReadWhoAmI(&who) || (who != ISM330DHCX_WHO_AM_I_EXPECTED)) {
        return false;
    }

    /* CTRL3_C: BDU = 1 (bit6), IF_INC = 1 (bit2), SIM = 0 (bit3, 4-wire SPI). */
    ok = ISM330DHCX_WriteReg(ISM330DHCX_REG_CTRL3_C, 0x44U) && ok;

    /* CTRL4_C: set I2C_disable (bit2), preserve other bits. */
    ok = ISM330DHCX_SetBits(ISM330DHCX_REG_CTRL4_C, 0x04U, 0x00U) && ok;

    /* CTRL9_XL: set DEVICE_CONF (bit1), preserve other/default bits. */
    ok = ISM330DHCX_SetBits(ISM330DHCX_REG_CTRL9_XL, 0x02U, 0x00U) && ok;

    /* CTRL1_XL: accel ODR = 104 Hz, full scale = +/-4g (0x48). */
    ok = ISM330DHCX_WriteReg(ISM330DHCX_REG_CTRL1_XL, 0x48U) && ok;

    /* CTRL2_G: gyro ODR = 104 Hz, full scale = +/-500 dps (0x44). */
    ok = ISM330DHCX_WriteReg(ISM330DHCX_REG_CTRL2_G, 0x44U) && ok;

    /* CTRL10_C: TIMESTAMP_EN (bit5) only when configured; otherwise leave default. */
#if IMU_ENABLE_TIMESTAMP
    ok = ISM330DHCX_SetBits(ISM330DHCX_REG_CTRL10_C, 0x20U, 0x00U) && ok;
#endif

    if (!ok) {
        return false;
    }

    /* Read back key configuration registers and verify. */
    uint8_t ctrl1 = 0U;
    uint8_t ctrl2 = 0U;
    uint8_t ctrl3 = 0U;

    if (!ISM330DHCX_ReadReg(ISM330DHCX_REG_CTRL1_XL, &ctrl1) ||
        !ISM330DHCX_ReadReg(ISM330DHCX_REG_CTRL2_G, &ctrl2) ||
        !ISM330DHCX_ReadReg(ISM330DHCX_REG_CTRL3_C, &ctrl3)) {
        return false;
    }

    if ((ctrl1 != 0x48U) || (ctrl2 != 0x44U) || (ctrl3 != 0x44U)) {
        return false;
    }

    return true;
}

static inline int16_t ISM330DHCX_LE16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

bool ISM330DHCX_ReadRaw(ImuRawSample_t *sample)
{
    if (sample == NULL) {
        return false;
    }

    uint8_t status = 0U;
    if (!ISM330DHCX_ReadReg(ISM330DHCX_REG_STATUS_REG, &status)) {
        return false;
    }

    /* Burst read OUT_TEMP_L (0x20) .. OUTZ_H_A (0x2D) = 14 bytes:
     * [0..1] temp, [2..7] gyro X/Y/Z, [8..13] accel X/Y/Z. */
    uint8_t buf[14];
    if (!ISM330DHCX_ReadMulti(ISM330DHCX_REG_OUT_TEMP_L, buf, sizeof(buf))) {
        return false;
    }

    sample->status_reg  = status;
    sample->temp_raw    = ISM330DHCX_LE16(&buf[0]);
    sample->gyro_raw_x  = ISM330DHCX_LE16(&buf[2]);
    sample->gyro_raw_y  = ISM330DHCX_LE16(&buf[4]);
    sample->gyro_raw_z  = ISM330DHCX_LE16(&buf[6]);
    sample->accel_raw_x = ISM330DHCX_LE16(&buf[8]);
    sample->accel_raw_y = ISM330DHCX_LE16(&buf[10]);
    sample->accel_raw_z = ISM330DHCX_LE16(&buf[12]);

    sample->imu_timestamp_25us = 0U;
#if IMU_ENABLE_TIMESTAMP
    (void)ISM330DHCX_ReadTimestamp(&sample->imu_timestamp_25us);
#endif

    return true;
}

bool ISM330DHCX_ReadTimestamp(uint32_t *timestamp_25us)
{
    if (timestamp_25us == NULL) {
        s_debug.last_hal_status = (uint32_t)HAL_ERROR;
        s_debug.last_hal_error = HAL_SPI_GetError(&hspi4);
        s_debug.read_multi_error_count++;
        return false;
    }

    uint8_t buf[4];
    if (!ISM330DHCX_ReadMulti(ISM330DHCX_REG_TIMESTAMP0, buf, sizeof(buf))) {
        return false;
    }

    *timestamp_25us = (uint32_t)buf[0] |
                      ((uint32_t)buf[1] << 8) |
                      ((uint32_t)buf[2] << 16) |
                      ((uint32_t)buf[3] << 24);
    return true;
}

void ISM330DHCX_GetDebugStats(ISM330DHCX_DebugStats_t *out)
{
    if (out == NULL) {
        return;
    }

    s_debug.expected_whoami = ISM330DHCX_WHO_AM_I_EXPECTED;
    *out = s_debug;
}

void ISM330DHCX_ResetDebugStats(void)
{
    memset(&s_debug, 0, sizeof(s_debug));
    s_debug.last_whoami = 0xFFU;
    s_debug.expected_whoami = ISM330DHCX_WHO_AM_I_EXPECTED;
    s_debug.last_hal_status = (uint32_t)HAL_OK;
}
