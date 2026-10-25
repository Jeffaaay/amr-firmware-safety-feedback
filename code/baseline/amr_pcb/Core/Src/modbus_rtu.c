/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus_rtu.c
  * @brief          : Minimal Modbus RTU helpers implementation.
  *
  * Battery runtime is read-only and polls with FC03 (read data registers). The
  * FC04 builder remains as a generic helper but is not used by runtime battery
  * polling. No FC06 write path exists.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "modbus_rtu.h"
#include <stddef.h>

static uint16_t Modbus_BuildReadRequest(uint8_t slave,
                                        uint8_t func,
                                        uint16_t start_addr,
                                        uint16_t count,
                                        uint8_t *out,
                                        uint16_t out_cap)
{
    if ((out == NULL) || (out_cap < 8U) || (count == 0U)) {
        return 0U;
    }

    out[0] = slave;
    out[1] = func;
    out[2] = (uint8_t)(start_addr >> 8);
    out[3] = (uint8_t)(start_addr & 0xFFU);
    out[4] = (uint8_t)(count >> 8);
    out[5] = (uint8_t)(count & 0xFFU);

    uint16_t crc = Modbus_CRC16(out, 6U);
    out[6] = (uint8_t)(crc & 0xFFU);        /* CRC low byte first */
    out[7] = (uint8_t)(crc >> 8);           /* CRC high byte */

    return 8U;
}

uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL) {
        return crc;
    }

    for (uint16_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }

    return crc;
}

uint16_t Modbus_BuildReadHoldingRequest(uint8_t slave,
                                        uint16_t start_addr,
                                        uint16_t count,
                                        uint8_t *out,
                                        uint16_t out_cap)
{
    return Modbus_BuildReadRequest(slave, MODBUS_FC_READ_HOLDING,
                                   start_addr, count, out, out_cap);
}

uint16_t Modbus_BuildReadInputRequest(uint8_t slave,
                                      uint16_t start_addr,
                                      uint16_t count,
                                      uint8_t *out,
                                      uint16_t out_cap)
{
    return Modbus_BuildReadRequest(slave, MODBUS_FC_READ_INPUT,
                                   start_addr, count, out, out_cap);
}

bool Modbus_ValidateReadResponse(const uint8_t *frame,
                                 uint16_t len,
                                 uint8_t slave,
                                 uint8_t expected_func,
                                 uint16_t expected_count)
{
    if (frame == NULL) {
        return false;
    }

    /* Minimum read response: addr + func + byte_count + 2 data + 2 CRC. */
    if (len < 7U) {
        return false;
    }

    if (frame[0] != slave) {
        return false;
    }

    /* Exception responses (func | 0x80) and any other function are rejected. */
    if (frame[1] != expected_func) {
        return false;
    }

    uint8_t byte_count = frame[2];
    if (byte_count != (uint8_t)(expected_count * 2U)) {
        return false;
    }

    uint16_t frame_len = (uint16_t)(3U + byte_count + 2U);
    if (len != frame_len) {
        return false;
    }

    return Modbus_CheckFrameCrc(frame, frame_len);
}

bool Modbus_CheckFrameCrc(const uint8_t *frame, uint16_t len)
{
    if ((frame == NULL) || (len < 4U)) {
        return false;
    }

    uint16_t crc_calc = Modbus_CRC16(frame, (uint16_t)(len - 2U));
    uint16_t crc_recv = (uint16_t)(frame[len - 2U] |
                                   ((uint16_t)frame[len - 1U] << 8));

    return (crc_calc == crc_recv);
}

bool Modbus_ValidateExceptionResponse(const uint8_t *frame,
                                      uint16_t len,
                                      uint8_t slave,
                                      uint8_t expected_func,
                                      uint8_t *exception_code)
{
    if ((frame == NULL) || (len != 5U)) {
        return false;
    }

    if (frame[0] != slave) {
        return false;
    }

    if (frame[1] != (uint8_t)(expected_func | MODBUS_EXCEPTION_FLAG)) {
        return false;
    }

    if (!Modbus_CheckFrameCrc(frame, len)) {
        return false;
    }

    if (exception_code != NULL) {
        *exception_code = frame[2];
    }

    return true;
}

bool Modbus_ParseU16BE(const uint8_t *data, uint16_t index, uint16_t *out)
{
    if ((data == NULL) || (out == NULL)) {
        return false;
    }

    *out = (uint16_t)(((uint16_t)data[index] << 8) | (uint16_t)data[index + 1U]);
    return true;
}
