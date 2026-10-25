/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus_rtu.h
  * @brief          : Minimal Modbus RTU helpers (CRC, read build/validate).
  *
  * Pure protocol layer with no HAL/peripheral dependency. Battery runtime is
  * read-only and polls with FC03 (read data registers); an FC04 builder also
  * exists as a generic helper but is unused by runtime battery polling. Function
  * code 06 (write single register / MOS state at 0x0037) is PROHIBITED in
  * runtime and is never built or transmitted.
  * See docs/protocol_contract.md.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define MODBUS_FC_READ_HOLDING            0x03U
#define MODBUS_FC_READ_INPUT              0x04U
/* FC06 write-single-register is prohibited in v1 runtime. Reference only;
 * no builder/transmitter exists for it. */
#define MODBUS_FC_WRITE_SINGLE_PROHIBITED 0x06U

#define MODBUS_EXCEPTION_FLAG             0x80U

/* CRC-16 (Modbus): polynomial 0xA001, initial value 0xFFFF. */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len);

/* Build an FC03 read-data (holding) registers request into out (needs 8 bytes).
 * This is the function code used for runtime battery polling per the client BMS
 * Modbus PDF. Returns the frame length, or 0 on bad arguments / insufficient
 * capacity.
 *
 * Known-good CRC checks:
 *   01 03 00 12 00 17 A5 C1   (battery live-AI block, registers 18..40)
 *   01 03 00 02 00 02 65 CB
 */
uint16_t Modbus_BuildReadHoldingRequest(uint8_t slave,
                                        uint16_t start_addr,
                                        uint16_t count,
                                        uint8_t *out,
                                        uint16_t out_cap);

/* Build an FC04 read-input-registers request into out (needs 8 bytes).
 * Generic helper only; NOT used by runtime battery polling (which uses FC03).
 * Returns the frame length, or 0 on bad arguments / insufficient capacity.
 *
 * Known-good CRC checks:
 *   01 04 00 00 00 06 70 08
 *   01 04 00 10 00 02 70 0E
 */
uint16_t Modbus_BuildReadInputRequest(uint8_t slave,
                                      uint16_t start_addr,
                                      uint16_t count,
                                      uint8_t *out,
                                      uint16_t out_cap);

/* Validate a read response: slave match, expected function, byte count matches
 * expected_count*2, length consistent, and CRC valid. Returns false for
 * exception responses and any malformed frame. */
bool Modbus_ValidateReadResponse(const uint8_t *frame,
                                 uint16_t len,
                                 uint8_t slave,
                                 uint8_t expected_func,
                                 uint16_t expected_count);

/* Validate only the CRC over a complete Modbus RTU frame. */
bool Modbus_CheckFrameCrc(const uint8_t *frame, uint16_t len);

/* Validate a 5-byte Modbus exception response:
 * addr + (expected_func | 0x80) + exception_code + crc_lo + crc_hi.
 * The CRC must pass before the frame is treated as a real exception. */
bool Modbus_ValidateExceptionResponse(const uint8_t *frame,
                                      uint16_t len,
                                      uint8_t slave,
                                      uint8_t expected_func,
                                      uint8_t *exception_code);

/* Read a big-endian uint16 at byte offset index from data. */
bool Modbus_ParseU16BE(const uint8_t *data, uint16_t index, uint16_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_H */
