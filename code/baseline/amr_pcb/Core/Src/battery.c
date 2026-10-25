/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : battery.c
  * @brief          : Battery BMS service (Modbus RTU read-only, USART2/RS485).
  *
 * Owns the USART2 + PG0 (RS485_WR1) half-duplex transport and the BMS poll
  * state machine. Runtime polling is read-only: this BMS PDF uses FC03 read-data
  * registers over Modbus RTU (one block covering AI registers 18..40). FC06 MOS
  * writes are supported by the PDF but are never built or transmitted here.
  * Battery_Update is non-blocking apart from the bounded request transmit; the
  * response wait is timestamp/state-machine based with no HAL_Delay. A battery
  * fault never stops motors. See docs/protocol_contract.md.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "battery.h"

#include "app_config.h"
#include "diagnostics.h"
#include "jetson_protocol.h"
#include "jetson_tx.h"
#include "main.h"
#include "modbus_rtu.h"

#include <string.h>

extern UART_HandleTypeDef huart2;

typedef enum
{
    BAT_STATE_UNINIT = 0,
    BAT_STATE_IDLE,
    BAT_STATE_TX_REQUEST,
    BAT_STATE_WAIT_RESPONSE,
    BAT_STATE_FRAME_READY,
    BAT_STATE_RECOVERY
} BatteryTxState_t;

typedef enum
{
    /* Single FC03 read-data query covering the vendor AI registers 18..40. */
    BAT_QUERY_LIVE_AI = 0
} BatteryQuery_t;

/* Live AI block, per the client Modbus PDF (FC03 read-data registers):
 *   start register 18 (0x0012), 23 registers (0x0017) -> registers 18..40.
 * Response = addr + func + byte_count(46) + 46 data + 2 CRC = 51 bytes.
 * Known-good request for slave 1: 01 03 00 12 00 17 A5 C1. */
#define BAT_LIVE_AI_START    0x0012U    /* register 18: data index 0 */
#define BAT_LIVE_AI_COUNT    0x0017U    /* 23 registers -> 18..40 */

#define BAT_RX_BUF_SIZE      64U        /* >= 51-byte live-AI response */

static volatile BatteryTxState_t s_state = BAT_STATE_UNINIT;
static BatteryStatus_t  s_status;
static BatteryFaultStats_t s_fault;
static uint32_t         s_fault_bits = 0U;
static bool             s_offline = false;

/* Scheduling (HAL tick deadlines, wrap-safe via signed compare). */
static uint32_t         s_next_live_ms = 0U;
static uint32_t         s_next_request_allowed_ms = 0U;

/* Active transaction. */
static BatteryQuery_t   s_active_query = BAT_QUERY_LIVE_AI;
static uint8_t          s_active_func = MODBUS_FC_READ_HOLDING;
static uint16_t         s_active_count = 0U;
static uint16_t         s_expected_len = 0U;
static uint8_t          s_tx_buf[8];
static uint16_t         s_tx_len = 0U;
static uint8_t          s_last_request[8];
static uint16_t         s_last_request_len = 0U;
static uint32_t         s_request_count = 0U;
static uint32_t         s_resp_start_ms = 0U;

/* RX (written from ISR context). */
static volatile uint8_t  s_rx_byte = 0U;
static volatile uint8_t  s_rx_buf[BAT_RX_BUF_SIZE];
static volatile uint16_t s_rx_len = 0U;
static volatile bool     s_rx_collecting = false;
static volatile bool     s_rx_armed = false;
static volatile bool     s_frame_ready = false;
static volatile bool     s_rx_overflow = false;
static volatile uint16_t s_frame_target_len = 0U;
static volatile bool     s_recovery_failure_pending = false;

/* RX lifecycle diagnostics. Counters are monotonic until Battery_Init(). */
static volatile uint32_t s_rx_irq_count = 0U;
static volatile uint32_t s_rx_total_bytes = 0U;
static volatile uint32_t s_rx_ignored_bytes = 0U;
static volatile uint16_t s_max_rx_len = 0U;
static volatile uint16_t s_timeout_rx_len = 0U;
static volatile uint32_t s_frame_ready_count = 0U;
static volatile uint32_t s_arm_ok_count = 0U;
static volatile uint32_t s_arm_busy_count = 0U;
static volatile uint32_t s_arm_error_count = 0U;
static volatile uint32_t s_rearm_ok_count = 0U;
static volatile uint32_t s_rearm_busy_count = 0U;
static volatile uint32_t s_rearm_error_count = 0U;
static volatile uint32_t s_ore_count = 0U;
static volatile uint32_t s_fe_count = 0U;
static volatile uint32_t s_ne_count = 0U;
static volatile uint32_t s_pe_count = 0U;
static volatile uint32_t s_last_hal_error = 0U;
static volatile uint32_t s_first_rx_delay_ms = 0U;
static volatile uint32_t s_first_rx_ms = 0U;
static volatile uint32_t s_last_rx_ms = 0U;
static volatile uint32_t s_timeout_first_rx_ms = 0U;
static volatile uint32_t s_timeout_last_rx_ms = 0U;
static volatile uint8_t  s_last_rx_byte = 0U;
static volatile uint8_t  s_last16[16];
static volatile uint8_t  s_last16_count = 0U;
static volatile bool     s_pg0_tx = false;

/* ---- RS485 direction control (PG0 / RS485_WR1) ---- */

static inline void Battery_RS485_Receive(void)
{
    HAL_GPIO_WritePin(RS485_WR1_GPIO_Port, RS485_WR1_Pin, GPIO_PIN_RESET); /* LOW = RX */
    s_pg0_tx = false;
}

static inline void Battery_RS485_Transmit(void)
{
    HAL_GPIO_WritePin(RS485_WR1_GPIO_Port, RS485_WR1_Pin, GPIO_PIN_SET);   /* HIGH = TX */
    s_pg0_tx = true;
}

static bool Battery_WaitTxComplete(uint32_t start_ms)
{
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {
        if ((HAL_GetTick() - start_ms) >= BATTERY_TX_TIMEOUT_MS) {
            return false;
        }
    }

    return true;
}

/* ---- helpers ---- */

static void Battery_EnterRecovery(void)
{
    s_rx_collecting = false;
    s_frame_ready = false;
    s_rx_armed = false;
    s_recovery_failure_pending = true;
    s_fault.uart_error_count++;
    s_fault_bits |= BATTERY_FAULT_UART;
    s_state = BAT_STATE_RECOVERY;
}

static bool Battery_ArmNextRxByte(bool rearm)
{
    HAL_StatusTypeDef status;

    if (!s_rx_collecting || (s_state != BAT_STATE_WAIT_RESPONSE) ||
        s_frame_ready || s_rx_armed) {
        if (rearm) {
            s_rearm_error_count++;
        } else {
            s_arm_error_count++;
        }
        Battery_EnterRecovery();
        return false;
    }

    /* Reserve ownership before enabling RX interrupts. A byte can arrive as
     * soon as HAL enables RXNE; setting this after HAL returns would let that
     * callback rearm and then be overwritten by the outer first-arm path. */
    s_rx_armed = true;
    status = HAL_UART_Receive_IT(&huart2, (uint8_t *)&s_rx_byte, 1U);
    if (status == HAL_OK) {
        if (rearm) {
            s_rearm_ok_count++;
        } else {
            s_arm_ok_count++;
        }
        return true;
    }

    s_rx_armed = false;
    if (status == HAL_BUSY) {
        if (rearm) {
            s_rearm_busy_count++;
        } else {
            s_arm_busy_count++;
        }
    } else {
        if (rearm) {
            s_rearm_error_count++;
        } else {
            s_arm_error_count++;
        }
    }

    Battery_EnterRecovery();
    return false;
}

static void Battery_ResetCurrentFrame(void)
{
    s_rx_len = 0U;
    s_rx_collecting = false;
    s_rx_armed = false;
    s_frame_ready = false;
    s_rx_overflow = false;
    s_frame_target_len = s_expected_len;
}

static bool Battery_StopOutstandingRxAndClearErrors(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    HAL_UART_StateTypeDef rx_state = huart2.RxState;

    s_rx_collecting = false;
    s_frame_ready = false;

    if (s_rx_armed ||
        (rx_state == HAL_UART_STATE_BUSY_RX) ||
        (rx_state == HAL_UART_STATE_BUSY_TX_RX)) {
        status = HAL_UART_AbortReceive(&huart2);
    }

    s_rx_armed = false;
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);

    if (status == HAL_OK) {
        huart2.ErrorCode = HAL_UART_ERROR_NONE;
        return true;
    }

    return false;
}

static void Battery_RememberRxByte(uint8_t byte)
{
    uint8_t count = s_last16_count;

    if (count < (uint8_t)sizeof(s_last16)) {
        s_last16[count] = byte;
        s_last16_count = (uint8_t)(count + 1U);
    } else {
        for (uint8_t i = 1U; i < (uint8_t)sizeof(s_last16); i++) {
            s_last16[i - 1U] = s_last16[i];
        }
        s_last16[sizeof(s_last16) - 1U] = byte;
    }
}

/* ---- ISR hooks (registered with the USART RX/error dispatcher) ---- */

void Battery_RxByteFromISR(void)
{
    uint32_t now_ms;
    uint16_t len;

    s_rx_irq_count++;
    s_rx_armed = false;

    if (!s_rx_collecting || (s_state != BAT_STATE_WAIT_RESPONSE) ||
        s_frame_ready) {
        s_rx_ignored_bytes++;
        return;
    }

    if ((s_rx_len >= BAT_RX_BUF_SIZE) ||
        ((s_frame_target_len != 0U) && (s_rx_len >= s_frame_target_len))) {
        s_rx_overflow = true;
        s_fault.framing_error_count++;
        Battery_EnterRecovery();
        return;
    }

    now_ms = HAL_GetTick();
    len = s_rx_len;
    s_rx_buf[len] = s_rx_byte;
    len = (uint16_t)(len + 1U);
    s_rx_len = len;
    s_rx_total_bytes++;
    s_last_rx_byte = s_rx_byte;
    Battery_RememberRxByte(s_rx_byte);

    if (len == 1U) {
        s_first_rx_ms = now_ms;
        s_first_rx_delay_ms = now_ms - s_resp_start_ms;
    }
    s_last_rx_ms = now_ms;
    if (len > s_max_rx_len) {
        s_max_rx_len = len;
    }

    if ((len == 2U) &&
        (s_rx_buf[0] == BATTERY_MODBUS_SLAVE_ADDR) &&
        (s_rx_buf[1] == (uint8_t)(s_active_func | MODBUS_EXCEPTION_FLAG))) {
        s_frame_target_len = 5U;
    }

    if ((s_frame_target_len != 0U) && (len == s_frame_target_len)) {
        s_rx_collecting = false;
        s_frame_ready = true;
        s_frame_ready_count++;
        s_state = BAT_STATE_FRAME_READY;
        return;
    }

    (void)Battery_ArmNextRxByte(true);
}

void Battery_UartErrorFromISR(void)
{
    uint32_t error = HAL_UART_GetError(&huart2);

    s_last_hal_error = error;
    if ((error & HAL_UART_ERROR_ORE) != 0U) {
        s_ore_count++;
    }
    if ((error & HAL_UART_ERROR_FE) != 0U) {
        s_fe_count++;
    }
    if ((error & HAL_UART_ERROR_NE) != 0U) {
        s_ne_count++;
    }
    if ((error & HAL_UART_ERROR_PE) != 0U) {
        s_pe_count++;
    }

    Battery_EnterRecovery();
}

static BatteryHealth_t Battery_ComputeHealth(void)
{
    if (s_fault.consecutive_failure_count >= BATTERY_OFFLINE_FAILURE_LIMIT) {
        return BATTERY_HEALTH_OFFLINE;
    }
    if (!s_status.valid_basic) {
        return BATTERY_HEALTH_INIT;
    }
    if (s_fault.consecutive_failure_count > 0U) {
        return BATTERY_HEALTH_DEGRADED;
    }
    return BATTERY_HEALTH_OK;
}

static void Battery_RecordSuccess(uint32_t now_ms)
{
    s_fault.successful_response_count++;
    s_fault.consecutive_failure_count = 0U;
    s_fault.last_good_response_ms = now_ms;
    s_offline = false;
    /* fault_bits reports current communication health. Historical counts stay
     * in BatteryFaultStats_t; UART faults stay latched until ResetFaults().
     */
    s_fault_bits &= ~(BATTERY_FAULT_TX |
                      BATTERY_FAULT_TIMEOUT |
                      BATTERY_FAULT_CRC |
                      BATTERY_FAULT_EXCEPTION |
                      BATTERY_FAULT_FRAME |
                      BATTERY_FAULT_OFFLINE);
}

static void Battery_RecordFailure(uint32_t now_ms, uint32_t fault_bit)
{
    (void)now_ms;
    s_fault.consecutive_failure_count++;
    s_fault_bits |= fault_bit;

    if (s_fault.consecutive_failure_count >= BATTERY_OFFLINE_FAILURE_LIMIT) {
        if (!s_offline) {
            s_offline = true;
            s_fault.offline_count++;
        }
        s_fault_bits |= BATTERY_FAULT_OFFLINE;
    }
}

static uint8_t Battery_ClampRsocPercent(uint16_t raw_rsoc)
{
    /* RSOC register: high byte reserved, low byte = remaining-capacity percent,
     * clamped to 0..100 per the vendor PDF. */
    uint16_t percent = (uint16_t)(raw_rsoc & 0x00FFU);

    if (percent > 100U) {
        percent = 100U;
    }

    return (uint8_t)percent;
}

/* Vendor 10 mA current: charge positive, discharge negative. The PDF encodes
 * discharge as (65535 - magnitude) (e.g. -10 A -> 65535 - 1000 = 64535), so this
 * is NOT plain two's complement (which would wrap at 65536). Decode with the
 * vendor's 65535 base.
 *
 * TODO(battery): this vendor current formula is slightly non-standard at the
 * sign boundary; verify it against real charge/discharge Modbus traces before
 * trusting the magnitude near +-32767.
 */
static int32_t Battery_DecodeVendorCurrent_mA(uint16_t raw)
{
    if (raw <= 32767U) {
        return (int32_t)raw * 10;               /* charge (positive) */
    }
    return -(int32_t)((uint32_t)(65535U - raw) * 10U); /* discharge (negative) */
}

static bool Battery_ParseRegU16(const uint8_t *data,
                                uint16_t reg_index,
                                uint16_t *out)
{
    return Modbus_ParseU16BE(data, (uint16_t)(reg_index * 2U), out);
}

static void Battery_ScheduleNext(uint32_t now_ms)
{
    switch (s_active_query) {
    case BAT_QUERY_LIVE_AI:
    default:
        s_next_live_ms = now_ms + BATTERY_LIVE_POLL_MS;
        break;
    }
}

static void Battery_StartQuery(BatteryQuery_t query)
{
    uint16_t start_addr = BAT_LIVE_AI_START;
    uint16_t count = BAT_LIVE_AI_COUNT;

    s_active_query = query;
    s_active_func = MODBUS_FC_READ_HOLDING;   /* PDF FC03 read-data registers */
    s_active_count = count;
    s_expected_len = (uint16_t)(5U + (count * 2U)); /* addr+func+bytecount + data + crc */
    /* With slave 1, BAT_QUERY_LIVE_AI must build: 01 03 00 12 00 17 A5 C1. */
    s_tx_len = Modbus_BuildReadHoldingRequest(BATTERY_MODBUS_SLAVE_ADDR,
                                              start_addr, count,
                                              s_tx_buf, sizeof(s_tx_buf));
    s_last_request_len = s_tx_len;
    if (s_last_request_len > sizeof(s_last_request)) {
        s_last_request_len = sizeof(s_last_request);
    }
    memcpy(s_last_request, s_tx_buf, s_last_request_len);
    s_state = BAT_STATE_TX_REQUEST;
}

static void Battery_DecodeResponse(uint32_t now_ms)
{
    /* Data payload begins after addr, func, byte_count. reg[i] maps to vendor AI
     * register (18 + i): reg[0] == register 18 up to reg[22] == register 40.
     * Values are big-endian per register. */
    const uint8_t *data = (const uint8_t *)&s_rx_buf[3];
    uint16_t reg[BAT_LIVE_AI_COUNT];

    for (uint16_t i = 0U; i < BAT_LIVE_AI_COUNT; i++) {
        reg[i] = 0U;
        (void)Battery_ParseRegU16(data, i, &reg[i]);
    }

    const uint16_t reg18 = reg[0];    /* total voltage, 10 mV */
    const uint16_t reg19 = reg[1];    /* charge/discharge current, 10 mA (vendor) */
    const uint16_t reg20 = reg[2];    /* remaining capacity, 10 mAh */
    /* reg[3..18] == registers 21..36 == cell 1..16 voltage, mV */
    const uint16_t reg37 = reg[19];   /* battery temperature 1, 0.1 K */
    const uint16_t reg38 = reg[20];   /* battery temperature 2, 0.1 K */
    const uint16_t reg39 = reg[21];   /* nominal capacity, 10 mAh */
    const uint16_t reg40 = reg[22];   /* RSOC: low byte = percent, high reserved */

    s_status.raw_total_voltage = reg18;
    s_status.voltage_mV = (uint32_t)reg18 * 10U;

    /* raw_current keeps the reg19 word for diagnostics; current_mA uses the
     * vendor sign convention (charge positive, discharge negative). */
    s_status.raw_current = (int16_t)reg19;
    s_status.current_mA = Battery_DecodeVendorCurrent_mA(reg19);

    s_status.raw_remaining_capacity = reg20;
    s_status.remaining_mAh = (uint32_t)reg20 * 10U;

    for (uint16_t i = 0U; i < 16U; i++) {
        s_status.cell_mv[i] = reg[3U + i];
    }
    s_status.valid_cells = true;

    /* PDF: actual degC = (raw - 2731) / 10, so deci-Celsius = raw - 2731. */
    s_status.raw_temp1 = reg37;
    s_status.raw_temp2 = reg38;
    s_status.temp1_dC = (int16_t)((int16_t)reg37 - 2731);
    s_status.temp2_dC = (int16_t)((int16_t)reg38 - 2731);

    s_status.raw_nominal_capacity = reg39;
    s_status.nominal_mAh = (uint32_t)reg39 * 10U;

    s_status.raw_rsoc = reg40;
    s_status.rsoc_percent = Battery_ClampRsocPercent(reg40);
    /* Legacy compatibility field: this PDF reports whole-percent RSOC, not tenths
     * of a percent. Keep raw_soc_tenths_percent = rsoc_percent * 10 so existing
     * readers stay in range; it is no longer a 0.1% raw value. */
    s_status.raw_soc_tenths_percent = (uint16_t)((uint16_t)s_status.rsoc_percent * 10U);

    s_status.last_update_ms = now_ms;
    s_status.valid_basic = true;
    s_status.valid_temp_soc = true;
    s_status.valid_capacity = true;
    /* valid_state stays false: this PDF map defines no alarm/charge-state
     * register, so alarm_status/charge_discharge_state/charge_mode are unused. */
}

/* ---- public API ---- */

void Battery_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    memset(&s_fault, 0, sizeof(s_fault));
    s_fault_bits = 0U;
    s_offline = false;
    s_expected_len = (uint16_t)(5U + (BAT_LIVE_AI_COUNT * 2U));
    s_recovery_failure_pending = false;

    s_rx_irq_count = 0U;
    s_rx_total_bytes = 0U;
    s_rx_ignored_bytes = 0U;
    s_max_rx_len = 0U;
    s_timeout_rx_len = 0U;
    s_frame_ready_count = 0U;
    s_arm_ok_count = 0U;
    s_arm_busy_count = 0U;
    s_arm_error_count = 0U;
    s_rearm_ok_count = 0U;
    s_rearm_busy_count = 0U;
    s_rearm_error_count = 0U;
    s_ore_count = 0U;
    s_fe_count = 0U;
    s_ne_count = 0U;
    s_pe_count = 0U;
    s_last_hal_error = 0U;
    s_first_rx_delay_ms = 0U;
    s_first_rx_ms = 0U;
    s_last_rx_ms = 0U;
    s_timeout_first_rx_ms = 0U;
    s_timeout_last_rx_ms = 0U;
    s_last_rx_byte = 0U;
    s_last16_count = 0U;
    memset((void *)s_last16, 0, sizeof(s_last16));

    Battery_ResetCurrentFrame();

    Battery_RS485_Receive();

    JetsonProtocol_SetUart2RxByteCallback(Battery_RxByteFromISR);
    JetsonProtocol_SetUart2ErrorCallback(Battery_UartErrorFromISR);

    uint32_t now = HAL_GetTick();
    s_next_live_ms = now;
    s_next_request_allowed_ms = now;

    if (Battery_StopOutstandingRxAndClearErrors()) {
        s_state = BAT_STATE_IDLE;
    } else {
        Battery_EnterRecovery();
    }
}

void Battery_Update(uint32_t now_ms)
{
    switch (s_state) {
    case BAT_STATE_UNINIT:
        break;

    case BAT_STATE_IDLE:
        if ((int32_t)(now_ms - s_next_request_allowed_ms) < 0) {
            break;
        }

        if ((int32_t)(now_ms - s_next_live_ms) >= 0) {
            Battery_StartQuery(BAT_QUERY_LIVE_AI);
        }
        break;

    case BAT_STATE_TX_REQUEST: {
        uint32_t transaction_ms;

        if (s_tx_len == 0U) {
            Battery_RecordFailure(now_ms, BATTERY_FAULT_TX);
            Battery_ScheduleNext(now_ms);
            s_state = BAT_STATE_IDLE;
            break;
        }

        if (!Battery_StopOutstandingRxAndClearErrors()) {
            Battery_EnterRecovery();
            break;
        }
        Battery_ResetCurrentFrame();
        s_first_rx_delay_ms = 0U;
        s_first_rx_ms = 0U;
        s_last_rx_ms = 0U;
        s_last_rx_byte = 0U;
        s_last16_count = 0U;
        memset((void *)s_last16, 0, sizeof(s_last16));

        Battery_RS485_Transmit();
        s_request_count++;
        HAL_StatusTypeDef st = HAL_UART_Transmit(&huart2, s_tx_buf, s_tx_len,
                                                 BATTERY_TX_TIMEOUT_MS);
        if ((st == HAL_OK) && !Battery_WaitTxComplete(HAL_GetTick())) {
            st = HAL_TIMEOUT;
        }
        Battery_RS485_Receive();
        transaction_ms = HAL_GetTick();
        s_next_request_allowed_ms =
            transaction_ms + BATTERY_MIN_REQUEST_INTERVAL_MS;

        if (st != HAL_OK) {
            s_fault.tx_error_count++;
            Battery_RecordFailure(transaction_ms, BATTERY_FAULT_TX);
            Battery_ScheduleNext(transaction_ms);
            s_state = BAT_STATE_IDLE;
        } else {
            s_resp_start_ms = transaction_ms;
            s_state = BAT_STATE_WAIT_RESPONSE;
            s_rx_collecting = true;
            s_frame_target_len = s_expected_len;
            (void)Battery_ArmNextRxByte(false);
        }
        break;
    }

    case BAT_STATE_WAIT_RESPONSE:
        if ((now_ms - s_resp_start_ms) >= BATTERY_RESPONSE_TIMEOUT_MS) {
            s_timeout_rx_len = s_rx_len;
            s_timeout_first_rx_ms = s_first_rx_ms;
            s_timeout_last_rx_ms = s_last_rx_ms;
            s_rx_collecting = false;
            s_frame_ready = false;
            s_fault.rx_timeout_count++;
            Battery_RecordFailure(now_ms, BATTERY_FAULT_TIMEOUT);
            Battery_ScheduleNext(now_ms);
            s_state = BAT_STATE_RECOVERY;
        }
        break;

    case BAT_STATE_FRAME_READY: {
        uint16_t len = s_rx_len;
        const uint8_t *frame = (const uint8_t *)s_rx_buf;
        uint8_t exception_code = 0U;

        s_frame_ready = false;
        if ((len == 5U) &&
            (frame[0] == BATTERY_MODBUS_SLAVE_ADDR) &&
            (frame[1] == (uint8_t)(s_active_func | MODBUS_EXCEPTION_FLAG))) {
            if (Modbus_ValidateExceptionResponse(frame, len,
                                                 BATTERY_MODBUS_SLAVE_ADDR,
                                                 s_active_func,
                                                 &exception_code)) {
                (void)exception_code;
                s_fault.exception_response_count++;
                Battery_RecordFailure(now_ms, BATTERY_FAULT_EXCEPTION);
            } else {
                s_fault.crc_error_count++;
                Battery_RecordFailure(now_ms, BATTERY_FAULT_CRC);
            }
        } else if (len != s_expected_len) {
            s_fault.length_error_count++;
            s_fault.parse_error_count++;
            Battery_RecordFailure(now_ms, BATTERY_FAULT_FRAME);
        } else if (frame[0] != BATTERY_MODBUS_SLAVE_ADDR) {
            s_fault.slave_error_count++;
            s_fault.parse_error_count++;
            Battery_RecordFailure(now_ms, BATTERY_FAULT_FRAME);
        } else if (frame[1] != s_active_func) {
            s_fault.function_error_count++;
            s_fault.parse_error_count++;
            Battery_RecordFailure(now_ms, BATTERY_FAULT_FRAME);
        } else if (frame[2] != (uint8_t)(s_active_count * 2U)) {
            s_fault.byte_count_error_count++;
            s_fault.parse_error_count++;
            Battery_RecordFailure(now_ms, BATTERY_FAULT_FRAME);
        } else if (!Modbus_CheckFrameCrc(frame, len)) {
            s_fault.crc_error_count++;
            Battery_RecordFailure(now_ms, BATTERY_FAULT_CRC);
        } else if (Modbus_ValidateReadResponse(frame, len,
                                               BATTERY_MODBUS_SLAVE_ADDR,
                                               s_active_func,
                                               s_active_count)) {
            Battery_DecodeResponse(now_ms);
            Battery_RecordSuccess(now_ms);
        } else {
            s_fault.framing_error_count++;
            s_fault.parse_error_count++;
            Battery_RecordFailure(now_ms, BATTERY_FAULT_FRAME);
        }

        Battery_ScheduleNext(now_ms);
        s_state = BAT_STATE_IDLE;
        break;
    }

    case BAT_STATE_RECOVERY:
        if (!Battery_StopOutstandingRxAndClearErrors()) {
            break;
        }
        if (s_recovery_failure_pending) {
            s_recovery_failure_pending = false;
            Battery_RecordFailure(now_ms, BATTERY_FAULT_UART);
            Battery_ScheduleNext(now_ms);
        }
        Battery_ResetCurrentFrame();
        s_state = BAT_STATE_IDLE;
        break;

    default:
        s_state = BAT_STATE_RECOVERY;
        break;
    }
}

bool Battery_GetStatus(BatteryStatus_t *out)
{
    if (out == NULL) {
        return false;
    }

    uint32_t now = HAL_GetTick();
    s_status.age_ms = s_status.valid_basic ? (now - s_status.last_update_ms) : 0xFFFFFFFFU;
    s_status.health = (uint8_t)Battery_ComputeHealth();
    s_status.fault_bits = s_fault_bits;

    *out = s_status;
    return s_status.valid_basic;
}

BatteryHealth_t Battery_GetHealth(void)
{
    return Battery_ComputeHealth();
}

BatteryFaultStats_t Battery_GetFaultStats(void)
{
    return s_fault;
}

BatteryRxDiagnostics_t Battery_GetRxDiagnostics(void)
{
    BatteryRxDiagnostics_t diag;

    memset(&diag, 0, sizeof(diag));
    diag.rx_irq_count = s_rx_irq_count;
    diag.rx_total_bytes = s_rx_total_bytes;
    diag.rx_ignored_bytes = s_rx_ignored_bytes;
    diag.current_rx_len = s_rx_len;
    diag.max_rx_len = s_max_rx_len;
    diag.timeout_rx_len = s_timeout_rx_len;
    diag.frame_ready_count = s_frame_ready_count;

    diag.arm_ok_count = s_arm_ok_count;
    diag.arm_busy_count = s_arm_busy_count;
    diag.arm_error_count = s_arm_error_count;
    diag.rearm_ok_count = s_rearm_ok_count;
    diag.rearm_busy_count = s_rearm_busy_count;
    diag.rearm_error_count = s_rearm_error_count;

    diag.uart_error_count = s_fault.uart_error_count;
    diag.ore_count = s_ore_count;
    diag.fe_count = s_fe_count;
    diag.ne_count = s_ne_count;
    diag.pe_count = s_pe_count;
    diag.last_hal_error = s_last_hal_error;

    diag.first_rx_delay_ms = s_first_rx_delay_ms;
    diag.first_rx_ms = s_first_rx_ms;
    diag.last_rx_ms = s_last_rx_ms;
    diag.timeout_first_rx_ms = s_timeout_first_rx_ms;
    diag.timeout_last_rx_ms = s_timeout_last_rx_ms;
    diag.last_rx_byte = s_last_rx_byte;
    diag.last16_count = s_last16_count;
    for (uint8_t i = 0U; i < (uint8_t)sizeof(diag.last16); i++) {
        diag.last16[i] = s_last16[i];
    }

    diag.collecting = s_rx_collecting;
    diag.frame_ready = s_frame_ready;
    diag.rx_armed = s_rx_armed;
    diag.hal_rx_state = (uint32_t)huart2.RxState;
    diag.pg0_tx = s_pg0_tx;
    return diag;
}

void Battery_ResetFaults(void)
{
    memset(&s_fault, 0, sizeof(s_fault));
    s_fault_bits = 0U;
    s_offline = false;
}

void Battery_SendJetsonStatus_1000ms(void)
{
    static uint32_t last_publish_ms = 0U;
    uint32_t now = HAL_GetTick();

    if ((now - last_publish_ms) < BATTERY_LIVE_POLL_MS) {
        return;
    }
    last_publish_ms = now;

    uint16_t min_cell = 0U;
    uint16_t max_cell = 0U;
    if (s_status.valid_cells) {
        min_cell = s_status.cell_mv[0];
        max_cell = s_status.cell_mv[0];
        for (uint16_t i = 1U; i < 16U; i++) {
            if (s_status.cell_mv[i] < min_cell) {
                min_cell = s_status.cell_mv[i];
            }
            if (s_status.cell_mv[i] > max_cell) {
                max_cell = s_status.cell_mv[i];
            }
        }
    }

    uint32_t age = s_status.valid_basic ? (now - s_status.last_update_ms) : 0xFFFFFFFFU;
    BatteryHealth_t health = Battery_ComputeHealth();

    /* BAT t_ms health fault age voltage_mV current_mA remaining_mAh rsoc
     *     temp1_dC temp2_dC min_cell_mV max_cell_mV */
    JetsonTx_Printf(JETSON_TX_PRIORITY_STATUS,
                "BAT %lu %u %lu %lu %lu %ld %lu %u %d %d %u %u\r\n",
                (unsigned long)now,
                (unsigned)health,
                (unsigned long)s_fault_bits,
                (unsigned long)age,
                (unsigned long)s_status.voltage_mV,
                (long)s_status.current_mA,
                (unsigned long)s_status.remaining_mAh,
                (unsigned)s_status.rsoc_percent,
                (int)s_status.temp1_dC,
                (int)s_status.temp2_dC,
                (unsigned)min_cell,
                (unsigned)max_cell);
}

void Battery_PrintDebug(uint32_t request_nonce)
{
    static uint16_t s_battery_snapshot_sequence = 0U;
    uint32_t now = HAL_GetTick();
    uint32_t age = s_status.valid_basic ? (now - s_status.last_update_ms) : 0xFFFFFFFFU;
    uint8_t req[8] = {0};
    BatteryRxDiagnostics_t rx = Battery_GetRxDiagnostics();
    uint32_t last_rx_age =
        (rx.last_rx_ms != 0U) ? (now - rx.last_rx_ms) : 0xFFFFFFFFU;
    unsigned snap;

    for (uint16_t i = 0U; (i < s_last_request_len) && (i < sizeof(req)); i++) {
        req[i] = s_last_request[i];
    }

    /* All three lines of one response share snap= and echo req= so a host
     * can never stitch lines from different snapshots together. */
    s_battery_snapshot_sequence++;
    snap = (unsigned)s_battery_snapshot_sequence;
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "BAT_DEBUG 1/3 slave=%u dir=PG0 RS485_WR1 polarity=HIGH_TX_LOW_RX poll=%lu resp=%lu snap=%u req=%08lX\r\n",
                (unsigned)BATTERY_MODBUS_SLAVE_ADDR,
                (unsigned long)s_request_count,
                (unsigned long)s_fault.successful_response_count,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "BAT_DEBUG 2/3 timeout=%lu crc=%lu exception=%lu uart=%lu age=%lu snap=%u req=%08lX\r\n",
                (unsigned long)s_fault.rx_timeout_count,
                (unsigned long)s_fault.crc_error_count,
                (unsigned long)s_fault.exception_response_count,
                (unsigned long)s_fault.uart_error_count,
                (unsigned long)age,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "BAT_DEBUG 3/3 last_req=%02X %02X %02X %02X %02X %02X %02X %02X snap=%u req=%08lX\r\n",
                req[0], req[1], req[2], req[3], req[4], req[5], req[6], req[7],
                snap, (unsigned long)request_nonce);

    /* Keep the three-line BAT_DEBUG wire contract intact. These companion
     * lifecycle lines are intentionally separate so existing Jetson parsers
     * continue accepting the original atomic snapshot unchanged. */
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "BAT_RX_DEBUG 1/5 irq=%lu total=%lu ignored=%lu current=%u max=%u timeout_len=%u ready=%lu snap=%u req=%08lX\r\n",
                (unsigned long)rx.rx_irq_count,
                (unsigned long)rx.rx_total_bytes,
                (unsigned long)rx.rx_ignored_bytes,
                (unsigned)rx.current_rx_len,
                (unsigned)rx.max_rx_len,
                (unsigned)rx.timeout_rx_len,
                (unsigned long)rx.frame_ready_count,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "BAT_RX_DEBUG 2/5 arm_ok=%lu arm_busy=%lu arm_error=%lu rearm_ok=%lu rearm_busy=%lu rearm_error=%lu snap=%u req=%08lX\r\n",
                (unsigned long)rx.arm_ok_count,
                (unsigned long)rx.arm_busy_count,
                (unsigned long)rx.arm_error_count,
                (unsigned long)rx.rearm_ok_count,
                (unsigned long)rx.rearm_busy_count,
                (unsigned long)rx.rearm_error_count,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "BAT_RX_DEBUG 3/5 uart=%lu ore=%lu fe=%lu ne=%lu pe=%lu hal_error=%08lX collecting=%u ready=%u armed=%u rxstate=%lu pg0=%u snap=%u req=%08lX\r\n",
                (unsigned long)rx.uart_error_count,
                (unsigned long)rx.ore_count,
                (unsigned long)rx.fe_count,
                (unsigned long)rx.ne_count,
                (unsigned long)rx.pe_count,
                (unsigned long)rx.last_hal_error,
                rx.collecting ? 1U : 0U,
                rx.frame_ready ? 1U : 0U,
                rx.rx_armed ? 1U : 0U,
                (unsigned long)rx.hal_rx_state,
                rx.pg0_tx ? 1U : 0U,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "BAT_RX_DEBUG 4/5 first_delay=%lu first_ms=%lu last_age=%lu timeout_first=%lu timeout_last=%lu last_byte=%02X last16_count=%u snap=%u req=%08lX\r\n",
                (unsigned long)rx.first_rx_delay_ms,
                (unsigned long)rx.first_rx_ms,
                (unsigned long)last_rx_age,
                (unsigned long)rx.timeout_first_rx_ms,
                (unsigned long)rx.timeout_last_rx_ms,
                rx.last_rx_byte,
                (unsigned)rx.last16_count,
                snap, (unsigned long)request_nonce);
    JetsonTx_Printf(JETSON_TX_PRIORITY_DEBUG,
                "BAT_RX_DEBUG 5/5 last16=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X snap=%u req=%08lX\r\n",
                rx.last16[0], rx.last16[1], rx.last16[2], rx.last16[3],
                rx.last16[4], rx.last16[5], rx.last16[6], rx.last16[7],
                rx.last16[8], rx.last16[9], rx.last16[10], rx.last16[11],
                rx.last16[12], rx.last16[13], rx.last16[14], rx.last16[15],
                snap, (unsigned long)request_nonce);
}
