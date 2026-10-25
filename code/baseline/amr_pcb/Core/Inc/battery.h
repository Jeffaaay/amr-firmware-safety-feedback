/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : battery.h
  * @brief          : Battery BMS service (Modbus RTU read-only over USART2/RS485).
  *
  * Owns BMS state/timing/fault and the latest decoded battery data, plus the
 * USART2 + PG0 (RS485_WR1) half-duplex transport and the Jetson BAT line.
  * Runtime polling is read-only: this BMS PDF uses FC03 read-data registers over
  * Modbus RTU for live data (AI registers 18..40). It never writes BMS MOS state
  * (FC06, defined by the PDF at register 0x0037) and never controls motors.
  * See docs/protocol_contract.md.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef BATTERY_H
#define BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    BATTERY_HEALTH_INIT = 0,
    BATTERY_HEALTH_OK,
    BATTERY_HEALTH_DEGRADED,
    BATTERY_HEALTH_OFFLINE,
    BATTERY_HEALTH_FAULT
} BatteryHealth_t;

/* Current fault bitmask values reported in BatteryStatus_t.fault_bits.
 * Transient communication bits are cleared after a valid response; counters in
 * BatteryFaultStats_t hold history. UART faults remain latched until
 * Battery_ResetFaults() because they indicate a hardware/peripheral error path.
 */
#define BATTERY_FAULT_TX        (1U << 0)
#define BATTERY_FAULT_TIMEOUT   (1U << 1)
#define BATTERY_FAULT_CRC       (1U << 2)
#define BATTERY_FAULT_EXCEPTION (1U << 3)
#define BATTERY_FAULT_OFFLINE   (1U << 4)
#define BATTERY_FAULT_UART      (1U << 5)
#define BATTERY_FAULT_FRAME     (1U << 6)

typedef struct
{
    /* Populated from the FC03 live-AI block (registers 18..40). Fields this PDF
     * map does not provide stay zero/false: raw_power/power_dW,
     * raw_corrected_capacity/corrected_mAh, and the alarm/charge-state fields
     * (valid_state stays false). */
    uint32_t last_update_ms;
    uint32_t age_ms;

    uint16_t raw_total_voltage;
    int16_t  raw_current;
    uint32_t raw_power;
    uint32_t raw_remaining_capacity;
    uint16_t cell_mv[16];
    uint16_t raw_temp1;
    uint16_t raw_temp2;
    uint32_t raw_nominal_capacity;
    uint32_t raw_corrected_capacity;
    uint16_t raw_rsoc;
    uint16_t raw_soc_tenths_percent;
    uint16_t alarm_status;
    uint16_t charge_discharge_state;
    uint16_t charge_mode;

    uint32_t voltage_mV;
    int32_t  current_mA;
    uint32_t power_dW;
    uint32_t remaining_mAh;
    int16_t  temp1_dC;
    int16_t  temp2_dC;
    uint32_t nominal_mAh;
    uint32_t corrected_mAh;
    /* Remaining-capacity percent for the BAT line: low byte of the RSOC register
     * (register 40), clamped to 0..100. raw_soc_tenths_percent is kept only for
     * compatibility (= rsoc_percent * 10); this PDF reports whole percent, not
     * tenths of a percent.
     */
    uint8_t  rsoc_percent;

    uint8_t  health;
    uint32_t fault_bits;
    bool     valid_basic;
    bool     valid_temp_soc;
    bool     valid_capacity;
    bool     valid_state;
    bool     valid_cells;
} BatteryStatus_t;

typedef struct
{
    uint32_t tx_error_count;
    uint32_t rx_timeout_count;
    uint32_t crc_error_count;
    uint32_t exception_response_count;
    uint32_t parse_error_count;
    uint32_t length_error_count;
    uint32_t slave_error_count;
    uint32_t function_error_count;
    uint32_t byte_count_error_count;
    uint32_t framing_error_count;
    uint32_t consecutive_failure_count;
    uint32_t successful_response_count;
    uint32_t offline_count;
    uint32_t uart_error_count;
    uint32_t last_good_response_ms;
} BatteryFaultStats_t;

typedef struct
{
    uint32_t rx_irq_count;
    uint32_t rx_total_bytes;
    uint32_t rx_ignored_bytes;
    uint16_t current_rx_len;
    uint16_t max_rx_len;
    uint16_t timeout_rx_len;
    uint32_t frame_ready_count;

    uint32_t arm_ok_count;
    uint32_t arm_busy_count;
    uint32_t arm_error_count;
    uint32_t rearm_ok_count;
    uint32_t rearm_busy_count;
    uint32_t rearm_error_count;

    uint32_t uart_error_count;
    uint32_t ore_count;
    uint32_t fe_count;
    uint32_t ne_count;
    uint32_t pe_count;
    uint32_t last_hal_error;

    uint32_t first_rx_delay_ms;
    uint32_t first_rx_ms;
    uint32_t last_rx_ms;
    uint32_t timeout_first_rx_ms;
    uint32_t timeout_last_rx_ms;
    uint8_t  last_rx_byte;
    uint8_t  last16[16];
    uint8_t  last16_count;

    bool     collecting;
    bool     frame_ready;
    bool     rx_armed;
    uint32_t hal_rx_state;
    bool     pg0_tx;
} BatteryRxDiagnostics_t;

void            Battery_Init(void);
void            Battery_Update(uint32_t now_ms);
bool            Battery_GetStatus(BatteryStatus_t *out);
BatteryHealth_t Battery_GetHealth(void);
BatteryFaultStats_t Battery_GetFaultStats(void);
BatteryRxDiagnostics_t Battery_GetRxDiagnostics(void);
void            Battery_ResetFaults(void);
void            Battery_SendJetsonStatus_1000ms(void);
void            Battery_PrintDebug(uint32_t request_nonce);

/* Interrupt-context hooks, dispatched from the USART RX/error callback for
 * USART2 (registered with jetson_protocol). Keep these short. */
void            Battery_RxByteFromISR(void);
void            Battery_UartErrorFromISR(void);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
