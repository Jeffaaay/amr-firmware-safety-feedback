/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : jetson_protocol.c
  * @brief          : Jetson UART command protocol dispatch.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "jetson_protocol.h"

#include "app_config.h"
#include "battery.h"
#include "can_trace.h"
#include "command_result.h"
#include "diagnostics.h"
#include "drive.h"
#include "drive_target.h"
#include "firmware_identity.h"
#include "imu.h"
#include "jetson_tx.h"
#include "lift.h"
#include "main.h"
#include "status_light.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* The blocking RAW backdoor must be compiled out of the RELEASE_TEST
 * profile unconditionally; a release firmware may never pause the safety
 * supervisors for a test move. */
#if (FIRMWARE_BUILD_PROFILE_ID == FIRMWARE_PROFILE_RELEASE_TEST) && \
    (ENABLE_DANGEROUS_RAW_TEST != 0)
#error "ENABLE_DANGEROUS_RAW_TEST must be 0 in the RELEASE_TEST profile"
#endif

typedef struct
{
    uint8_t byte;
    uint32_t arrival_tick;
    uint32_t stream_generation;
} JetsonRxEntry_t;

typedef struct
{
    volatile uint16_t head;
    volatile uint16_t tail;
    JetsonRxEntry_t buf[UART_RX_RING_SIZE];
} JetsonRxRing_t;

#define JETSON_RX_DMA_BUFFER_SIZE       4096U
#define JETSON_RX_DMA_BUFFER_MASK       (JETSON_RX_DMA_BUFFER_SIZE - 1U)
#define JETSON_RX_DMA_HALF_SIZE         (JETSON_RX_DMA_BUFFER_SIZE / 2U)
#define JETSON_RX_DMA_DRAIN_BUDGET      1024U
#define JETSON_RX_DMA_COPY_CHUNK         256U
#define JETSON_RX_UART_BAUD           460800U
#define JETSON_RX_UART_BITS_PER_BYTE      10U

_Static_assert((JETSON_RX_DMA_BUFFER_SIZE &
                JETSON_RX_DMA_BUFFER_MASK) == 0U,
               "Jetson RX DMA buffer must be a power of two");
_Static_assert(JETSON_RX_DMA_BUFFER_SIZE <= UINT16_MAX,
               "Jetson RX DMA buffer must fit HAL uint16_t Size");
_Static_assert(JETSON_RX_DMA_COPY_CHUNK <= JETSON_RX_DMA_DRAIN_BUDGET,
               "Jetson RX DMA copy chunk must fit the drain budget");

static JetsonRxRing_t s_jetson_rx_ring;
/* FLASH builds place .bss in DMA1-accessible AXI SRAM (RAM_D1,
 * 0x24000000). D-cache is not enabled by this firmware. Keep the buffer
 * 32-byte aligned so a future cache policy can give it whole cache lines;
 * cache coherency must be revisited before D-cache is ever enabled. */
static uint8_t s_jetson_rx_dma_buffer[JETSON_RX_DMA_BUFFER_SIZE]
    __attribute__((aligned(32)));
/* Main-owned rollback buffer. DMA never writes this storage. A copied chunk
 * is committed only after a second producer/epoch check proves that none of
 * its source locations could have been overwritten during the copy. */
static uint8_t s_jetson_rx_dma_copy[JETSON_RX_DMA_COPY_CHUNK];
static volatile uint32_t s_jetson_rx_overflow_count = 0U;

/* Every corruption source enters this ONE fail-closed transition.  It
 * increments a monotonic stream generation, flushes queued entries, and
 * starts WIRE-side discard immediately.  Therefore an EOL already queued
 * before damage can never resynchronize the stream.  Each accepted entry is
 * stamped with both this generation and its ISR arrival tick. */
typedef enum
{
    JETSON_RX_DAMAGE_RING_OVERFLOW = 0,
    JETSON_RX_DAMAGE_UART_ERROR,
    JETSON_RX_DAMAGE_DMA_OVERWRITE,
    JETSON_RX_DAMAGE_DMA_INVARIANT,
    JETSON_RX_DAMAGE_INVALID_CHAR,
    JETSON_RX_DAMAGE_LINE_OVERFLOW,
    JETSON_RX_DAMAGE_ASSEMBLY_TIMEOUT,
    JETSON_RX_DAMAGE_REARM_FAILURE
} JetsonRxDamageReason_t;

typedef enum
{
    RX_LINK_HEALTHY = 0,
    RX_LINK_OUTAGE_PENDING,
    RX_LINK_OUTAGE_FAULTED
} JetsonRxLinkState_t;

typedef enum
{
    JETSON_RX_DMA_SNAPSHOT_OK = 0,
    JETSON_RX_DMA_SNAPSHOT_RETRY,
    JETSON_RX_DMA_SNAPSHOT_INVARIANT,
    JETSON_RX_DMA_SNAPSHOT_OVERWRITE
} JetsonRxDmaSnapshotResult_t;

static volatile uint32_t s_rx_damage_generation;
static volatile bool s_rx_wire_discard_until_eol;
/* If the 32-bit generation ever saturates, remain permanently fail-closed.
 * Reusing an old generation after wrap could otherwise validate damaged
 * data.  This state is reachable only after UINT32_MAX corruption events. */
static volatile bool s_rx_generation_exhausted;
static volatile uint32_t s_rx_line_overflow_count;
static volatile uint32_t s_rx_uart_error_count;
static volatile uint32_t s_rx_invalid_char_count;
static volatile uint32_t s_rx_assembly_timeout_count;
static volatile uint32_t s_rx_stale_line_count;
static volatile uint32_t s_rx_desync_count;
static volatile uint32_t s_rx_byte_budget_hit_count;
static volatile uint32_t s_rx_command_budget_hit_count;
static uint16_t s_rx_debug_snapshot_sequence;

/* Persistent circular-DMA state and bounded main-context restart recovery.
 * "armed/rearm" names remain for the protocol-v4 UART_RX_DEBUG wire grammar:
 * armed means DMA active; rearm means DMA restart. */
static volatile bool s_rx_armed;
static volatile bool s_rx_rearm_pending;
static volatile bool s_rx_arm_attempt_in_progress;
static volatile uint32_t s_rx_arm_attempt_sequence;
/* Natural wrap is intentional: this is the DMA RxEvent sequence exposed by
 * the backward-compatible completion_seq diagnostic field. */
static volatile uint32_t s_rx_completion_sequence;
static volatile uint32_t s_rx_rearm_failure_count;
static volatile uint32_t s_rx_rearm_recovery_count;
static volatile uint32_t s_rx_rearm_fault_event_count;
/* Every RX stream-damage event publishes one main-context Drive + Lift stop
 * request. Multiple events may coalesce behind the pending bit, while the
 * event and dispatch counters retain diagnostic observability. */
static volatile bool s_rx_immediate_safety_stop_pending;
static volatile uint32_t s_rx_immediate_safety_stop_event_count;
static volatile uint32_t s_rx_immediate_safety_stop_dispatch_count;
static volatile uint32_t s_rx_rearm_consecutive_failures;
static volatile JetsonRxLinkState_t s_rx_link_state = RX_LINK_HEALTHY;
static volatile uint32_t s_rx_outage_start_tick;
static volatile HAL_StatusTypeDef s_rx_last_rearm_hal_status = HAL_OK;
static volatile bool s_rx_dma_callbacks_enabled;
static volatile bool s_rx_dma_restart_reset_required;
static volatile bool s_rx_dma_phase_mismatch_pending;
static volatile uint32_t s_rx_dma_session_epoch;
static volatile uint32_t s_rx_dma_ht_epoch;
static volatile uint32_t s_rx_dma_tc_epoch;
static volatile uint32_t s_rx_dma_produced_total;
static volatile uint32_t s_rx_dma_consumed_total;
static volatile uint16_t s_rx_dma_last_event_position;
static volatile uint32_t s_rx_dma_backlog_arrival_tick;
static volatile uint32_t s_rx_dma_empty_lower_bound_tick;
static volatile uint32_t s_rx_dma_last_event_earliest_tick;
static volatile bool s_rx_dma_backlog_tick_valid;
static volatile uint32_t s_rx_dma_backlog_high_water;
static volatile uint32_t s_rx_dma_idle_event_count;
static volatile uint32_t s_rx_dma_ht_event_count;
static volatile uint32_t s_rx_dma_tc_event_count;
static volatile uint32_t s_rx_dma_duplicate_event_count;
static volatile uint32_t s_rx_dma_overwrite_count;
static volatile uint32_t s_rx_dma_stale_callback_count;
static volatile uint32_t s_rx_last_uart_error;
#if ENABLE_JETSON_SELF_LOOPBACK_TEST
static volatile bool s_loopback_print_pending = false;
static volatile uint8_t s_loopback_print_byte = 0U;
#endif
static char s_jetson_line[UART_COMMAND_LINE_MAX];
static uint32_t s_jetson_len = 0U;
static uint32_t s_rx_line_generation;
static uint32_t s_rx_first_arrival_tick;
static uint32_t s_rx_last_arrival_tick;
#ifdef AMR_HOST_TEST
static uint32_t s_test_rx_error_after_pops;
static bool s_test_rx_dma_hardware_total_override_valid;
static uint32_t s_test_rx_dma_hardware_total_override;
static bool s_test_rx_dma_advance_after_copy_pending;
static uint32_t s_test_rx_dma_total_after_copy;
#endif
static void (*s_raw_command_callback)(int32_t left_rpm,
                                      int32_t right_rpm,
                                      uint32_t duration_ms);
static void (*s_uart2_rx_byte_callback)(void);
static void (*s_uart2_error_callback)(void);

void JetsonProtocol_SetRawCommandCallback(void (*fn)(int32_t left_rpm,
                                                    int32_t right_rpm,
                                                    uint32_t duration_ms))
{
    s_raw_command_callback = fn;
}

void JetsonProtocol_SetUart2RxByteCallback(void (*fn)(void))
{
    s_uart2_rx_byte_callback = fn;
}

void JetsonProtocol_SetUart2ErrorCallback(void (*fn)(void))
{
    s_uart2_error_callback = fn;
}

static uint32_t JetsonRx_EnterCritical(void)
{
#ifdef AMR_HOST_TEST
    return 0U;
#else
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#endif
}

static void JetsonRx_ExitCritical(uint32_t primask)
{
#ifdef AMR_HOST_TEST
    (void)primask;
#else
    if ((primask & 1U) == 0U) {
        __enable_irq();
    }
#endif
}

static void JetsonRx_Increment(volatile uint32_t *counter)
{
    if (*counter != UINT32_MAX) {
        (*counter)++;
    }
}

static void JetsonRx_ResetPartialAssemblyForGeneration(uint32_t generation)
{
    s_jetson_len = 0U;
    s_rx_line_generation = generation;
    s_rx_first_arrival_tick = 0U;
    s_rx_last_arrival_tick = 0U;
}

static void JetsonRx_ResetAssembly(void)
{
    JetsonRx_ResetPartialAssemblyForGeneration(0U);
}

/* Caller holds the short RX critical section.  No parsing, formatting, HAL
 * call, or motor operation is performed with interrupts disabled. */
static void JetsonRx_MarkDamageLocked(JetsonRxDamageReason_t reason)
{
    if (s_rx_damage_generation != UINT32_MAX) {
        s_rx_damage_generation++;
    } else {
        s_rx_generation_exhausted = true;
    }
    s_jetson_rx_ring.tail = s_jetson_rx_ring.head;
    /* DMA bytes already observed but not copied into the software ring belong
     * to the damaged stream too. Drop that entire published backlog. Bytes
     * produced after this point remain visible and are consumed only by the
     * post-damage discard-until-EOL path. */
    s_rx_dma_consumed_total = s_rx_dma_produced_total;
    s_rx_dma_backlog_tick_valid = false;
    s_rx_wire_discard_until_eol = true;
    JetsonRx_Increment(&s_rx_desync_count);
    s_rx_immediate_safety_stop_pending = true;
    JetsonRx_Increment(&s_rx_immediate_safety_stop_event_count);

    switch (reason) {
    case JETSON_RX_DAMAGE_RING_OVERFLOW:
        JetsonRx_Increment(&s_jetson_rx_overflow_count);
        break;
    case JETSON_RX_DAMAGE_UART_ERROR:
        JetsonRx_Increment(&s_rx_uart_error_count);
        break;
    case JETSON_RX_DAMAGE_DMA_OVERWRITE:
        /* Keep the five-line protocol-v4 grammar unchanged: ring_ovf is the
         * aggregate "RX storage was overrun" counter for either DMA storage
         * or the established software ring. */
        JetsonRx_Increment(&s_jetson_rx_overflow_count);
        JetsonRx_Increment(&s_rx_dma_overwrite_count);
        break;
    case JETSON_RX_DAMAGE_DMA_INVARIANT:
        /* An impossible DMA event phase proves that producer progress is no
         * longer knowable. Surface it through the established UART damage
         * field without adding a wire-incompatible sixth debug line. */
        JetsonRx_Increment(&s_rx_uart_error_count);
        break;
    case JETSON_RX_DAMAGE_INVALID_CHAR:
        JetsonRx_Increment(&s_rx_invalid_char_count);
        break;
    case JETSON_RX_DAMAGE_LINE_OVERFLOW:
        JetsonRx_Increment(&s_rx_line_overflow_count);
        break;
    case JETSON_RX_DAMAGE_ASSEMBLY_TIMEOUT:
        JetsonRx_Increment(&s_rx_assembly_timeout_count);
        break;
    case JETSON_RX_DAMAGE_REARM_FAILURE:
        /* rearm_fail already counts every failed HAL arm attempt.  This
         * reason records the episode's stream discontinuity through the
         * shared damage generation/desync counters without adding another
         * wire-visible diagnostic field. */
        break;
    default:
        break;
    }
}

static void JetsonRx_MarkDamage(JetsonRxDamageReason_t reason)
{
    uint32_t primask = JetsonRx_EnterCritical();
    JetsonRx_MarkDamageLocked(reason);
    JetsonRx_ExitCritical(primask);
}

/* Caller holds the RX critical section. */
static void JetsonRx_HandleWireByteLocked(uint8_t byte,
                                          uint32_t arrival_tick)
{
    JetsonRxRing_t *ring = &s_jetson_rx_ring;
    uint16_t next;

    if (s_rx_wire_discard_until_eol) {
        /* Only a delimiter observed on the WIRE after the damage event may
         * clear discard.  It is consumed here and never enters the ring. */
        if (((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n')) &&
            !s_rx_generation_exhausted) {
            s_rx_wire_discard_until_eol = false;
        }
        return;
    }

    next = (uint16_t)((ring->head + 1U) % UART_RX_RING_SIZE);

    if (next == ring->tail) {
        /* Drop the triggering byte, flush all old entries, and discard on
         * the wire.  Never advance tail and continue parsing a gapped line. */
        JetsonRx_MarkDamageLocked(JETSON_RX_DAMAGE_RING_OVERFLOW);
        return;
    }

    ring->buf[ring->head].byte = byte;
    ring->buf[ring->head].arrival_tick = arrival_tick;
    ring->buf[ring->head].stream_generation = s_rx_damage_generation;
    ring->head = next;
}

static void JetsonRx_HandleWireByte(uint8_t byte, uint32_t arrival_tick)
{
    uint32_t primask = JetsonRx_EnterCritical();

    JetsonRx_HandleWireByteLocked(byte, arrival_tick);
    JetsonRx_ExitCritical(primask);
}

/* A DMA chunk is copied before it is committed. If damage/restart changes
 * either token before an individual byte is published, that pre-damage byte
 * is discarded and, critically, cannot clear discard-until-EOL. */
static bool JetsonRx_HandleDmaWireByteIfCurrent(uint8_t byte,
                                                uint32_t arrival_tick,
                                                uint32_t stream_generation,
                                                uint32_t session_epoch)
{
    uint32_t primask = JetsonRx_EnterCritical();
    bool current = s_rx_dma_callbacks_enabled &&
                   (s_rx_dma_session_epoch == session_epoch) &&
                   (s_rx_damage_generation == stream_generation);

    if (current) {
        /* Keep the established wire-byte entry point in the production DMA
         * path. Its nested critical section observes PRIMASK=1 and therefore
         * cannot reopen the outer token-check section between validation and
         * publication. */
        JetsonRx_HandleWireByte(byte, arrival_tick);
    }
    JetsonRx_ExitCritical(primask);
    return current;
}

static bool JetsonRxRing_Pop(JetsonRxRing_t *ring, JetsonRxEntry_t *entry)
{
    uint32_t primask = JetsonRx_EnterCritical();

    if (ring->tail == ring->head) {
        JetsonRx_ExitCritical(primask);
        return false;
    }

    *entry = ring->buf[ring->tail];
    ring->tail = (uint16_t)((ring->tail + 1U) % UART_RX_RING_SIZE);
    JetsonRx_ExitCritical(primask);
    return true;
}

static uint32_t JetsonRx_QueuedCount(void)
{
    uint32_t primask = JetsonRx_EnterCritical();
    uint16_t head = s_jetson_rx_ring.head;
    uint16_t tail = s_jetson_rx_ring.tail;
    uint32_t count = (head >= tail) ?
        (uint32_t)(head - tail) :
        (uint32_t)(UART_RX_RING_SIZE - tail + head);
    JetsonRx_ExitCritical(primask);
    return count;
}

/* Caller holds the RX critical section. One slot remains empty to distinguish
 * full from empty, matching JetsonRx_HandleWireByteLocked(). */
static uint32_t JetsonRx_RingFreeLocked(void)
{
    uint16_t head = s_jetson_rx_ring.head;
    uint16_t tail = s_jetson_rx_ring.tail;
    uint32_t queued = (head >= tail) ?
        (uint32_t)(head - tail) :
        (uint32_t)(UART_RX_RING_SIZE - tail + head);

    return (UART_RX_RING_SIZE - 1U) - queued;
}

static bool JetsonRx_LineGenerationIsValid(uint32_t generation)
{
    uint32_t primask = JetsonRx_EnterCritical();
    bool valid = !s_rx_wire_discard_until_eol &&
                 !s_rx_generation_exhausted &&
                 (s_rx_damage_generation == generation);
    JetsonRx_ExitCritical(primask);
    return valid;
}

static const char *JetsonRx_LinkStateName(JetsonRxLinkState_t state)
{
    switch (state) {
    case RX_LINK_HEALTHY:
        return "HEALTHY";
    case RX_LINK_OUTAGE_PENDING:
        return "OUTAGE_PENDING";
    case RX_LINK_OUTAGE_FAULTED:
        return "OUTAGE_FAULTED";
    default:
        return "UNKNOWN";
    }
}

static uint32_t JetsonRx_DmaWireDurationMs(uint32_t byte_count)
{
    uint32_t numerator =
        (byte_count * JETSON_RX_UART_BITS_PER_BYTE * 1000U) +
        (JETSON_RX_UART_BAUD - 1U);

    return numerator / JETSON_RX_UART_BAUD;
}

/* Caller holds the RX critical section. This is the one transport-outage
 * transition used by UART/DMA errors, overwrite and impossible DMA phases. */
static void JetsonRx_BeginTransportOutageLocked(
    JetsonRxDamageReason_t reason,
    HAL_StatusTypeDef status,
    uint32_t now)
{
    bool new_episode = s_rx_link_state == RX_LINK_HEALTHY;

    s_rx_armed = false;
    s_rx_rearm_pending = true;
    s_rx_dma_callbacks_enabled = false;
    s_rx_dma_restart_reset_required = true;
    s_rx_dma_phase_mismatch_pending = false;
    s_rx_dma_consumed_total = s_rx_dma_produced_total;
    s_rx_dma_backlog_tick_valid = false;
    s_rx_last_rearm_hal_status = status;

    if (new_episode) {
        JetsonRx_MarkDamageLocked(reason);
        s_rx_link_state = RX_LINK_OUTAGE_PENDING;
        s_rx_outage_start_tick = now;
        s_rx_rearm_consecutive_failures = 0U;
    }
}

/* Caller holds the RX critical section. A failed start creates one damage
 * episode; later cooperative failures in that same outage only update retry
 * diagnostics and never schedule duplicate immediate stops. */
static void JetsonRx_RecordArmFailureLocked(HAL_StatusTypeDef status,
                                            uint32_t now)
{
    if (s_rx_link_state == RX_LINK_HEALTHY) {
        JetsonRx_BeginTransportOutageLocked(
            JETSON_RX_DAMAGE_REARM_FAILURE, status, now);
    } else {
        s_rx_armed = false;
        s_rx_rearm_pending = true;
        s_rx_dma_callbacks_enabled = false;
        s_rx_dma_restart_reset_required = true;
        s_rx_last_rearm_hal_status = status;
    }
    JetsonRx_Increment(&s_rx_rearm_failure_count);
    JetsonRx_Increment(&s_rx_rearm_consecutive_failures);
}

/* Caller holds the RX critical section. */
static void JetsonRx_RecordStableArmLocked(void)
{
    bool recovered = s_rx_link_state != RX_LINK_HEALTHY;

    s_rx_armed = true;
    s_rx_rearm_pending = false;
    s_rx_dma_callbacks_enabled = true;
    s_rx_dma_restart_reset_required = false;
    s_rx_rearm_consecutive_failures = 0U;
    s_rx_outage_start_tick = 0U;
    s_rx_last_rearm_hal_status = HAL_OK;
    s_rx_link_state = RX_LINK_HEALTHY;
    if (recovered) {
        JetsonRx_Increment(&s_rx_rearm_recovery_count);
    }
}

/* Caller holds the RX critical section. A new epoch gives callbacks and main
 * an explicit session token. On hardware, the old DMA IRQ is also disabled,
 * aborted and cleared before this token is published. */
static uint32_t JetsonRx_ResetDmaSessionLocked(uint32_t now)
{
    s_rx_dma_session_epoch++;
    s_rx_dma_ht_epoch = 0U;
    s_rx_dma_tc_epoch = 0U;
    s_rx_dma_produced_total = 0U;
    s_rx_dma_consumed_total = 0U;
    s_rx_dma_last_event_position = 0U;
    s_rx_dma_backlog_arrival_tick = now;
    s_rx_dma_empty_lower_bound_tick = now;
    s_rx_dma_last_event_earliest_tick = now;
    s_rx_dma_backlog_tick_valid = false;
    s_rx_dma_phase_mismatch_pending = false;
    s_rx_dma_callbacks_enabled = true;
#ifdef AMR_HOST_TEST
    s_test_rx_dma_hardware_total_override_valid = false;
    s_test_rx_dma_advance_after_copy_pending = false;
#endif
    /* Provisional during HAL start. A synchronous UART error callback sets it
     * back to true, preventing the outer HAL_OK from publishing armed=1. */
    s_rx_rearm_pending = false;
    return s_rx_dma_session_epoch;
}

static HAL_StatusTypeDef JetsonRx_TryArm(void)
{
    HAL_StatusTypeDef status;
    HAL_StatusTypeDef abort_status = HAL_OK;
    uint32_t attempt_epoch;
    uint32_t primask = JetsonRx_EnterCritical();
    bool reset_required;

    /* Exactly one bounded attempt per call. Recovery is never performed from
     * an ISR and never loops, delays or waits for a later state transition. */
    if (s_rx_immediate_safety_stop_pending) {
        s_rx_rearm_pending = true;
        status = s_rx_last_rearm_hal_status;
        JetsonRx_ExitCritical(primask);
        return status;
    }
    if (s_rx_armed) {
        status = s_rx_last_rearm_hal_status;
        JetsonRx_ExitCritical(primask);
        return status;
    }
    if (s_rx_arm_attempt_in_progress) {
        s_rx_rearm_pending = true;
        status = s_rx_last_rearm_hal_status;
        JetsonRx_ExitCritical(primask);
        return status;
    }

    s_rx_arm_attempt_in_progress = true;
    s_rx_armed = false;
    s_rx_dma_callbacks_enabled = false;
    reset_required = s_rx_dma_restart_reset_required;
    JetsonRx_Increment(&s_rx_arm_attempt_sequence);
    JetsonRx_ExitCritical(primask);

#ifndef AMR_HOST_TEST
    if (reset_required) {
        /* Only the RX DMA IRQ is gated. USART3 IRQ stays enabled so the
         * independent interrupt-driven TX manager is never paused. */
        HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
    }
#endif

    if (reset_required) {
        /* Local HAL_UART_AbortReceive() is a bounded blocking register/DMA
         * stop with a finite HAL DMA timeout. It affects RX only, preserving
         * an in-flight USART3 interrupt-driven TX frame. */
        abort_status = HAL_UART_AbortReceive(&huart3);
#ifndef AMR_HOST_TEST
        __HAL_UART_CLEAR_FLAG(
            &huart3,
            UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF |
            UART_CLEAR_FEF | UART_CLEAR_IDLEF);
        __HAL_UART_SEND_REQ(&huart3, UART_RXDATA_FLUSH_REQUEST);
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
        /* Abort disabled the old RX sources and the flags above removed their
         * causes. Clear a latched USART vector as the equivalent old-session
         * callback barrier. Any independent TX source remains asserted and
         * will pend USART3 again, so interrupt-driven TX is not stopped. */
        HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
#endif
        if (abort_status != HAL_OK) {
#ifndef AMR_HOST_TEST
            HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
#endif
            primask = JetsonRx_EnterCritical();
            s_rx_arm_attempt_in_progress = false;
            JetsonRx_RecordArmFailureLocked(abort_status, HAL_GetTick());
            JetsonRx_ExitCritical(primask);
            return abort_status;
        }
    }

    {
        uint32_t start_tick = HAL_GetTick();

        primask = JetsonRx_EnterCritical();
        attempt_epoch = JetsonRx_ResetDmaSessionLocked(start_tick);
    }
    JetsonRx_ExitCritical(primask);

    if ((huart3.hdmarx != &hdma_usart3_rx) ||
        (hdma_usart3_rx.Init.Mode != DMA_CIRCULAR)) {
        status = HAL_ERROR;
    } else {
        status = HAL_UARTEx_ReceiveToIdle_DMA(
            &huart3,
            s_jetson_rx_dma_buffer,
            (uint16_t)JETSON_RX_DMA_BUFFER_SIZE);
    }

#ifndef AMR_HOST_TEST
    if (reset_required) {
        /* HAL start cannot receive half a 4096-byte buffer during these few
         * instructions. Clearing a leftover old-session pending bit here is
         * therefore safe and completes the stale-callback barrier. */
        HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
        HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    }
#endif

    primask = JetsonRx_EnterCritical();
    s_rx_arm_attempt_in_progress = false;
    if ((status == HAL_OK) &&
        s_rx_dma_callbacks_enabled &&
        !s_rx_rearm_pending &&
        (s_rx_dma_session_epoch == attempt_epoch)) {
        JetsonRx_RecordStableArmLocked();
    } else if (status != HAL_OK) {
        s_rx_dma_callbacks_enabled = false;
        JetsonRx_RecordArmFailureLocked(status, HAL_GetTick());
    }
    /* If HAL returned OK but a synchronous error callback invalidated the
     * provisional session, that callback already recorded the outage once. */
    JetsonRx_ExitCritical(primask);
    return status;
}

void JetsonProtocol_StartUartRxInterrupts(void)
{
    HAL_StatusTypeDef rxst3 = JetsonRx_TryArm();
    uart_printf("USART3 RX DMA start status=%d\r\n", rxst3);
}

void JetsonProtocol_DispatchPendingRxSafetyStop(void)
{
    bool dispatch_stop;
    uint32_t primask = JetsonRx_EnterCritical();

    /* Atomic take-and-clear: damage arriving after this point remains pending
     * for the next main-loop dispatch and cannot be lost. */
    dispatch_stop = s_rx_immediate_safety_stop_pending;
    if (dispatch_stop) {
        s_rx_immediate_safety_stop_pending = false;
    }
    JetsonRx_ExitCritical(primask);

    if (!dispatch_stop) {
        return;
    }

    /* Main context only; never call Drive, Lift, HAL, printing or formatting
     * while the RX critical section is held. */
    Drive_CommandStop();
    Lift_RequestCommunicationStop();

    primask = JetsonRx_EnterCritical();
    JetsonRx_Increment(&s_rx_immediate_safety_stop_dispatch_count);
    JetsonRx_ExitCritical(primask);
}

/* O(1) circular-DMA producer accounting. The live HAL Size is a position,
 * never a delta. HT and TC epochs disambiguate wrap and delayed callbacks:
 *
 *   absolute producer = (TC epoch * 4096) + normalized position
 *   expected HT epoch = TC epoch + (position in second half ? 1 : 0)
 *
 * If HT and TC flags were pending together, HAL invokes HT first and both
 * callbacks may see the same post-wrap position. The first callback leaves a
 * transient phase mismatch; TC immediately repairs it and publishes the full
 * monotonic delta. Main treats a mismatch that survives IRQ completion as
 * transport damage. */
static void JetsonRx_RecordDmaEvent(uint16_t size,
                                    HAL_UART_RxEventTypeTypeDef event_type,
                                    uint32_t event_tick,
                                    uint32_t callback_epoch)
{
    uint32_t primask = JetsonRx_EnterCritical();
    uint32_t position;
    uint32_t expected_ht_epoch;
    uint32_t candidate_total;
    uint32_t delta;
    uint32_t backlog_before;
    uint32_t backlog_after;
    uint32_t span_age;
    uint32_t span_earliest_tick;

    s_rx_completion_sequence++;
    if (!s_rx_dma_callbacks_enabled ||
        (callback_epoch != s_rx_dma_session_epoch)) {
        JetsonRx_Increment(&s_rx_dma_stale_callback_count);
        JetsonRx_ExitCritical(primask);
        return;
    }

    if ((size > JETSON_RX_DMA_BUFFER_SIZE) ||
        ((event_type != HAL_UART_RXEVENT_IDLE) &&
         (event_type != HAL_UART_RXEVENT_HT) &&
         (event_type != HAL_UART_RXEVENT_TC)) ||
        ((event_type == HAL_UART_RXEVENT_IDLE) && (size == 0U))) {
        JetsonRx_BeginTransportOutageLocked(
            JETSON_RX_DAMAGE_DMA_INVARIANT, HAL_ERROR, event_tick);
        JetsonRx_ExitCritical(primask);
        return;
    }

    if (event_type == HAL_UART_RXEVENT_HT) {
        s_rx_dma_ht_epoch++;
        JetsonRx_Increment(&s_rx_dma_ht_event_count);
    } else if (event_type == HAL_UART_RXEVENT_TC) {
        s_rx_dma_tc_epoch++;
        JetsonRx_Increment(&s_rx_dma_tc_event_count);
    } else {
        JetsonRx_Increment(&s_rx_dma_idle_event_count);
    }

    position = (size == JETSON_RX_DMA_BUFFER_SIZE) ? 0U :
        (uint32_t)size;
    s_rx_dma_last_event_position = size;
    expected_ht_epoch = s_rx_dma_tc_epoch +
        ((position >= JETSON_RX_DMA_HALF_SIZE) ? 1U : 0U);
    if (s_rx_dma_ht_epoch != expected_ht_epoch) {
        s_rx_dma_phase_mismatch_pending = true;
        JetsonRx_ExitCritical(primask);
        return;
    }
    s_rx_dma_phase_mismatch_pending = false;

    /* JETSON_RX_DMA_BUFFER_SIZE is 2^12. Unsigned wrap of the 32-bit byte
     * sequence is intentional and backlog subtraction remains valid. */
    candidate_total =
        (s_rx_dma_tc_epoch << 12U) + position;
    delta = (uint32_t)(candidate_total - s_rx_dma_produced_total);
    if (delta == 0U) {
        JetsonRx_Increment(&s_rx_dma_duplicate_event_count);
        JetsonRx_ExitCritical(primask);
        return;
    }
    if (delta > JETSON_RX_DMA_BUFFER_SIZE) {
        JetsonRx_BeginTransportOutageLocked(
            JETSON_RX_DAMAGE_DMA_INVARIANT, HAL_ERROR, event_tick);
        JetsonRx_ExitCritical(primask);
        return;
    }

    backlog_before =
        (uint32_t)(s_rx_dma_produced_total - s_rx_dma_consumed_total);
    span_age = JetsonRx_DmaWireDurationMs(delta);
    span_earliest_tick = event_tick - span_age;
    s_rx_dma_last_event_earliest_tick = span_earliest_tick;
    if (backlog_before == 0U) {
        uint32_t empty_age =
            (uint32_t)(event_tick -
                       s_rx_dma_empty_lower_bound_tick);

        /* The event-minus-wire-duration estimate satisfies the requested DMA
         * span rule. A prior reliable empty observation is an even stronger
         * empty-boundary lower bound when IRQ service was delayed, so retain
         * whichever is
         * older. This can only reject a command as stale, never refresh it. */
        s_rx_dma_backlog_arrival_tick =
            (empty_age >= span_age) ?
            s_rx_dma_empty_lower_bound_tick :
            span_earliest_tick;
        s_rx_dma_backlog_tick_valid = true;
    }
    s_rx_dma_produced_total = candidate_total;
    backlog_after =
        (uint32_t)(s_rx_dma_produced_total - s_rx_dma_consumed_total);
    if (backlog_after > s_rx_dma_backlog_high_water) {
        s_rx_dma_backlog_high_water = backlog_after;
    }

    /* At exactly N the producer aliases the consumer. The bytes may still
     * exist at that instant, but the next DMA beat can overwrite the byte
     * main is reading. Fail one byte early rather than accept that race. */
    if (backlog_after >= JETSON_RX_DMA_BUFFER_SIZE) {
        JetsonRx_BeginTransportOutageLocked(
            JETSON_RX_DAMAGE_DMA_OVERWRITE, HAL_ERROR, event_tick);
    }
    JetsonRx_ExitCritical(primask);
}

static void JetsonRx_ResolvePendingDmaPhase(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t primask = JetsonRx_EnterCritical();

    if (s_rx_dma_phase_mismatch_pending &&
        s_rx_dma_callbacks_enabled && s_rx_armed) {
        JetsonRx_BeginTransportOutageLocked(
            JETSON_RX_DAMAGE_DMA_INVARIANT, HAL_ERROR, now);
    }
    JetsonRx_ExitCritical(primask);
}

/* Caller holds the RX critical section. On target, combine the live NDTR
 * position with HT/TC flags that hardware has raised but whose same-priority
 * IRQ callbacks have not yet run. Reading flags on both sides of NDTR makes
 * a boundary crossing detectable; two bounded attempts are enough for this
 * main-context safety check and never spin. The result is used to prove that
 * a copied source span was not overwritten while main was reading it.
 *
 * Host tests publish producer progress explicitly through the real event
 * accounting hook, so their hardware snapshot is the published total. */
static JetsonRxDmaSnapshotResult_t
JetsonRx_CaptureHardwareProducerLocked(uint32_t *producer_total)
{
    if (producer_total == NULL) {
        return JETSON_RX_DMA_SNAPSHOT_INVARIANT;
    }

#ifdef AMR_HOST_TEST
    *producer_total = s_test_rx_dma_hardware_total_override_valid ?
        s_test_rx_dma_hardware_total_override :
        s_rx_dma_produced_total;
    return JETSON_RX_DMA_SNAPSHOT_OK;
#else
    uint32_t attempt;

    if ((huart3.hdmarx != &hdma_usart3_rx) ||
        (hdma_usart3_rx.Instance != DMA1_Stream0) ||
        (hdma_usart3_rx.Init.Mode != DMA_CIRCULAR)) {
        return JETSON_RX_DMA_SNAPSHOT_INVARIANT;
    }

    for (attempt = 0U; attempt < 2U; attempt++) {
        uint32_t flags_before = 0U;
        uint32_t flags_after = 0U;
        uint32_t remaining;
        uint32_t position;
        uint32_t effective_ht_epoch;
        uint32_t effective_tc_epoch;
        uint32_t expected_ht_epoch;
        uint32_t candidate_total;
        uint32_t unpublished;

        if (__HAL_DMA_GET_FLAG(
                &hdma_usart3_rx,
                __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_usart3_rx)) != 0U) {
            flags_before |= 1U;
        }
        if (__HAL_DMA_GET_FLAG(
                &hdma_usart3_rx,
                __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart3_rx)) != 0U) {
            flags_before |= 2U;
        }
        __DMB();
        remaining = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
        __DMB();
        if (__HAL_DMA_GET_FLAG(
                &hdma_usart3_rx,
                __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_usart3_rx)) != 0U) {
            flags_after |= 1U;
        }
        if (__HAL_DMA_GET_FLAG(
                &hdma_usart3_rx,
                __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart3_rx)) != 0U) {
            flags_after |= 2U;
        }

        if (flags_before != flags_after) {
            continue;
        }
        if (remaining > JETSON_RX_DMA_BUFFER_SIZE) {
            return JETSON_RX_DMA_SNAPSHOT_INVARIANT;
        }

        position =
            (JETSON_RX_DMA_BUFFER_SIZE - remaining) &
            JETSON_RX_DMA_BUFFER_MASK;
        effective_ht_epoch = s_rx_dma_ht_epoch +
            (((flags_after & 1U) != 0U) ? 1U : 0U);
        effective_tc_epoch = s_rx_dma_tc_epoch +
            (((flags_after & 2U) != 0U) ? 1U : 0U);
        expected_ht_epoch = effective_tc_epoch +
            ((position >= JETSON_RX_DMA_HALF_SIZE) ? 1U : 0U);
        if (effective_ht_epoch != expected_ht_epoch) {
            return JETSON_RX_DMA_SNAPSHOT_INVARIANT;
        }

        candidate_total = (effective_tc_epoch << 12U) + position;
        unpublished =
            (uint32_t)(candidate_total - s_rx_dma_produced_total);
        if (unpublished > JETSON_RX_DMA_BUFFER_SIZE) {
            return JETSON_RX_DMA_SNAPSHOT_OVERWRITE;
        }
        *producer_total = candidate_total;
        return JETSON_RX_DMA_SNAPSHOT_OK;
    }

    /* DMA crossed a half/full boundary during both snapshots. Defer this
     * iteration; the pending IRQ will publish a stable position immediately
     * after the short critical section ends. Instability alone is not proof
     * of stream loss. */
    return JETSON_RX_DMA_SNAPSHOT_RETRY;
#endif
}

static void JetsonRx_DrainDma(void)
{
    uint32_t drained = 0U;

    while (drained < JETSON_RX_DMA_DRAIN_BUDGET) {
        uint32_t now = HAL_GetTick();
        uint32_t primask = JetsonRx_EnterCritical();
        uint32_t produced;
        uint32_t hardware_produced;
        uint32_t consumer_start;
        uint32_t available;
        uint32_t hardware_available;
        uint32_t ring_free;
        uint32_t batch;
        uint32_t session_epoch;
        uint32_t stream_generation;
        uint32_t arrival_tick;
        uint32_t index;
        bool discard_mode;
        JetsonRxDmaSnapshotResult_t snapshot_result;

        if (!s_rx_armed || !s_rx_dma_callbacks_enabled ||
            s_rx_rearm_pending || s_rx_dma_phase_mismatch_pending) {
            JetsonRx_ExitCritical(primask);
            return;
        }

        produced = s_rx_dma_produced_total;
        consumer_start = s_rx_dma_consumed_total;
        snapshot_result =
            JetsonRx_CaptureHardwareProducerLocked(&hardware_produced);
        if (snapshot_result == JETSON_RX_DMA_SNAPSHOT_RETRY) {
            JetsonRx_ExitCritical(primask);
            return;
        }
        if (snapshot_result == JETSON_RX_DMA_SNAPSHOT_INVARIANT) {
            JetsonRx_BeginTransportOutageLocked(
                JETSON_RX_DAMAGE_DMA_INVARIANT,
                HAL_ERROR,
                now);
            JetsonRx_ExitCritical(primask);
            return;
        }
        hardware_available =
            (uint32_t)(hardware_produced - consumer_start);
        if ((snapshot_result == JETSON_RX_DMA_SNAPSHOT_OVERWRITE) ||
            (hardware_available >= JETSON_RX_DMA_BUFFER_SIZE)) {
            JetsonRx_BeginTransportOutageLocked(
                JETSON_RX_DAMAGE_DMA_OVERWRITE,
                HAL_ERROR,
                now);
            JetsonRx_ExitCritical(primask);
            return;
        }
        available = (uint32_t)(produced - consumer_start);
        if (available == 0U) {
            if (hardware_produced == consumer_start) {
                /* HAL_GetTick() was sampled before this live DMA-empty proof,
                 * so it cannot be newer than any byte received afterwards. */
                s_rx_dma_empty_lower_bound_tick = now;
                s_rx_dma_backlog_tick_valid = false;
            }
            JetsonRx_ExitCritical(primask);
            return;
        }
        if (available >= JETSON_RX_DMA_BUFFER_SIZE) {
            JetsonRx_BeginTransportOutageLocked(
                JETSON_RX_DAMAGE_DMA_OVERWRITE,
                HAL_ERROR,
                now);
            JetsonRx_ExitCritical(primask);
            return;
        }

        discard_mode = s_rx_wire_discard_until_eol;
        ring_free = JetsonRx_RingFreeLocked();
        if (!discard_mode && (ring_free == 0U)) {
            /* Software parser backpressure is not corruption. Leave the DMA
             * backlog in place; a later loop drains it after PollLine frees
             * software-ring slots. The DMA overwrite guard remains active. */
            JetsonRx_ExitCritical(primask);
            return;
        }

        batch = available;
        if (batch > (JETSON_RX_DMA_DRAIN_BUDGET - drained)) {
            batch = JETSON_RX_DMA_DRAIN_BUDGET - drained;
        }
        if (batch > JETSON_RX_DMA_COPY_CHUNK) {
            batch = JETSON_RX_DMA_COPY_CHUNK;
        }
        if (!discard_mode && (batch > ring_free)) {
            batch = ring_free;
        }
        session_epoch = s_rx_dma_session_epoch;
        stream_generation = s_rx_damage_generation;
        arrival_tick = s_rx_dma_backlog_tick_valid ?
            s_rx_dma_backlog_arrival_tick : 0U;
        JetsonRx_ExitCritical(primask);

        for (index = 0U; index < batch; index++) {
            s_jetson_rx_dma_copy[index] =
                s_jetson_rx_dma_buffer[
                    (consumer_start + index) & JETSON_RX_DMA_BUFFER_MASK];
        }
#ifndef AMR_HOST_TEST
        __DMB();
#else
        __asm__ volatile("" ::: "memory");
#endif

#ifdef AMR_HOST_TEST
        /* A one-shot host seam models DMA advancing after the CPU copy but
         * before the commit snapshot. It exercises the production rollback
         * decision without adding target instructions or a reference parser. */
        if (s_test_rx_dma_advance_after_copy_pending) {
            s_test_rx_dma_hardware_total_override =
                s_test_rx_dma_total_after_copy;
            s_test_rx_dma_hardware_total_override_valid = true;
            s_test_rx_dma_advance_after_copy_pending = false;
        }
#endif

        primask = JetsonRx_EnterCritical();
        produced = s_rx_dma_produced_total;
        snapshot_result =
            JetsonRx_CaptureHardwareProducerLocked(&hardware_produced);
        if (snapshot_result == JETSON_RX_DMA_SNAPSHOT_RETRY) {
            JetsonRx_ExitCritical(primask);
            return;
        }
        if (snapshot_result == JETSON_RX_DMA_SNAPSHOT_INVARIANT) {
            JetsonRx_BeginTransportOutageLocked(
                JETSON_RX_DAMAGE_DMA_INVARIANT,
                HAL_ERROR,
                now);
            JetsonRx_ExitCritical(primask);
            return;
        }
        available = (uint32_t)(produced - consumer_start);
        hardware_available =
            (uint32_t)(hardware_produced - consumer_start);
        if (!s_rx_armed || !s_rx_dma_callbacks_enabled ||
            s_rx_rearm_pending ||
            s_rx_dma_phase_mismatch_pending ||
            (s_rx_dma_session_epoch != session_epoch) ||
            (s_rx_damage_generation != stream_generation) ||
            (s_rx_dma_consumed_total != consumer_start) ||
            (available < batch)) {
            JetsonRx_ExitCritical(primask);
            return;
        }
        if ((snapshot_result == JETSON_RX_DMA_SNAPSHOT_OVERWRITE) ||
            (hardware_available >= JETSON_RX_DMA_BUFFER_SIZE) ||
            (available >= JETSON_RX_DMA_BUFFER_SIZE)) {
            JetsonRx_BeginTransportOutageLocked(
                JETSON_RX_DAMAGE_DMA_OVERWRITE,
                HAL_ERROR,
                now);
            JetsonRx_ExitCritical(primask);
            return;
        }

        s_rx_dma_consumed_total = consumer_start + batch;
        if ((s_rx_dma_consumed_total == s_rx_dma_produced_total) &&
            (s_rx_dma_consumed_total == hardware_produced)) {
            /* now precedes the post-copy hardware-empty snapshot, making it
             * a conservative lower bound for the next DMA backlog. */
            s_rx_dma_empty_lower_bound_tick = now;
            s_rx_dma_backlog_tick_valid = false;
        }
        JetsonRx_ExitCritical(primask);

        for (index = 0U; index < batch; index++) {
            if (!JetsonRx_HandleDmaWireByteIfCurrent(
                    s_jetson_rx_dma_copy[index],
                    arrival_tick,
                    stream_generation,
                    session_epoch)) {
                return;
            }
#if ENABLE_JETSON_SELF_LOOPBACK_TEST
            s_loopback_print_byte = s_jetson_rx_dma_copy[index];
            s_loopback_print_pending = true;
#endif
        }
        drained += batch;
    }
}

void JetsonProtocol_ServiceRx(void)
{
    uint32_t now;
    uint32_t outage_start_tick;
    uint32_t outage_age = 0U;
    uint32_t consecutive_failures;
    HAL_StatusTypeDef last_status;
    bool should_retry;
    bool should_fault = false;
    uint32_t primask;

    /* A transient HT-before-TC phase cannot survive return from the pair of
     * same-priority DMA callbacks. Resolve any surviving mismatch before the
     * safety dispatch, outage evaluation or recovery attempt. */
    JetsonRx_ResolvePendingDmaPhase();

    /* Always consume stream-damage safety before outage evaluation or any
     * cooperative DMA restart attempt. */
    JetsonProtocol_DispatchPendingRxSafetyStop();

    /* Evaluate the persistent-outage deadline after the immediate stops.
     * Lift's established stop route may perform bounded CAN work, so refresh
     * the time here to preserve the rule that the deadline is checked before
     * this iteration's retry. */
    now = HAL_GetTick();
    primask = JetsonRx_EnterCritical();

    /* Deadline is evaluated BEFORE this iteration's retry.  Therefore an
     * outage already older than the limit faults even when that retry happens
     * to return HAL_OK. */
    outage_start_tick = s_rx_outage_start_tick;
    if (s_rx_link_state != RX_LINK_HEALTHY) {
        outage_age = (uint32_t)(now - outage_start_tick);
    }
    if ((s_rx_link_state == RX_LINK_OUTAGE_PENDING) &&
        (outage_age > JETSON_RX_REARM_FAULT_MS)) {
        s_rx_link_state = RX_LINK_OUTAGE_FAULTED;
        JetsonRx_Increment(&s_rx_rearm_fault_event_count);
        should_fault = true;
    }
    /* An ISR may have created a new episode after the first critical
     * section.  Never retry that episode until its immediate stop has been
     * consumed by a later ServiceRx call. */
    should_retry = !s_rx_immediate_safety_stop_pending &&
                   s_rx_rearm_pending && !s_rx_armed &&
                   !s_rx_arm_attempt_in_progress;
    consecutive_failures = s_rx_rearm_consecutive_failures;
    last_status = s_rx_last_rearm_hal_status;
    JetsonRx_ExitCritical(primask);

    if (should_fault) {
        /* Main context only: one bounded CRITICAL diagnostic followed by
         * the recoverable Drive zero/IDLE stop. The RX outage remains
         * diagnostic history and never becomes a motion-inhibit latch. */
        (void)JetsonTx_Printf(
            JETSON_TX_PRIORITY_CRITICAL,
            "CRITICAL USART3 RX rearm failed age=%lu consecutive=%lu hal=%d\r\n",
            (unsigned long)outage_age,
            (unsigned long)consecutive_failures,
            (int)last_status);
        Drive_RequestCommunicationFaultStop();
    }
    if (should_retry) {
        /* One bounded abort/start attempt. Failure remains pending for one
         * cooperative retry on a later main-loop iteration. */
        (void)JetsonRx_TryArm();
    }
    JetsonRx_DrainDma();
}

void JetsonProtocol_PrintPendingWarnings(void)
{
    static uint32_t last_overflow_warn_ms = 0U;
    static uint32_t last_overflow_warn_count = 0U;
    static uint32_t last_error_warn_count = 0U;
    uint32_t now = HAL_GetTick();
    uint32_t overflow_count;
    uint32_t error_count;
    uint32_t primask = JetsonRx_EnterCritical();

    overflow_count = s_jetson_rx_overflow_count;
    error_count = s_rx_uart_error_count;
    JetsonRx_ExitCritical(primask);

    if ((overflow_count != last_overflow_warn_count) &&
        ((now - last_overflow_warn_ms) >= 1000U)) {
        last_overflow_warn_ms = now;
        last_overflow_warn_count = overflow_count;
        uart_printf("UART RX overflow on USART3 count=%lu\r\n",
                    (unsigned long)overflow_count);
    }

    if (error_count != last_error_warn_count) {
        last_error_warn_count = error_count;
        uart_printf("UART RX error on USART3 count=%lu\r\n",
                    (unsigned long)error_count);
    }

#if ENABLE_JETSON_SELF_LOOPBACK_TEST
    if (s_loopback_print_pending) {
        uint8_t ch = s_loopback_print_byte;

        s_loopback_print_pending = false;
        if ((ch >= 0x20U) && (ch <= 0x7EU)) {
            uart_printf("USART3 loopback RX: %c\r\n", ch);
        } else {
            uart_printf("USART3 loopback RX: 0x%02X\r\n", ch);
        }
    }
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart->Instance == USART2) &&
        (s_uart2_rx_byte_callback != NULL)) {
        /* Secondary UART (USART2/battery). The hook re-arms its own reception. */
        s_uart2_rx_byte_callback();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    HAL_UART_RxEventTypeTypeDef event_type;
    uint32_t callback_epoch;
    uint32_t event_tick;
    uint32_t primask;
    bool callback_eligible;

    if (huart->Instance != USART3) {
        return;
    }

    /* DMA1_Stream0 and USART3 use the same preemption priority, so the shared
     * HAL RxEventType field cannot be changed by a nested RX event between
     * this read and the bounded bookkeeping call. */
    event_type = HAL_UARTEx_GetRxEventType(huart);
    event_tick = HAL_GetTick();
    primask = JetsonRx_EnterCritical();
    callback_eligible =
        s_rx_dma_callbacks_enabled &&
        !s_rx_rearm_pending &&
        (s_rx_armed || s_rx_arm_attempt_in_progress);
#ifndef AMR_HOST_TEST
    callback_eligible =
        callback_eligible &&
        (huart->hdmarx == &hdma_usart3_rx) &&
        (hdma_usart3_rx.Instance == DMA1_Stream0) &&
        (hdma_usart3_rx.Init.Mode == DMA_CIRCULAR) &&
        ((huart->Instance->CR3 & USART_CR3_DMAR) != 0U);
#endif
    callback_epoch = s_rx_dma_session_epoch;
    JetsonRx_ExitCritical(primask);
    if (!callback_eligible) {
        primask = JetsonRx_EnterCritical();
        JetsonRx_Increment(&s_rx_completion_sequence);
        JetsonRx_Increment(&s_rx_dma_stale_callback_count);
        JetsonRx_ExitCritical(primask);
        return;
    }
    JetsonRx_RecordDmaEvent(
        size, event_type, event_tick, callback_epoch);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    JetsonTx_OnTransmitComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uint32_t error = HAL_UART_GetError(huart);
        uint32_t now = HAL_GetTick();
        uint32_t primask;

        JetsonTx_OnUartErrorFromISR(huart, error);
        /* ISR work ends after O(1) fail-closed bookkeeping. The local H7 HAL
         * has already ended/aborted DMA reception for a DMA-mode UART error;
         * main performs the explicit bounded reset/start after Drive + Lift
         * safety dispatch. */
        primask = JetsonRx_EnterCritical();
        s_rx_last_uart_error = error;
        if (s_rx_dma_callbacks_enabled ||
            s_rx_armed || s_rx_arm_attempt_in_progress) {
            JetsonRx_BeginTransportOutageLocked(
                JETSON_RX_DAMAGE_UART_ERROR,
                HAL_ERROR,
                now);
        } else {
            JetsonRx_Increment(&s_rx_dma_stale_callback_count);
        }
        JetsonRx_ExitCritical(primask);
    } else if ((huart->Instance == USART2) && (s_uart2_error_callback != NULL)) {
        s_uart2_error_callback();
    }
}

/* Strict full-line argument grammar. Every motion / state-changing command
 * is parsed with a cursor that must consume the ENTIRE line: one or more
 * plain spaces between tokens, no extra tokens, no trailing garbage. The
 * old "sscanf the tokens we need and ignore the rest" pattern is gone. */

static bool Parse_SkipRequiredSpaces(const char **cursor)
{
    if (**cursor != ' ') {
        return false;
    }
    while (**cursor == ' ') {
        (*cursor)++;
    }
    return true;
}

/* Strict int32 field: optional '-', one or more decimal digits,
 * overflow-checked against the full int32 range. No '+', no '.', no
 * exponent, no hex, no NaN/Inf text - a garbled byte can therefore never
 * turn one valid number into a different valid number. */
static bool Parse_Int32Field(const char **cursor, int32_t *out_value)
{
    const char *p = *cursor;
    bool negative = false;
    uint32_t magnitude = 0U;
    uint32_t limit;
    bool saw_digit = false;

    if (*p == '-') {
        negative = true;
        p++;
    }
    limit = negative ? 2147483648U : 2147483647U;
    while ((*p >= '0') && (*p <= '9')) {
        uint32_t digit = (uint32_t)(*p - '0');

        if ((magnitude > (limit / 10U)) ||
            ((magnitude == (limit / 10U)) && (digit > (limit % 10U)))) {
            return false;
        }
        magnitude = (magnitude * 10U) + digit;
        saw_digit = true;
        p++;
    }
    if (!saw_digit) {
        return false;
    }
    if (negative) {
        *out_value = (magnitude == 2147483648U) ? INT32_MIN
                                                : -(int32_t)magnitude;
    } else {
        *out_value = (int32_t)magnitude;
    }
    *cursor = p;
    return true;
}

/* "<CMD> <int> [... <int>]" with NOTHING after the last argument. */
static bool Parse_ExactIntArgs(const char *line,
                               const char *cmd,
                               int32_t *values,
                               uint32_t count)
{
    const char *cursor = line;
    size_t cmd_length = strlen(cmd);
    uint32_t index;

    if (strncmp(cursor, cmd, cmd_length) != 0) {
        return false;
    }
    cursor += cmd_length;
    for (index = 0U; index < count; index++) {
        if (!Parse_SkipRequiredSpaces(&cursor)) {
            return false;
        }
        if (!Parse_Int32Field(&cursor, &values[index])) {
            return false;
        }
    }
    return *cursor == '\0';
}

/* Monotonic acknowledgement counter for stateful drive commands, plus the
 * host request nonce echo. The host attaches req=<8-hex> to a command and
 * only trusts responses that echo the same nonce, because its own RX-buffer
 * clearing can never flush frames still queued inside this TX manager. The
 * seq= counter remains as a diagnostic ordering hint. */
static uint32_t s_command_ack_sequence;

typedef enum
{
    REQUEST_NONCE_ABSENT = 0,
    REQUEST_NONCE_VALID,
    REQUEST_NONCE_MALFORMED
} RequestNonceStatus_t;

/* Single strict nonce grammar shared by every nonce-bound command and
 * query: " req=" must appear exactly once, as the final token, followed by
 * exactly eight hex digits and nothing else. 00000000 is reserved as the
 * unbound value and is rejected. */
static RequestNonceStatus_t Parse_RequestNonce(const char *line,
                                               uint32_t *out_nonce)
{
    const char *first = strstr(line, " req=");
    const char *digits;
    uint32_t value = 0U;
    size_t index;

    *out_nonce = 0U;
    if (first == NULL) {
        return REQUEST_NONCE_ABSENT;
    }
    if (strstr(first + 1, " req=") != NULL) {
        return REQUEST_NONCE_MALFORMED;
    }
    digits = first + 5;
    for (index = 0U; index < 8U; index++) {
        char character = digits[index];
        uint32_t nibble;

        if ((character >= '0') && (character <= '9')) {
            nibble = (uint32_t)(character - '0');
        } else if ((character >= 'A') && (character <= 'F')) {
            nibble = 10U + (uint32_t)(character - 'A');
        } else if ((character >= 'a') && (character <= 'f')) {
            nibble = 10U + (uint32_t)(character - 'a');
        } else {
            return REQUEST_NONCE_MALFORMED;
        }
        value = (value << 4) | nibble;
    }
    if (digits[8] != '\0') {
        return REQUEST_NONCE_MALFORMED;
    }
    if (value == 0U) {
        return REQUEST_NONCE_MALFORMED;
    }
    *out_nonce = value;
    return REQUEST_NONCE_VALID;
}

/* LIGHT RGB takes exactly three single-character binary channels. A final
 * req= tail is optional; Parse_RequestNonce validates it before this helper. */
static bool Parse_LightRgbCommand(const char *line,
                                  RequestNonceStatus_t nonce_status,
                                  bool channels[3])
{
    const char *cursor = line;
    uint32_t index;

    if (strncmp(cursor, "LIGHT", 5U) != 0) {
        return false;
    }
    cursor += 5U;
    if (!Parse_SkipRequiredSpaces(&cursor) ||
        (strncmp(cursor, "RGB", 3U) != 0)) {
        return false;
    }
    cursor += 3U;
    for (index = 0U; index < 3U; index++) {
        if (!Parse_SkipRequiredSpaces(&cursor) ||
            ((*cursor != '0') && (*cursor != '1'))) {
            return false;
        }
        channels[index] = (*cursor == '1');
        cursor++;
    }
    if (nonce_status == REQUEST_NONCE_VALID) {
        return Parse_SkipRequiredSpaces(&cursor) &&
               (strncmp(cursor, "req=", 4U) == 0);
    }
    return *cursor == '\0';
}

/* Commands with NO parameters that are nonce-bound accept EXACTLY
 *     <COMMAND> req=XXXXXXXX
 * one or more plain spaces between the command token and the nonce tail,
 * no extra tokens, no prefix or suffix garbage, no duplicated req=. This
 * covers every transaction-starting command AND every structured query.
 * Safety-directed commands (STOP/DISABLE/LIFT_STOP/CMD_VEL) are
 * deliberately excluded: an emergency stop must never be refused because
 * of request-grammar problems. Their acknowledgements simply stay unbound
 * (req=00000000) so a v4 host never mistakes them for its own
 * transaction. */
static bool Command_UsesStrictNoArgGrammar(const char *cmd)
{
    return (strcmp(cmd, "ENABLE") == 0) ||
           (strcmp(cmd, "DRIVE_FAULT_RESET_SAFE") == 0) ||
           (strcmp(cmd, "DRIVE_BASE_SAFE_REFRESH") == 0) ||
           (strcmp(cmd, "LIFT_FAULT_RESET") == 0) ||
           (strcmp(cmd, "LIFT_RST") == 0) ||
           (strcmp(cmd, "FAULT_RST") == 0) ||
           (strcmp(cmd, "LIFT_HOME") == 0) ||
           (strcmp(cmd, "LIFT_RECOVER") == 0) ||
           (strcmp(cmd, "LIFT_RECOVER_TOP") == 0) ||
           (strcmp(cmd, "DRIVE_FAULT_DEBUG") == 0) ||
           (strcmp(cmd, "DRIVE_HEARTBEAT_DEBUG") == 0) ||
           (strcmp(cmd, "DRIVE_NODE_GUARD_DEBUG") == 0) ||
           (strcmp(cmd, "DRIVE_TPDO_STATUS") == 0) ||
           (strcmp(cmd, "DRIVE_TPDO_ACTIVE_ENABLE") == 0) ||
           (strcmp(cmd, "DRIVE_TPDO_ACTIVE_STATUS") == 0) ||
           (strcmp(cmd, "DRIVE_TPDO_ACTIVE_DISABLE") == 0) ||
           (strcmp(cmd, "DRIVE_BASE_SAFE") == 0) ||
           (strcmp(cmd, "DRIVE_CAN_TRACE_STATUS") == 0) ||
           (strcmp(cmd, "DRIVE_CAN_TRACE_DUMP") == 0) ||
           (strcmp(cmd, "DRIVE_YAW_DEBUG") == 0) ||
           (strcmp(cmd, "DRIVE_RPDO_DEBUG") == 0) ||
           (strcmp(cmd, "UART_TX_DEBUG") == 0) ||
           (strcmp(cmd, "UART_RX_DEBUG") == 0) ||
           (strcmp(cmd, "BAT_DEBUG") == 0);
}

/* Motion-arming lift commands take NO arguments and must match the whole
 * line exactly; trailing garbage never starts lift motion. LIFT_STOP and
 * the jog-stop aliases stay exempt (they only reduce danger). */
static bool Command_IsBareLiftMotion(const char *cmd)
{
    return (strcmp(cmd, "LIFT_UP") == 0) ||
           (strcmp(cmd, "LIFT_DOWN") == 0) ||
           (strcmp(cmd, "LIFT_JOG_UP") == 0) ||
           (strcmp(cmd, "LIFT_JOG_UP_SLOW") == 0) ||
           (strcmp(cmd, "LIFT_JOG_DN") == 0) ||
           (strcmp(cmd, "LIFT_JOG_DOWN_SLOW") == 0);
}

/* Full-line shape check for the strict no-argument grammar. Called only
 * after Parse_RequestNonce returned VALID, which already proves the line
 * ends in the single " req=XXXXXXXX" tail; this rejects anything except
 * plain spaces between the command token and that tail, and any prefix
 * before the command token. */
static bool Parse_StrictNoArgShape(const char *line, const char *cmd)
{
    size_t cmd_length = strlen(cmd);
    const char *cursor;

    if (strncmp(line, cmd, cmd_length) != 0) {
        return false;
    }
    cursor = line + cmd_length;
    if (*cursor != ' ') {
        return false;
    }
    while (*cursor == ' ') {
        cursor++;
    }
    return strncmp(cursor, "req=", 4U) == 0;
}

/* Exact commissioning grammar:
 * DRIVE_TPDO_CONFIG node=1|2|all req=XXXXXXXX. The shared nonce parser has
 * already proved the final token is one nonzero eight-hex request nonce. */
static bool Parse_TpdoConfigCommand(const char *line, uint8_t *node_id)
{
    const char *cursor = line;
    static const char command[] = "DRIVE_TPDO_CONFIG";

    if ((node_id == NULL) ||
        (strncmp(cursor, command, sizeof(command) - 1U) != 0)) {
        return false;
    }
    cursor += sizeof(command) - 1U;
    if (!Parse_SkipRequiredSpaces(&cursor) ||
        (strncmp(cursor, "node=", 5U) != 0)) {
        return false;
    }
    cursor += 5U;
    if ((cursor[0] == '1') && (cursor[1] == ' ')) {
        *node_id = LEFT_NODE_ID;
        cursor++;
    } else if ((cursor[0] == '2') && (cursor[1] == ' ')) {
        *node_id = RIGHT_NODE_ID;
        cursor++;
    } else if ((strncmp(cursor, "all", 3U) == 0) && (cursor[3] == ' ')) {
        *node_id = 0U;
        cursor += 3U;
    } else {
        return false;
    }
    if (!Parse_SkipRequiredSpaces(&cursor)) {
        return false;
    }
    return strncmp(cursor, "req=", 4U) == 0;
}

/* Maps a typed command result onto the CMD_ACK reason= token. The
 * dispatcher must never emit "accepted=0 reason=OK": every rejection
 * reports its real cause. */
static const char *JetsonProtocol_CommandResultName(CommandResult_t result)
{
    switch (result) {
    case CMD_RESULT_ACCEPTED:                return "OK";
    case CMD_RESULT_BAD_REQ:                 return "BAD_REQ";
    case CMD_RESULT_BAD_COMMAND:             return "BAD_COMMAND";
    case CMD_RESULT_TRANSITION_BUSY:         return "TRANSITION_BUSY";
    case CMD_RESULT_LIFT_ACTIVE:             return "LIFT_ACTIVE";
    case CMD_RESULT_NOT_CONFIRMED_DISABLED:  return "NOT_CONFIRMED_DISABLED";
    case CMD_RESULT_FAULT_LATCHED:           return "FAULT_LATCHED";
    case CMD_RESULT_NOT_ENABLED:             return "NOT_ENABLED";
    case CMD_RESULT_BASE_NOT_SAFE:           return "BASE_NOT_SAFE";
    case CMD_RESULT_OPERATION_ENABLED:       return "OPERATION_ENABLED";
    case CMD_RESULT_INTERNAL_ERROR:          return "INTERNAL_ERROR";
    default:                                 return "INTERNAL_ERROR";
    }
}

static void JetsonProtocol_Ack(const char *command,
                               bool accepted,
                               const char *reason,
                               uint32_t request_nonce)
{
    s_command_ack_sequence++;
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "CMD_ACK %s seq=%lu accepted=%u reason=%s req=%08lX\r\n",
        command,
        (unsigned long)s_command_ack_sequence,
        accepted ? 1U : 0U,
        reason,
        (unsigned long)request_nonce);
}

/* Strict full-line YAW_PI grammar: "YAW_PI <ACTION>" where ACTION is one
 * uppercase-letter token and the line ends immediately after it. */
static bool Parse_YawPi_Command(const char *line, char action[16])
{
    const char *cursor = line;
    uint32_t length = 0U;

    if (strncmp(cursor, "YAW_PI", 6U) != 0) {
        return false;
    }
    cursor += 6U;
    if (!Parse_SkipRequiredSpaces(&cursor)) {
        return false;
    }
    while ((*cursor >= 'A') && (*cursor <= 'Z')) {
        if (length >= 15U) {
            return false;
        }
        action[length++] = *cursor;
        cursor++;
    }
    action[length] = '\0';
    return (length > 0U) && (*cursor == '\0');
}

/* Bound single-line RX framing diagnostics: every fail-closed counter and
 * the current sync state, echoing the query's req= and a snapshot id. */
static void JetsonProtocol_PrintRxDebug(uint32_t request_nonce)
{
    uint32_t now = HAL_GetTick();
    uint32_t ring_overflow;
    uint32_t line_overflow;
    uint32_t uart_error;
    uint32_t invalid_char;
    uint32_t assembly_timeout;
    uint32_t stale_line;
    uint32_t damage_generation;
    uint32_t desync;
    uint32_t arm_attempt_sequence;
    uint32_t completion_sequence;
    uint32_t rearm_failures;
    uint32_t rearm_recoveries;
    uint32_t rearm_fault_events;
    uint32_t immediate_stop_events;
    uint32_t immediate_stop_dispatches;
    uint32_t consecutive_failures;
    uint32_t outage_start_tick;
    uint32_t outage_age;
    uint32_t byte_budget_hits;
    uint32_t command_budget_hits;
    bool armed;
    bool rearm_pending;
    bool immediate_stop_pending;
    bool synced;
    JetsonRxLinkState_t link_state;
    HAL_StatusTypeDef last_hal_status;
    uint32_t primask = JetsonRx_EnterCritical();

    ring_overflow = s_jetson_rx_overflow_count;
    line_overflow = s_rx_line_overflow_count;
    uart_error = s_rx_uart_error_count;
    invalid_char = s_rx_invalid_char_count;
    assembly_timeout = s_rx_assembly_timeout_count;
    stale_line = s_rx_stale_line_count;
    damage_generation = s_rx_damage_generation;
    desync = s_rx_desync_count;
    armed = s_rx_armed;
    rearm_pending = s_rx_rearm_pending;
    arm_attempt_sequence = s_rx_arm_attempt_sequence;
    completion_sequence = s_rx_completion_sequence;
    link_state = s_rx_link_state;
    rearm_failures = s_rx_rearm_failure_count;
    rearm_recoveries = s_rx_rearm_recovery_count;
    rearm_fault_events = s_rx_rearm_fault_event_count;
    immediate_stop_pending = s_rx_immediate_safety_stop_pending;
    immediate_stop_events = s_rx_immediate_safety_stop_event_count;
    immediate_stop_dispatches = s_rx_immediate_safety_stop_dispatch_count;
    consecutive_failures = s_rx_rearm_consecutive_failures;
    outage_start_tick = s_rx_outage_start_tick;
    last_hal_status = s_rx_last_rearm_hal_status;
    byte_budget_hits = s_rx_byte_budget_hit_count;
    command_budget_hits = s_rx_command_budget_hit_count;
    synced = !s_rx_wire_discard_until_eol &&
             !s_rx_generation_exhausted;
    JetsonRx_ExitCritical(primask);
    outage_age = (link_state == RX_LINK_HEALTHY) ? 0U :
        (uint32_t)(now - outage_start_tick);

    s_rx_debug_snapshot_sequence++;
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "UART_RX_DEBUG 1/5 armed=%u rearm_pending=%u arm_seq=%lu completion_seq=%lu link_state=%s outage_age=%lu snap=%u req=%08lX\r\n",
        armed ? 1U : 0U,
        rearm_pending ? 1U : 0U,
        (unsigned long)arm_attempt_sequence,
        (unsigned long)completion_sequence,
        JetsonRx_LinkStateName(link_state),
        (unsigned long)outage_age,
        (unsigned)s_rx_debug_snapshot_sequence,
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "UART_RX_DEBUG 2/5 rearm_fail=%lu rearm_recover=%lu rearm_fault=%lu consecutive=%lu last_hal=%d snap=%u req=%08lX\r\n",
        (unsigned long)rearm_failures,
        (unsigned long)rearm_recoveries,
        (unsigned long)rearm_fault_events,
        (unsigned long)consecutive_failures,
        (int)last_hal_status,
        (unsigned)s_rx_debug_snapshot_sequence,
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "UART_RX_DEBUG 3/5 stop_pending=%u stop_events=%lu stop_dispatch=%lu ring_ovf=%lu line_ovf=%lu snap=%u req=%08lX\r\n",
        immediate_stop_pending ? 1U : 0U,
        (unsigned long)immediate_stop_events,
        (unsigned long)immediate_stop_dispatches,
        (unsigned long)ring_overflow,
        (unsigned long)line_overflow,
        (unsigned)s_rx_debug_snapshot_sequence,
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "UART_RX_DEBUG 4/5 uart_err=%lu bad_ch=%lu asm_to=%lu stale=%lu damage_gen=%lu desync=%lu snap=%u req=%08lX\r\n",
        (unsigned long)uart_error,
        (unsigned long)invalid_char,
        (unsigned long)assembly_timeout,
        (unsigned long)stale_line,
        (unsigned long)damage_generation,
        (unsigned long)desync,
        (unsigned)s_rx_debug_snapshot_sequence,
        (unsigned long)request_nonce);
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "UART_RX_DEBUG 5/5 byte_budget=%lu cmd_budget=%lu sync=%u snap=%u req=%08lX\r\n",
        (unsigned long)byte_budget_hits,
        (unsigned long)command_budget_hits,
        synced ? 1U : 0U,
        (unsigned)s_rx_debug_snapshot_sequence,
        (unsigned long)request_nonce);
}

void JetsonProtocol_Execute(const char *line)
{
    char cmd[32] = {0};
    char yaw_pi_action[16] = {0};
    uint8_t tpdo_config_node = 0U;
    uint32_t request_nonce;
    RequestNonceStatus_t nonce_status;

    if (sscanf(line, "%31s", cmd) != 1) {
        return;
    }
    nonce_status = Parse_RequestNonce(line, &request_nonce);
    if (strcmp(cmd, "LIGHT") == 0) {
        bool channels[3];

        if (nonce_status == REQUEST_NONCE_MALFORMED) {
            JetsonProtocol_Ack("LIGHT", false, "BAD_REQ", 0U);
        } else if (!Parse_LightRgbCommand(line, nonce_status, channels)) {
            JetsonProtocol_Ack("LIGHT", false, "BAD_COMMAND", request_nonce);
        } else {
            StatusLight_RequestRgb(channels[0], channels[1], channels[2]);
            JetsonProtocol_Ack("LIGHT", true, "OK", request_nonce);
        }
        return;
    }
    if (strcmp(cmd, "DRIVE_TPDO_CONFIG") == 0) {
        if (nonce_status != REQUEST_NONCE_VALID) {
            (void)JetsonTx_Printf(
                JETSON_TX_PRIORITY_STATUS,
                "DRIVE_TPDO_CONFIG state=FAILED reason=BAD_REQ node=0 req=00000000\r\n");
            return;
        }
        if (!Parse_TpdoConfigCommand(line, &tpdo_config_node)) {
            (void)JetsonTx_Printf(
                JETSON_TX_PRIORITY_STATUS,
                "DRIVE_TPDO_CONFIG state=FAILED reason=BAD_COMMAND node=0 req=%08lX\r\n",
                (unsigned long)request_nonce);
            return;
        }
    }
    if (Command_UsesStrictNoArgGrammar(cmd)) {
        if (nonce_status != REQUEST_NONCE_VALID) {
            /* Strict commands never run with a missing or malformed
             * nonce: reject BEFORE any side effect. */
            JetsonProtocol_Ack(cmd, false, "BAD_REQ", 0U);
            return;
        }
        if (!Parse_StrictNoArgShape(line, cmd)) {
            /* Valid nonce but extra tokens or garbage in the line shape:
             * reject BEFORE any side effect, echoing the valid nonce so
             * the host can bind the rejection to its own request. */
            JetsonProtocol_Ack(cmd, false, "BAD_COMMAND", request_nonce);
            return;
        }
    }
    if (Command_IsBareLiftMotion(cmd) && (strcmp(line, cmd) != 0)) {
        /* Motion-arming lift commands take no arguments: the whole line
         * must be exactly the command. Trailing garbage never queues a
         * lift action. */
        JetsonProtocol_Ack(cmd, false, "BAD_COMMAND", 0U);
        return;
    }
    Lift_SetCommandRequestNonce(request_nonce);

    if (strcmp(cmd, "CMD_VEL") == 0) {
        int32_t cmd_vel_args[2] = {0, 0};

        if (!Parse_ExactIntArgs(line, "CMD_VEL", cmd_vel_args, 2U)) {
            /* A malformed motion command is rejected outright - even
             * "CMD_VEL 0 0 <garbage>". Stopping is STOP's job; a garbled
             * line must never command the base. */
            JetsonProtocol_Ack("CMD_VEL", false, "BAD_COMMAND", 0U);
        } else {
            Drive_CommandSetCmdVel((float)cmd_vel_args[0],
                                   (float)cmd_vel_args[1]);
        }
    } else if (strcmp(cmd, "RAW") == 0) {
        int32_t raw_args[3] = {0, 0, 0};

        if (!Parse_ExactIntArgs(line, "RAW", raw_args, 3U) ||
            (raw_args[2] < 0)) {
            JetsonProtocol_Ack("RAW", false, "BAD_COMMAND", 0U);
        } else if (ENABLE_DANGEROUS_RAW_TEST == 0) {
            /* The blocking RAW backdoor is disabled in every standard
             * build profile; see ENABLE_DANGEROUS_RAW_TEST. No motor
             * command, no blocking loop. */
            JetsonProtocol_Ack("RAW", false, "RAW_DISABLED", 0U);
        } else if (s_raw_command_callback != NULL) {
            s_raw_command_callback(
                Clamp_TestMotorRPM(raw_args[0]),
                Clamp_TestMotorRPM(raw_args[1]),
                Clamp_TestDurationMS((uint32_t)raw_args[2]));
        }
#if APP_ENABLE_IMU
    } else if (strcmp(cmd, "IMU_DEBUG") == 0) {
        IMU_PrintDebug();
#endif
#if APP_ENABLE_BATTERY
    } else if (strcmp(cmd, "BAT_DEBUG") == 0) {
        Battery_PrintDebug(request_nonce);
#endif
    } else if ((strcmp(cmd, "FW_VERSION") == 0) ||
               (strcmp(cmd, "VERSION") == 0)) {
        FirmwareIdentity_Print();
    } else if (strcmp(cmd, "UART_TX_DEBUG") == 0) {
        JetsonTx_PrintDebug(request_nonce);
    } else if (strcmp(cmd, "UART_RX_DEBUG") == 0) {
        JetsonProtocol_PrintRxDebug(request_nonce);
    } else if (strcmp(cmd, "DRIVE_FAULT_DEBUG") == 0) {
        Drive_PrintFaultDebug(request_nonce);
    } else if (strcmp(cmd, "DRIVE_HEARTBEAT_DEBUG") == 0) {
        Drive_PrintHeartbeatDebug(request_nonce);
    } else if (strcmp(cmd, "DRIVE_NODE_GUARD_DEBUG") == 0) {
        Drive_PrintHeartbeatCompatibilityDebug(request_nonce);
    } else if (strcmp(cmd, "DRIVE_TPDO_CONFIG") == 0) {
        (void)Drive_CommandConfigureWheelTpdo(
            tpdo_config_node, request_nonce);
    } else if (strcmp(cmd, "DRIVE_TPDO_STATUS") == 0) {
        Drive_PrintWheelTpdoStatus(request_nonce);
    } else if (strcmp(cmd, "DRIVE_TPDO_ACTIVE_ENABLE") == 0) {
        (void)Drive_CommandWheelTpdoActiveEnable(request_nonce);
    } else if (strcmp(cmd, "DRIVE_TPDO_ACTIVE_STATUS") == 0) {
        Drive_PrintWheelTpdoActiveStatus(request_nonce);
    } else if (strcmp(cmd, "DRIVE_TPDO_ACTIVE_DISABLE") == 0) {
        (void)Drive_CommandWheelTpdoActiveDisable(request_nonce);
    } else if (strcmp(cmd, "DRIVE_FAULT_RESET_SAFE") == 0) {
        (void)Drive_CommandFaultResetSafe(request_nonce);
    } else if (strcmp(cmd, "DRIVE_BASE_SAFE_REFRESH") == 0) {
        (void)Drive_RequestBaseSafeRefresh(request_nonce);
    } else if (strcmp(cmd, "DRIVE_BASE_SAFE") == 0) {
        Drive_PrintBaseSafeDebug(request_nonce);
    } else if (strcmp(cmd, "DRIVE_CAN_TRACE_STATUS") == 0) {
        (void)CanTrace_RequestStatus(request_nonce);
    } else if (strcmp(cmd, "DRIVE_CAN_TRACE_DUMP") == 0) {
        (void)CanTrace_RequestDump(request_nonce);
    } else if (Lift_HandleCommand(cmd)) {
        /* Lift handler sends its own LIFT status line. */
    } else if (strcmp(cmd, "STOP") == 0) {
        Drive_CommandStop();
        JetsonProtocol_Ack("STOP", true,
                           (nonce_status == REQUEST_NONCE_MALFORMED) ?
                           "BAD_REQ" : "OK",
                           request_nonce);
    } else if (strcmp(cmd, "ENABLE") == 0) {
        CommandResult_t enable_result = Drive_CommandEnableChecked();

        JetsonProtocol_Ack("ENABLE",
                           enable_result == CMD_RESULT_ACCEPTED,
                           JetsonProtocol_CommandResultName(enable_result),
                           request_nonce);
    } else if (strcmp(cmd, "DISABLE") == 0) {
        CommandResult_t disable_result = Drive_CommandDisableChecked();

        JetsonProtocol_Ack("DISABLE",
                           disable_result == CMD_RESULT_ACCEPTED,
                           (nonce_status == REQUEST_NONCE_MALFORMED) ?
                           "BAD_REQ" :
                           JetsonProtocol_CommandResultName(disable_result),
                           request_nonce);
    } else if (strcmp(cmd, "YAW_PI") == 0) {
        bool enable;

        if (!Parse_YawPi_Command(line, yaw_pi_action)) {
            /* Wrong shape (extra tokens, missing/garbled action): reject
             * before any PI enable-state change. */
            uart_printf("Usage: YAW_PI ON|OFF|STATUS\r\n");
            JetsonProtocol_Ack("YAW_PI", false, "BAD_COMMAND", 0U);
            return;
        }
        if (strcmp(yaw_pi_action, "STATUS") == 0) {
            Drive_PrintYawPiStatus();
            return;
        }
        if (strcmp(yaw_pi_action, "ON") == 0) {
            enable = true;
        } else if (strcmp(yaw_pi_action, "OFF") == 0) {
            enable = false;
        } else {
            uart_printf("Usage: YAW_PI ON|OFF|STATUS\r\n");
            JetsonProtocol_Ack("YAW_PI", false, "BAD_COMMAND", 0U);
            return;
        }
        if (!Drive_CommandSetYawPiEnabled(enable)) {
            uart_printf("YAW_PI rejected=moving\r\n");
            return;
        }
        uart_printf("YAW_PI enabled=%u default=%u accepted=1\r\n",
                    Drive_IsYawPiEnabled() ? 1U : 0U,
                    (DRIVE_YAW_RATE_PI_DEFAULT_ENABLED != 0U) ? 1U : 0U);
    } else if (strcmp(cmd, "DRIVE_RPDO_DEBUG") == 0) {
        DriveTarget_PrintDiagnostics(request_nonce);
    } else if (strcmp(cmd, "DRIVE_YAW_DEBUG") == 0) {
        Drive_PrintYawRateDiagnostics(request_nonce);
    }
}

void JetsonProtocol_PollLine(void)
{
    JetsonRxEntry_t entry;
    uint32_t byte_count = 0U;
    uint32_t command_count = 0U;
    bool byte_budget_hit = false;
    bool command_budget_hit = false;

    JetsonProtocol_PrintPendingWarnings();

    while ((byte_count < JETSON_RX_BYTE_BUDGET_PER_LOOP) &&
           (command_count < JETSON_RX_COMMAND_BUDGET_PER_LOOP) &&
           JetsonRxRing_Pop(&s_jetson_rx_ring, &entry)) {
        uint8_t ch = entry.byte;

        byte_count++;
#ifdef AMR_HOST_TEST
        if (s_test_rx_error_after_pops != 0U) {
            s_test_rx_error_after_pops--;
            if (s_test_rx_error_after_pops == 0U) {
                huart3.ErrorCode = HAL_UART_ERROR_ORE;
                HAL_UART_ErrorCallback(&huart3);
                huart3.ErrorCode = HAL_UART_ERROR_NONE;
            }
        }
#endif

        /* Generation is checked before every consumed entry.  A callback
         * that ran after Pop flushed the rest of the ring and changed this
         * comparison, invalidating the in-progress line before execution. */
        if (!JetsonRx_LineGenerationIsValid(entry.stream_generation)) {
            JetsonRx_ResetAssembly();
            continue;
        }
        if ((s_jetson_len > 0U) &&
            (s_rx_line_generation != entry.stream_generation)) {
            /* The entry already belongs to the current valid generation.
             * Only the main-owned partial assembly is old.  Drop that old
             * partial state, adopt this entry's generation, and process this
             * same entry exactly once below.  This is resynchronization, not
             * new wire damage: do not advance generation, flush, or discard. */
            JetsonRx_ResetPartialAssemblyForGeneration(
                entry.stream_generation);
        }
#if DEBUG_JETSON_RX_BYTES
        if ((ch >= 0x20U) && (ch <= 0x7EU)) {
            uart_printf("USART3 got byte: %c\r\n", ch);
        } else {
            uart_printf("USART3 got byte: 0x%02X\r\n", ch);
        }
#endif

        if ((ch == '\r') || (ch == '\n')) {
            if (s_jetson_len > 0U) {
                uint32_t now_ms = HAL_GetTick();
                uint32_t line_generation = s_rx_line_generation;
                bool stale;

                command_count++;
                s_rx_last_arrival_tick = entry.arrival_tick;
                s_jetson_line[s_jetson_len] = '\0';
                stale = ((uint32_t)(s_rx_last_arrival_tick -
                                    s_rx_first_arrival_tick) >
                         JETSON_RX_ASSEMBLY_TIMEOUT_MS) ||
                        ((uint32_t)(now_ms - s_rx_first_arrival_tick) >
                         JETSON_RX_ASSEMBLY_TIMEOUT_MS);
                if (stale) {
                    uint32_t primask = JetsonRx_EnterCritical();
                    JetsonRx_Increment(&s_rx_stale_line_count);
                    JetsonRx_ExitCritical(primask);
                    /* The complete stale frame includes its same-generation
                     * EOL, so it can be dropped without wire desync.  This
                     * differs from a timed-out partial frame, which must wait
                     * for a FUTURE wire EOL.  Stale STOP is not excepted. */
                    JetsonRx_ResetAssembly();
                } else if (JetsonRx_LineGenerationIsValid(line_generation)) {
                    /* Final generation check immediately before dispatch.
                     * Reset partial state first; the completed frame is now
                     * independent of a later error in the following frame. */
                    JetsonRx_ResetAssembly();
                    JetsonProtocol_Execute(s_jetson_line);
                } else {
                    JetsonRx_ResetAssembly();
                }
            }
            continue;
        }

        if ((ch < 0x20U) || (ch > 0x7EU)) {
            JetsonRx_MarkDamage(JETSON_RX_DAMAGE_INVALID_CHAR);
            JetsonRx_ResetAssembly();
            continue;
        }

        if (s_jetson_len >= (UART_COMMAND_LINE_MAX - 1U)) {
            JetsonRx_MarkDamage(JETSON_RX_DAMAGE_LINE_OVERFLOW);
            JetsonRx_ResetAssembly();
            continue;
        }

        if (s_jetson_len == 0U) {
            s_rx_line_generation = entry.stream_generation;
            s_rx_first_arrival_tick = entry.arrival_tick;
        } else if ((uint32_t)(entry.arrival_tick -
                              s_rx_first_arrival_tick) >
                   JETSON_RX_ASSEMBLY_TIMEOUT_MS) {
            /* No EOL has been consumed and wire assembly already exceeded
             * the deadline: damage the stream and wait for a future EOL. */
            JetsonRx_MarkDamage(JETSON_RX_DAMAGE_ASSEMBLY_TIMEOUT);
            JetsonRx_ResetAssembly();
            continue;
        }
        s_rx_last_arrival_tick = entry.arrival_tick;
        s_jetson_line[s_jetson_len++] = (char)ch;
    }

    if ((byte_count >= JETSON_RX_BYTE_BUDGET_PER_LOOP) &&
        (JetsonRx_QueuedCount() != 0U)) {
        byte_budget_hit = true;
    }
    if ((command_count >= JETSON_RX_COMMAND_BUDGET_PER_LOOP) &&
        (JetsonRx_QueuedCount() != 0U)) {
        command_budget_hit = true;
    }
    if (byte_budget_hit || command_budget_hit) {
        uint32_t primask = JetsonRx_EnterCritical();
        if (byte_budget_hit) {
            JetsonRx_Increment(&s_rx_byte_budget_hit_count);
        }
        if (command_budget_hit) {
            JetsonRx_Increment(&s_rx_command_budget_hit_count);
        }
        JetsonRx_ExitCritical(primask);
    }

    if (s_jetson_len > 0U) {
        uint32_t now_ms = HAL_GetTick();

        if (!JetsonRx_LineGenerationIsValid(s_rx_line_generation)) {
            JetsonRx_ResetAssembly();
        } else if ((JetsonRx_QueuedCount() == 0U) &&
                   ((uint32_t)(now_ms - s_rx_first_arrival_tick) >
                    JETSON_RX_ASSEMBLY_TIMEOUT_MS)) {
            /* Unlike a complete stale line, a partial timed-out line has no
             * trusted delimiter.  Flush and require a future wire EOL. */
            JetsonRx_MarkDamage(JETSON_RX_DAMAGE_ASSEMBLY_TIMEOUT);
            JetsonRx_ResetAssembly();
        }
    }
}

#ifdef AMR_HOST_TEST
/* Host-test hooks: the RX framing tests drive the REAL production ring,
 * poller, and dispatcher - never a reference reimplementation. */
void JetsonProtocol_TestInjectRxByte(uint8_t byte)
{
    JetsonRx_HandleWireByte(byte, HAL_GetTick());
}

void JetsonProtocol_TestInjectRxError(void)
{
    huart3.ErrorCode = HAL_UART_ERROR_ORE;
    HAL_UART_ErrorCallback(&huart3);
    huart3.ErrorCode = HAL_UART_ERROR_NONE;
}

void JetsonProtocol_TestScheduleRxErrorAfterPops(uint32_t pop_count)
{
    s_test_rx_error_after_pops = pop_count;
}

void JetsonProtocol_TestSetRxCompletionSequence(uint32_t sequence)
{
    uint32_t primask = JetsonRx_EnterCritical();

    s_rx_completion_sequence = sequence;
    JetsonRx_ExitCritical(primask);
}

uint8_t *JetsonProtocol_TestGetRxDmaBuffer(void)
{
    return s_jetson_rx_dma_buffer;
}

uint32_t JetsonProtocol_TestGetRxDmaBufferSize(void)
{
    return JETSON_RX_DMA_BUFFER_SIZE;
}

uint32_t JetsonProtocol_TestGetRxDmaEpoch(void)
{
    uint32_t primask = JetsonRx_EnterCritical();
    uint32_t epoch = s_rx_dma_session_epoch;

    JetsonRx_ExitCritical(primask);
    return epoch;
}

void JetsonProtocol_TestWriteRxDma(uint32_t absolute_index,
                                  const uint8_t *data,
                                  uint32_t length)
{
    uint32_t index;

    for (index = 0U; index < length; index++) {
        s_jetson_rx_dma_buffer[
            (absolute_index + index) & JETSON_RX_DMA_BUFFER_MASK] =
            data[index];
    }
}

void JetsonProtocol_TestInjectRxDmaEvent(uint16_t size,
                                        uint32_t event_type,
                                        uint32_t callback_epoch)
{
    JetsonRx_RecordDmaEvent(
        size,
        (HAL_UART_RxEventTypeTypeDef)event_type,
        HAL_GetTick(),
        callback_epoch);
}

void JetsonProtocol_TestSetRxDmaAccounting(uint32_t ht_epoch,
                                           uint32_t tc_epoch,
                                           uint32_t produced_total,
                                           uint32_t consumed_total)
{
    uint32_t primask = JetsonRx_EnterCritical();

    s_rx_dma_ht_epoch = ht_epoch;
    s_rx_dma_tc_epoch = tc_epoch;
    s_rx_dma_produced_total = produced_total;
    s_rx_dma_consumed_total = consumed_total;
    s_rx_dma_phase_mismatch_pending = false;
    s_rx_dma_backlog_tick_valid =
        produced_total != consumed_total;
    if (s_rx_dma_backlog_tick_valid) {
        s_rx_dma_backlog_arrival_tick = HAL_GetTick();
    }
    JetsonRx_ExitCritical(primask);
}

void JetsonProtocol_TestScheduleRxDmaHardwareTotalAfterCopy(
    uint32_t hardware_total)
{
    uint32_t primask = JetsonRx_EnterCritical();

    s_test_rx_dma_total_after_copy = hardware_total;
    s_test_rx_dma_advance_after_copy_pending = true;
    JetsonRx_ExitCritical(primask);
}

void JetsonProtocol_TestResetRxState(void)
{
    uint32_t primask = JetsonRx_EnterCritical();

    s_jetson_rx_ring.head = 0U;
    s_jetson_rx_ring.tail = 0U;
    s_rx_damage_generation = 0U;
    s_rx_wire_discard_until_eol = false;
    s_rx_generation_exhausted = false;
    s_jetson_rx_overflow_count = 0U;
    s_rx_line_overflow_count = 0U;
    s_rx_uart_error_count = 0U;
    s_rx_invalid_char_count = 0U;
    s_rx_assembly_timeout_count = 0U;
    s_rx_stale_line_count = 0U;
    s_rx_desync_count = 0U;
    s_rx_byte_budget_hit_count = 0U;
    s_rx_command_budget_hit_count = 0U;
    s_rx_armed = false;
    s_rx_rearm_pending = false;
    s_rx_arm_attempt_in_progress = false;
    s_rx_arm_attempt_sequence = 0U;
    s_rx_completion_sequence = 0U;
    s_rx_rearm_failure_count = 0U;
    s_rx_rearm_recovery_count = 0U;
    s_rx_rearm_fault_event_count = 0U;
    s_rx_immediate_safety_stop_pending = false;
    s_rx_immediate_safety_stop_event_count = 0U;
    s_rx_immediate_safety_stop_dispatch_count = 0U;
    s_rx_rearm_consecutive_failures = 0U;
    s_rx_link_state = RX_LINK_HEALTHY;
    s_rx_outage_start_tick = 0U;
    s_rx_last_rearm_hal_status = HAL_OK;
    s_rx_dma_callbacks_enabled = false;
    s_rx_dma_restart_reset_required = false;
    s_rx_dma_phase_mismatch_pending = false;
    s_rx_dma_session_epoch = 0U;
    s_rx_dma_ht_epoch = 0U;
    s_rx_dma_tc_epoch = 0U;
    s_rx_dma_produced_total = 0U;
    s_rx_dma_consumed_total = 0U;
    s_rx_dma_last_event_position = 0U;
    s_rx_dma_backlog_arrival_tick = 0U;
    s_rx_dma_empty_lower_bound_tick = 0U;
    s_rx_dma_last_event_earliest_tick = 0U;
    s_rx_dma_backlog_tick_valid = false;
    s_rx_dma_backlog_high_water = 0U;
    s_rx_dma_idle_event_count = 0U;
    s_rx_dma_ht_event_count = 0U;
    s_rx_dma_tc_event_count = 0U;
    s_rx_dma_duplicate_event_count = 0U;
    s_rx_dma_overwrite_count = 0U;
    s_rx_dma_stale_callback_count = 0U;
    s_rx_last_uart_error = 0U;
    s_test_rx_dma_hardware_total_override_valid = false;
    s_test_rx_dma_hardware_total_override = 0U;
    s_test_rx_dma_advance_after_copy_pending = false;
    s_test_rx_dma_total_after_copy = 0U;
    JetsonRx_ExitCritical(primask);
    memset(s_jetson_rx_dma_buffer, 0,
           sizeof(s_jetson_rx_dma_buffer));
    memset(s_jetson_rx_dma_copy, 0,
           sizeof(s_jetson_rx_dma_copy));
    s_test_rx_error_after_pops = 0U;
    JetsonRx_ResetAssembly();
}

void JetsonProtocol_TestGetRxStats(uint32_t *ring_overflow,
                                   uint32_t *line_overflow,
                                   uint32_t *uart_error,
                                   uint32_t *invalid_char,
                                   uint32_t *assembly_timeout,
                                   uint32_t *desync,
                                   uint32_t *synced)
{
    *ring_overflow = s_jetson_rx_overflow_count;
    *line_overflow = s_rx_line_overflow_count;
    *uart_error = s_rx_uart_error_count;
    *invalid_char = s_rx_invalid_char_count;
    *assembly_timeout = s_rx_assembly_timeout_count;
    *desync = s_rx_desync_count;
    *synced = (!s_rx_wire_discard_until_eol &&
               !s_rx_generation_exhausted) ? 1U : 0U;
}

void JetsonProtocol_TestGetRxDetailedStats(
    JetsonProtocolTestRxStats_t *stats)
{
    uint32_t primask = JetsonRx_EnterCritical();

    stats->ring_overflow = s_jetson_rx_overflow_count;
    stats->line_overflow = s_rx_line_overflow_count;
    stats->uart_error = s_rx_uart_error_count;
    stats->invalid_char = s_rx_invalid_char_count;
    stats->assembly_timeout = s_rx_assembly_timeout_count;
    stats->stale_line = s_rx_stale_line_count;
    stats->damage_generation = s_rx_damage_generation;
    stats->desync = s_rx_desync_count;
    stats->wire_discard = s_rx_wire_discard_until_eol ? 1U : 0U;
    stats->rx_armed = s_rx_armed ? 1U : 0U;
    stats->rearm_pending = s_rx_rearm_pending ? 1U : 0U;
    stats->arm_attempt_sequence = s_rx_arm_attempt_sequence;
    stats->completion_sequence = s_rx_completion_sequence;
    stats->link_state = (uint32_t)s_rx_link_state;
    stats->outage_age = (s_rx_link_state == RX_LINK_HEALTHY) ? 0U :
        (uint32_t)(HAL_GetTick() - s_rx_outage_start_tick);
    stats->rearm_failures = s_rx_rearm_failure_count;
    stats->rearm_recoveries = s_rx_rearm_recovery_count;
    stats->rearm_fault_events = s_rx_rearm_fault_event_count;
    stats->immediate_stop_pending =
        s_rx_immediate_safety_stop_pending ? 1U : 0U;
    stats->immediate_stop_events =
        s_rx_immediate_safety_stop_event_count;
    stats->immediate_stop_dispatches =
        s_rx_immediate_safety_stop_dispatch_count;
    stats->rearm_consecutive_failures =
        s_rx_rearm_consecutive_failures;
    stats->last_rearm_hal_status =
        (uint32_t)s_rx_last_rearm_hal_status;
    stats->byte_budget_hits = s_rx_byte_budget_hit_count;
    stats->command_budget_hits = s_rx_command_budget_hit_count;
    stats->dma_session_epoch = s_rx_dma_session_epoch;
    stats->dma_produced_total = s_rx_dma_produced_total;
    stats->dma_consumed_total = s_rx_dma_consumed_total;
    stats->dma_backlog =
        (uint32_t)(s_rx_dma_produced_total -
                   s_rx_dma_consumed_total);
    stats->dma_backlog_high_water =
        s_rx_dma_backlog_high_water;
    stats->dma_idle_events = s_rx_dma_idle_event_count;
    stats->dma_ht_events = s_rx_dma_ht_event_count;
    stats->dma_tc_events = s_rx_dma_tc_event_count;
    stats->dma_duplicate_events =
        s_rx_dma_duplicate_event_count;
    stats->dma_overwrites = s_rx_dma_overwrite_count;
    stats->dma_stale_callbacks =
        s_rx_dma_stale_callback_count;
    stats->dma_last_event_position =
        (uint32_t)s_rx_dma_last_event_position;
    stats->last_uart_error = s_rx_last_uart_error;
    JetsonRx_ExitCritical(primask);
    stats->queued_entries = JetsonRx_QueuedCount();
}
#endif /* AMR_HOST_TEST */
