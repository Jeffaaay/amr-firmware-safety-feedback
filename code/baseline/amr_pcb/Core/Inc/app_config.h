/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : app_config.h
  * @brief          : Shared AMR firmware configuration constants.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#define ENABLE_BLOCKING_COM1_CONSOLE 0
#define ENABLE_JETSON_UART_MODE 1
#define ENABLE_JETSON_SELF_LOOPBACK_TEST 0
#define DEBUG_JETSON_RX_BYTES 0
/* Verbose periodic debug streams. Defaults are the BENCH_DEBUG profile; the
 * RELEASE_TEST build profile overrides CMD_VEL/SDO/statusword to 0 on the
 * compiler command line (see tools/clean_build.ps1). On-demand diagnostic
 * commands and critical fault events are unaffected by these switches. */
#ifndef DEBUG_CMD_VEL
#define DEBUG_CMD_VEL 1
#endif
#ifndef DEBUG_SDO
#define DEBUG_SDO 1
#endif
#ifndef DEBUG_STATUSWORD
#define DEBUG_STATUSWORD 1
#endif
#ifndef DEBUG_TO_JETSON
#define DEBUG_TO_JETSON 1
#endif
/* DANGEROUS developer-only backdoor: the blocking RAW test move (up to 30 s
 * with Jetson polling, PI, tracking supervision, watchdog, IMU, and battery
 * updates all paused). Default 0 in EVERY profile; the RELEASE_TEST profile
 * additionally enforces 0 at compile time (see jetson_protocol.c). No
 * supported build-only script may enable it; a developer must pass
 * -DENABLE_DANGEROUS_RAW_TEST=1 by hand and owns the consequences.
 * FW_VERSION reports the compiled state as raw_test=<0|1>; every supported
 * RELEASE_TEST and BENCH_DEBUG build injects raw_test=0. */
#ifndef ENABLE_DANGEROUS_RAW_TEST
#define ENABLE_DANGEROUS_RAW_TEST 0
#endif
#define UART_RX_RING_SIZE 2048U
/* A partial Jetson command line must complete (reach CR/LF) within this
 * window of its first byte, or the whole line is discarded and the RX
 * framer waits for the next EOL boundary (fail-closed re-sync). */
#define JETSON_RX_ASSEMBLY_TIMEOUT_MS 300U
/* USART3 cooperative-loop bounds and fail-closed receive recovery.  RX arm
 * retries are nonblocking: callbacks try once, then the main loop tries once
 * per service opportunity. */
#define JETSON_RX_BYTE_BUDGET_PER_LOOP      256U
#define JETSON_RX_COMMAND_BUDGET_PER_LOOP     8U
#define JETSON_RX_REARM_FAULT_MS            100U

#define CAN_BAUD_500K 500000U
#define CAN_BAUD_250K 250000U
#define CAN_BAUD_1M   1000000U

/* Diagnostic selection. FDCAN kernel clock is assumed to be 48 MHz. */
#define CAN_DIAG_BAUD CAN_BAUD_500K

#define LEFT_NODE_ID  1
#define RIGHT_NODE_ID 2

/* AIMOTOR lift drive external setup required before use:
 * - H02-00 = 8 for CANopen control mode.
 * - Node ID = LIFT_NODE_ID, recommended 3.
 * - CAN baud = 500 kbps. Change node ID/baud with AIMOTOR RS485 config software.
 */
#define LIFT_NODE_ID                 3U
#define LIFT_CAN_BAUD                500000U

#define LIFT_STROKE_MM               450.0f
#define LIFT_DOWN_MM                 0.0f
#define LIFT_UP_MM                   450.0f

#define LIFT_SCREW_LEAD_MM           5.0f
#define LIFT_REDUCER_RATIO           30.0f

#define LIFT_PULSES_PER_MOTOR_REV    1000.0f  /* TODO: confirm AIMOTOR H05-07/H05-09; drawing encoder says 1024 */
#define LIFT_PULSES_PER_MM           ((LIFT_PULSES_PER_MOTOR_REV * LIFT_REDUCER_RATIO) / LIFT_SCREW_LEAD_MM)

#define LIFT_DOWN_PUL                0
#define LIFT_UP_PUL                  ((int32_t)(LIFT_UP_MM * LIFT_PULSES_PER_MM))

/* Intentional Lift profile experiment. Normal motion and homing deliberately
 * share the derived profile values; the physical mm/s scaling is uncalibrated.
 */
#define LIFT_TEST_SPEED_MM_S         2.0f
#define LIFT_NORMAL_SPEED_MM_S       1200.0f
#define LIFT_ACCEL_MM_S2             3000.0f
#define LIFT_DECEL_MM_S2             3000.0f

/* AIMOTOR/CiA402 PP mode uses Pul/s for 0x6081 and Pul/s^2 for 0x6083/0x6084. */
#define LIFT_PROFILE_VELOCITY_PUL_S  ((uint32_t)(LIFT_NORMAL_SPEED_MM_S * LIFT_PULSES_PER_MM))
#define LIFT_PROFILE_ACCEL_PUL_S2    ((uint32_t)(LIFT_ACCEL_MM_S2 * LIFT_PULSES_PER_MM))
#define LIFT_PROFILE_DECEL_PUL_S2    ((uint32_t)(LIFT_DECEL_MM_S2 * LIFT_PULSES_PER_MM))

#define LIFT_TEST_SPEED_PUL_S        ((int32_t)(LIFT_TEST_SPEED_MM_S * LIFT_PULSES_PER_MM))
#define LIFT_NORMAL_SPEED_PUL_S      ((int32_t)(LIFT_NORMAL_SPEED_MM_S * LIFT_PULSES_PER_MM))
#define LIFT_ACCEL_PUL_S2            ((int32_t)(LIFT_ACCEL_MM_S2 * LIFT_PULSES_PER_MM))
#define LIFT_DECEL_PUL_S2            ((int32_t)(LIFT_DECEL_MM_S2 * LIFT_PULSES_PER_MM))

#define LIFT_POS_TOL_MM              1.0f
#define LIFT_MOVE_TIMEOUT_MS         60000U
#define LIFT_STATUS_PERIOD_MS        100U
/* Lift homing must be explicitly commanded. The drive may report stale 0x6064
 * after power cycle, so boot must not trust it or auto-move.
 */
#define LIFT_AUTO_ZERO_ON_BOOT             0U
#define LIFT_AUTO_PARK_ON_BOOT             0U
#define LIFT_HOMING_POLL_MS                50U
#define LIFT_HOMING_TIMEOUT_MS             90000U
#define LIFT_CIA402_STATE_TIMEOUT_MS       2000U
#define LIFT_CIA402_STATE_POLL_MS          20U
#define LIFT_HOMING_MODE_DISPLAY_TIMEOUT_MS 3000U
#define LIFT_HOMING_MODE_POLL_MS           20U
#define LIFT_HOMING_START_TIMEOUT_MS       1000U
#define LIFT_HOMING_START_POLL_MS          20U
#define LIFT_FAULT_RESET_ASSERT_MS         200U
#define LIFT_FAULT_RESET_SETTLE_MS         100U
/* Installed-model validation must confirm this mode-specific status mask.
 * Bit 13 follows the documented Kinco FD2S homing-error mapping; bit 12 is
 * deliberately not assumed because other Kinco families use it differently.
 */
#define LIFT_HOMING_ERROR_STATUS_MASK      (1U << 13)
#define LIFT_HOMING_WRITE_METHOD           0U
#define LIFT_HOMING_METHOD                 0
/* Service-only slow jog tuning. Jogging is separate from homing and never marks
 * the lift as homed. Keep the default speed at or below 20 mm/s and below 10%
 * of normal profile speed until hardware service procedures say otherwise.
 */
#define LIFT_SERVICE_JOG_SPEED_MM_S        20.0f
#define LIFT_SERVICE_JOG_ACCEL_MM_S2       2200.0f
#define LIFT_SERVICE_JOG_WATCHDOG_MS       500U
#define LIFT_JOG_USE_KINCO_MINUS3_MODE     1U

/* These signs convert logical forward wheel RPM into raw motor RPM sent to each
 * Kinco node. If forward/backward is wrong, flip one sign.
 */
#define LEFT_MOTOR_FORWARD_SIGN  -1
#define RIGHT_MOTOR_FORWARD_SIGN  1

#define MAX_TEST_MOTOR_RPM 2000
#define MAX_TEST_DURATION_MS 30000U
#define UART_COMMAND_LINE_MAX 80U
#define FINAL_CREEP_RPM 10
#define MOVE_TARGET_REACHED_MM 3.0f
#define MOVE_FINAL_CREEP_BAND_MM 10.0f
#define TURN_TARGET_REACHED_DEG 0.75f
#define TURN_FINAL_CREEP_BAND_DEG 2.0f
/* First lifted-wheel qualification profile. Keep the Kinco internal velocity
 * loop and differential wheel geometry, but make STM32 yaw PI/tracking
 * observational so transient feedback quality cannot inhibit basic motion. */
#ifndef DRIVE_BASIC_BRINGUP_PROFILE
#define DRIVE_BASIC_BRINGUP_PROFILE 1U
#endif
#define JETSON_CMD_TIMEOUT_MS 300U
#define JETSON_STATUS_PERIOD_MS 100U
#define DRIVE_CONTROL_PERIOD_MS 25U
#define DRIVE_CAN_REFRESH_MS 1000U
#define DRIVE_ENABLE_RETRY_MS 1000U
#define DRIVE_SPEED_FEEDBACK_PERIOD_MS 200U
#define DRIVE_CMD_VEL_SUMMARY_MS 1000U
/* In Kinco 6060 = -3 mode, zero-speed hold is being tested as the normal
 * idle behavior. Explicit DISABLE remains available for long-term parking or
 * fault-safe shutdown.
 */
#define DRIVE_ZERO_HOLD_ON_IDLE 1
#define DRIVE_IDLE_AUTO_DISABLE_ENABLE 0
#define DRIVE_IDLE_AUTO_DISABLE_MS 1000U
#define DRIVE_KINCO_VELOCITY_MODE ((int8_t)-3)
#define LINEAR_ACCEL_LIMIT_MM_S2 1000.0f
#define LINEAR_DECEL_LIMIT_MM_S2 1000.0f
#define ANGULAR_ACCEL_LIMIT_MRAD_S2 500.0f

/* Wheel-encoder yaw-rate loop. These gains, zero-mode thresholds/deadband and
 * correction limits are deliberately conservative commissioning values; all
 * require representative loaded-vehicle tuning. The controller uses the
 * final feasible CMD_VEL angular target and falls back to the same open-loop
 * differential-drive target whenever coherent feedback is unavailable. */
#if DRIVE_BASIC_BRINGUP_PROFILE
#define DRIVE_YAW_RATE_PI_DEFAULT_ENABLED         0U
#else
#define DRIVE_YAW_RATE_PI_DEFAULT_ENABLED         1U
#endif
#define DRIVE_YAW_RATE_KP                         0.15f
#define DRIVE_YAW_RATE_KI                         0.10f
#define DRIVE_YAW_RATE_TRACKING_GAIN              1.00f
#define DRIVE_YAW_RATE_INTEGRAL_LIMIT_MRAD        800.0f
#define DRIVE_YAW_RATE_CORRECTION_LIMIT_MRAD_S    300.0f
#define DRIVE_YAW_RATE_ZERO_THRESHOLD_MRAD_S      20.0f
#define DRIVE_YAW_RATE_ZERO_EXIT_MRAD_S            40.0f
#define DRIVE_ZERO_YAW_KP                          0.10f
#define DRIVE_ZERO_YAW_CORRECTION_LIMIT_MRAD_S     60.0f
#define DRIVE_ZERO_YAW_FEEDBACK_DEADBAND_MRAD_S    10.0f
#define DRIVE_ZERO_YAW_STOP_TARGET_MRAD_S           1.0f
#define DRIVE_ZERO_YAW_STOP_LINEAR_MM_S            5.0f
#define DRIVE_ZERO_YAW_STOP_WHEEL_MM_S             5.0f
#define DRIVE_YAW_RATE_REENTRY_VALID_UPDATES      1U
#define DRIVE_YAW_RATE_MIN_DT_MS                   10U
#define DRIVE_YAW_RATE_MAX_DT_MS                   200U
#define DRIVE_ACTUATOR_RATE_MAX_DT_MS              200U

/* Persistent yaw tracking supervision. Large feasible-command error first
 * degrades diagnostics; confirmed wrong-sign or monotonically diverging
 * response requests a controlled stop. All counters advance only on a new
 * coherent left/right feedback pair. */
#define DRIVE_TRACKING_COMMAND_STEP_MRAD_S         200.0f
#define DRIVE_TRACKING_ACTIVE_TARGET_MRAD_S        200.0f
#define DRIVE_TRACKING_LARGE_ERROR_MRAD_S          400.0f
#define DRIVE_TRACKING_WRONG_SIGN_ACTUAL_MRAD_S    100.0f
#define DRIVE_TRACKING_DIVERGENCE_RISE_MRAD_S      50.0f
#define DRIVE_TRACKING_SUBMITTED_STEP_MRAD_S        25.0f
#define DRIVE_TRACKING_GRACE_MS                    500U
#define DRIVE_TRACKING_LARGE_ERROR_DURATION_MS     1000U
#define DRIVE_TRACKING_SATURATION_DURATION_MS      1500U
#define DRIVE_TRACKING_LARGE_ERROR_CONSECUTIVE     5U
#define DRIVE_TRACKING_WRONG_SIGN_CONSECUTIVE      3U
#define DRIVE_TRACKING_DIVERGENCE_CONSECUTIVE      4U
#define DRIVE_TRACKING_RECOVERY_CONSECUTIVE        3U

/* SDO 0x606C is available only during safe boot initialization. Normal Drive
 * startup automatically configures and qualifies both TPDO streams; the
 * production runtime feedback authority is TPDO. */
#define DRIVE_WHEEL_FEEDBACK_POLL_MS              50U
#define DRIVE_STOP_FEEDBACK_POLL_MS               50U
#define DRIVE_FAULT_FEEDBACK_POLL_MS             100U
#define DRIVE_WHEEL_POSITION_POLL_MS              100U
#define DRIVE_WHEEL_FEEDBACK_REQUEST_TIMEOUT_MS   20U
#define DRIVE_WHEEL_FEEDBACK_STALE_MS             150U
#define DRIVE_WHEEL_FEEDBACK_MAX_SKEW_MS          30U
#define DRIVE_RUNTIME_STATUSWORD_POLL_MS          100U
#define DRIVE_RUNTIME_TARGET_TIMEOUT_MS           20U
#define DRIVE_SDO_QUARANTINE_MS                    50U
#define DRIVE_STOP_CONFIRM_TIMEOUT_MS             500U
#define DRIVE_NORMAL_ZERO_SETTLE_TIMEOUT_MS       1000U
#define DRIVE_NORMAL_ZERO_CONFIRM_TIMEOUT_MS       500U
#define DRIVE_NORMAL_ZERO_OVERALL_TIMEOUT_MS      2000U
/* Commission together with the installed drive's 0x606C cadence/noise. Three
 * 50 ms samples span the 100 ms dwell and fit inside the confirmation
 * deadline. These shared deadlines cover normal STOP and Lift base-stop
 * qualification; emergency/disable cleanup retains its stricter policy. */
#define DRIVE_STOP_ZERO_STABLE_SAMPLE_COUNT          3U
#define DRIVE_STOP_ZERO_DWELL_MS                    100U
#define DRIVE_STOP_WHEEL_THRESHOLD_MM_S             5.0f
#define DRIVE_DISABLE_STATUS_POLL_MS               20U
#define DRIVE_DISABLE_STATUS_TIMEOUT_MS           500U
#define DRIVE_DISABLE_STATUS_MAX_ATTEMPTS          25U
#define DRIVE_SAFETY_SDO_TIMEOUT_MS                 20U
#define DRIVE_SAFETY_RETRY_MS                       20U
#define DRIVE_SAFETY_SDO_MAX_ATTEMPTS                3U
#define DRIVE_ENABLE_STEP_SHORT_DELAY_MS            200U
#define DRIVE_ENABLE_STEP_LONG_DELAY_MS             300U
#define DRIVE_ENABLE_TRANSITION_TIMEOUT_MS         6000U
#define DRIVE_DISABLE_TRANSITION_TIMEOUT_MS        1500U
/* Startup no-response recovery only; never re-arm after boot completion. */
#define DRIVE_BOOT_RETRY_DELAY_MS                  5000U
#define DRIVE_BOOT_MAX_RETRIES                        1U
#if DRIVE_BOOT_MAX_RETRIES != 1U
#error "Boot recovery V1 permits exactly one automatic retry"
#endif
#define DRIVE_ROLLBACK_TRANSITION_TIMEOUT_MS       1500U
#define DRIVE_ROLLBACK_ZERO_ACK_TIMEOUT_MS          100U

/* Commissioning policy: small command jitter keeps PI continuity. A zero-mode
 * entry, clear angular reversal, or larger angular retarget resets it. */
#define DRIVE_CMD_CHANGE_LINEAR_EPSILON_MM_S          0.1f
#define DRIVE_CMD_CHANGE_ANGULAR_EPSILON_MRAD_S       0.5f
#define DRIVE_YAW_PI_RETARGET_RESET_STEP_MRAD_S     200.0f
#define DRIVE_YAW_PI_RETARGET_SIGN_MIN_MRAD_S        40.0f

/* Node-3 cooperative start sequencing. Validate on the installed lift drive. */
#define LIFT_ACTION_SDO_TIMEOUT_MS                    20U
#define LIFT_ACTION_CIA402_STEP_DELAY_MS              20U
/* Node-3 statusword supervision detects loss of Lift power in idle and motion.
 * Three consecutive misses reject a single transient CAN timeout. */
#define LIFT_HEALTH_POLL_MS                           500U
#define LIFT_HEALTH_MISSED_LIMIT                        3U
#define DRIVE_CANOPEN_RX_BUDGET_FRAMES            8U

/* Wheel feedback TPDO1 commissioning profile. These names are deliberately
 * TPDO-specific; the separate RPDO2 profile is in wheel_pdo_profile.h. */
#define DRIVE_WHEEL_TPDO_LEFT_COB_ID             0x181U
#define DRIVE_WHEEL_TPDO_RIGHT_COB_ID            0x182U
#define DRIVE_WHEEL_TPDO_COMM_INDEX             0x1800U
#define DRIVE_WHEEL_TPDO_MAP_INDEX              0x1A00U
#define DRIVE_WHEEL_TPDO_MAP_ENTRY          0x606C0020UL
#define DRIVE_WHEEL_TPDO_MAP_COUNT                  1U
#define DRIVE_WHEEL_TPDO_TRANSMISSION_TYPE         0xFEU
#define DRIVE_WHEEL_TPDO_INHIBIT_TIME                 0U
#define DRIVE_WHEEL_TPDO_EVENT_TIMER_MS              20U
#define DRIVE_WHEEL_TPDO_EXPECTED_DLC                  4U
#define DRIVE_WHEEL_TPDO_CONFIG_SDO_TIMEOUT_MS       500U
#define DRIVE_WHEEL_TPDO_ACTIVE_ARM_TIMEOUT_MS       1000U

#define WHEEL_DIAMETER_MM 150.0f
#define GEAR_RATIO 10.0f
#define ENCODER_RESOLUTION 65536.0f

// Distance between left/right drive wheel contact centerlines.
// From CAD: center to inner wheel edge = 234.25 mm,
// wheel width = 40 mm, so half width = 20 mm.
// Track width = 2 * (234.25 + 20) = 508.5 mm.
#define WHEEL_BASE_MM 508.5f
#define ODOM_PI 3.14159265358979323846f

/* Body-command layer. These derived limits match the existing 500 motor-RPM
 * envelope for a pure translation or pure rotation. Combined commands still
 * pass through coupled wheel scaling below this layer. */
#define DRIVE_BODY_MAX_LINEAR_MM_S \
    (((float)MAX_TEST_MOTOR_RPM / GEAR_RATIO) * ODOM_PI * \
     WHEEL_DIAMETER_MM / 60.0f)
#define DRIVE_BODY_MAX_ANGULAR_MRAD_S \
    ((2.0f * DRIVE_BODY_MAX_LINEAR_MM_S / WHEEL_BASE_MM) * 1000.0f)

/* ------------------------------------------------------------------------
 * IMU (ISM330DHCX, SPI4) and Battery (BMS, Modbus RTU over USART2/RS485_1)
 * v1 integration. These subsystems are additive and must not affect
 * drive/lift/CANopen/Jetson behavior. See docs/system_truth.md and
 * docs/protocol_contract.md.
 * ------------------------------------------------------------------------ */
#define APP_ENABLE_IMU               1
#define APP_ENABLE_BATTERY           1

/* PD15 / INPUTF: external inversion makes LOW normal, HIGH pressed.
 * Telemetry only; no Drive/Lift control-policy dependency. */
#define ESTOP_RELEASE_DEBOUNCE_MS       30U
#define ESTOP_EVENT_MIN_INTERVAL_MS     50U
#define ESTOP_PUBLISH_PERIOD_MS        100U

/* IMU timing/config (targets, not measured). */
#define IMU_ENABLE_TIMESTAMP         0       /* 0 keeps imu_ts = 0 in v1 */
#define IMU_UPDATE_PERIOD_MS         10U
/* IMU is published at the sample rate: 10 ms / 100 Hz over the 460800-baud
 * USART3 Jetson link (raised from 50 ms / 20 Hz). This is the sole knob that
 * controls the IMU publish cadence; the IMU_SendJetsonStatus_50ms() name is
 * historical. It does not affect FDCAN/CANopen, drive, lift, or battery. */
#define IMU_PUBLISH_PERIOD_MS        10U
#define IMU_SPI_TIMEOUT_MS           2U
#define IMU_POWER_WAIT_MS            50U
#define IMU_RECOVER_PERIOD_MS        1000U
#define IMU_DEGRADED_AGE_MS          100U
#define IMU_STALE_AGE_MS             500U
#define IMU_DEGRADE_FAIL_LIMIT       3U

/* Battery Modbus RTU read-only. This BMS PDF uses FC03 read-data registers over
 * Modbus RTU (one live block covering AI registers 18..40); FC06 MOS writes are
 * defined by the PDF but remain prohibited in runtime polling. */
#define BATTERY_LIVE_POLL_MS            1000U   /* single FC03 live-AI poll cadence */
#define BATTERY_MIN_REQUEST_INTERVAL_MS 550U    /* > vendor-recommended 500 ms */
#define BATTERY_RESPONSE_TIMEOUT_MS     200U    /* 51-byte reply @9600 ~= 53 ms */
#define BATTERY_TX_TIMEOUT_MS           50U     /* bounded TX (8B @9600 ~= 8.3 ms) */
#define BATTERY_OFFLINE_FAILURE_LIMIT   3U
/* Slave address per PDF = configured serial number + 1. Default 1 for now;
 * hardware must confirm the serial number / resulting address. */
#define BATTERY_MODBUS_SLAVE_ADDR       1U

#endif /* APP_CONFIG_H */
