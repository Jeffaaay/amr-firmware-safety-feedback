/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "drive.h"
#include "lift.h"
#include "canopen.h"
#include "diagnostics.h"
#include "jetson_protocol.h"
#include "jetson_tx.h"
#include "odom.h"
#include "imu.h"
#include "battery.h"
#include "can_trace.h"
#include "firmware_identity.h"
#include "status_light.h"
#include "estop_status.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t buf[UART_RX_RING_SIZE];
} UartRxRing_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

DMA_HandleTypeDef hdma_usart3_rx;

SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#if ENABLE_JETSON_UART_MODE
static UartRxRing_t s_console_rx_ring;
static volatile uint32_t s_console_rx_overflow_count = 0U;
static volatile bool s_console_rx_overflow_pending = false;
static volatile bool s_console_rx_error_pending = false;
static char s_console_line[UART_COMMAND_LINE_MAX];
static uint32_t s_console_len = 0U;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Read-only recorder seams are intentionally kept out of the controlled
 * Drive/Lift command APIs. */
extern uint32_t Drive_GetStateForDiagnostics(void);
extern uint32_t Lift_GetStateForDiagnostics(void);

static void COM1_Init_For_Log(void)
{
    /* PCB project has no Nucleo COM1/BSP console. Debug logging is disabled
     * by default so the Jetson protocol UART only carries ODOM/LIFT lines. */
}

#if ENABLE_JETSON_UART_MODE
static bool UartRing_Pop(UartRxRing_t *ring, uint8_t *ch)
{
    if (ring->tail == ring->head) {
        return false;
    }

    *ch = ring->buf[ring->tail];
    ring->tail = (uint16_t)((ring->tail + 1U) % UART_RX_RING_SIZE);
    return true;
}

static void UartRx_PrintPendingWarnings(void)
{
    static uint32_t last_overflow_warn_ms = 0U;
    uint32_t now = HAL_GetTick();

    if (s_console_rx_overflow_pending &&
        ((now - last_overflow_warn_ms) >= 1000U)) {
        last_overflow_warn_ms = now;
    } else {
        goto print_errors;
    }

    if (s_console_rx_overflow_pending) {
        s_console_rx_overflow_pending = false;
        uart_printf("UART RX overflow on COM1 count=%lu\r\n",
                    (unsigned long)s_console_rx_overflow_count);
    }

print_errors:
    if (s_console_rx_error_pending) {
        s_console_rx_error_pending = false;
        uart_printf("UART RX error on COM1\r\n");
    }

    JetsonProtocol_PrintPendingWarnings();
}
#endif

static const char *can_diag_baud_name(void)
{
#if CAN_DIAG_BAUD == CAN_BAUD_1M
    return "1 Mbps";
#elif CAN_DIAG_BAUD == CAN_BAUD_500K
    return "500 kbps";
#elif CAN_DIAG_BAUD == CAN_BAUD_250K
    return "250 kbps";
#else
    return "unsupported";
#endif
}

static void FDCAN_Init_AcceptAll(FDCAN_HandleTypeDef *hfdcan, const char *mode_label)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
        uart_printf("%s filter config failed\r\n", FDCAN_HandleName(hfdcan));
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        uart_printf("%s global filter failed\r\n", FDCAN_HandleName(hfdcan));
        Error_Handler();
    }

    if ((HAL_FDCAN_ConfigTimestampCounter(hfdcan, FDCAN_TIMESTAMP_PRESC_1) != HAL_OK) ||
        (HAL_FDCAN_EnableTimestampCounter(hfdcan, FDCAN_TIMESTAMP_INTERNAL) != HAL_OK)) {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        uart_printf("%s start failed\r\n", FDCAN_HandleName(hfdcan));
        Error_Handler();
    }

    uart_printf("%s started, baud=%s, mode=%s\r\n",
                FDCAN_HandleName(hfdcan),
                can_diag_baud_name(),
                mode_label);
}

static void CAN1_Init_AcceptAll(void)
{
    FDCAN_Init_AcceptAll(&hfdcan1, "Kinco SDO enable test");
}

static bool Parse_Rpm_Duration(const char *line,
                               const char *cmd,
                               int32_t *rpm,
                               uint32_t *duration_ms)
{
    char parsed_cmd[16] = {0};
    long rpm_in = 0;
    long duration_in = 0;

    if (sscanf(line, "%15s %ld %ld", parsed_cmd, &rpm_in, &duration_in) != 3) {
        uart_printf("Usage: %s <rpm> <ms>\r\n", cmd);
        return false;
    }

    if (duration_in < 0) {
        uart_printf("Duration must be >= 0 ms\r\n");
        return false;
    }

    *rpm = Clamp_TestMotorRPM((int32_t)rpm_in);
    *duration_ms = Clamp_TestDurationMS((uint32_t)duration_in);
    return true;
}

static bool Parse_Raw_Command(const char *line,
                              int32_t *left_rpm,
                              int32_t *right_rpm,
                              uint32_t *duration_ms)
{
    char parsed_cmd[16] = {0};
    long left_in = 0;
    long right_in = 0;
    long duration_in = 0;

    if (sscanf(line, "%15s %ld %ld %ld", parsed_cmd, &left_in, &right_in, &duration_in) != 4) {
        uart_printf("Usage: raw <left_rpm> <right_rpm> <ms>\r\n");
        return false;
    }

    if (duration_in < 0) {
        uart_printf("Duration must be >= 0 ms\r\n");
        return false;
    }

    *left_rpm = Clamp_TestMotorRPM((int32_t)left_in);
    *right_rpm = Clamp_TestMotorRPM((int32_t)right_in);
    *duration_ms = Clamp_TestDurationMS((uint32_t)duration_in);
    return true;
}

static bool Parse_FloatToken(const char *text, float *out_value)
{
    const char *p = text;
    float value = 0.0f;
    float frac_scale = 0.1f;
    int32_t sign = 1;
    bool saw_digit = false;

    if (*p == '-') {
        sign = -1;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while ((*p >= '0') && (*p <= '9')) {
        value = (value * 10.0f) + (float)(*p - '0');
        saw_digit = true;
        p++;
    }

    if (*p == '.') {
        p++;
        while ((*p >= '0') && (*p <= '9')) {
            value += (float)(*p - '0') * frac_scale;
            frac_scale *= 0.1f;
            saw_digit = true;
            p++;
        }
    }

    if ((*p != '\0') || !saw_digit) {
        return false;
    }

    *out_value = (float)sign * value;
    return true;
}

static bool Parse_Target_Rpm_Command(const char *line,
                                     const char *cmd,
                                     const char *target_name,
                                     float *target,
                                     int32_t *max_rpm)
{
    char parsed_cmd[16] = {0};
    char target_text[24] = {0};
    long rpm_in = 0;

    if (sscanf(line, "%15s %23s %ld", parsed_cmd, target_text, &rpm_in) != 3) {
        uart_printf("Usage: %s <%s> <max_rpm>\r\n", cmd, target_name);
        return false;
    }

    if (!Parse_FloatToken(target_text, target)) {
        uart_printf("Invalid %s '%s'\r\n", target_name, target_text);
        return false;
    }

    *max_rpm = Clamp_ClosedLoopMaxRPM((int32_t)rpm_in);
    return true;
}

static bool Parse_CmdVel_Command(const char *line,
                                 const char *cmd,
                                 float *linear_mm_s,
                                 float *angular_mrad_s)
{
    char parsed_cmd[16] = {0};
    char linear_text[24] = {0};
    char angular_text[24] = {0};

    if (sscanf(line, "%15s %23s %23s", parsed_cmd, linear_text, angular_text) != 3) {
        uart_printf("Usage: %s <linear_mm_s> <angular_mrad_s>\r\n", cmd);
        return false;
    }

    if (!Parse_FloatToken(linear_text, linear_mm_s)) {
        uart_printf("Invalid linear_mm_s '%s'\r\n", linear_text);
        return false;
    }

    if (!Parse_FloatToken(angular_text, angular_mrad_s)) {
        uart_printf("Invalid angular_mrad_s '%s'\r\n", angular_text);
        return false;
    }

    return true;
}

static void __attribute__((unused)) UART_ReadLine_Blocking(char *line, uint32_t max_len)
{
    (void)max_len;
    line[0] = '\0';
}

static void Drive_Console_Execute(const char *line)
{
    char cmd[16] = {0};
    int32_t rpm = 0;
    int32_t left_rpm = 0;
    int32_t right_rpm = 0;
    float target = 0.0f;
    float linear_mm_s = 0.0f;
    float angular_mrad_s = 0.0f;
    uint32_t duration_ms = 0U;

    uart_printf("CONSOLE EXEC: '%s'\r\n", line);

    if (sscanf(line, "%15s", cmd) != 1) {
        return;
    }

    if (strcmp(cmd, "help") == 0) {
        Drive_PrintHelp();
    } else if (strcmp(cmd, "enable") == 0) {
        (void)Drive_CommandEnable();
    } else if (strcmp(cmd, "disable") == 0) {
        Drive_CommandDisable();
    } else if (strcmp(cmd, "stop") == 0) {
        Drive_CommandStop();
    } else if (strcmp(cmd, "status") == 0) {
        Drive_PrintFeedback();
    } else if (strcmp(cmd, "odom") == 0) {
        (void)Odom_UpdateAndPrint();
    } else if (strcmp(cmd, "odom_reset") == 0) {
        (void)Odom_Reset();
    } else if (strcmp(cmd, "fwd") == 0) {
        if (Parse_Rpm_Duration(line, "fwd", &rpm, &duration_ms)) {
            Drive_Console_MoveBoth(rpm, rpm, duration_ms);
        }
    } else if (strcmp(cmd, "back") == 0) {
        if (Parse_Rpm_Duration(line, "back", &rpm, &duration_ms)) {
            Drive_Console_MoveBoth(-rpm, -rpm, duration_ms);
        }
    } else if (strcmp(cmd, "spin") == 0) {
        if (Parse_Rpm_Duration(line, "spin", &rpm, &duration_ms)) {
            Drive_Console_MoveBoth(rpm, -rpm, duration_ms);
        }
    } else if (strcmp(cmd, "raw") == 0) {
        if (Parse_Raw_Command(line, &left_rpm, &right_rpm, &duration_ms)) {
            Drive_Console_MoveBoth(left_rpm, right_rpm, duration_ms);
        }
    } else if (strcmp(cmd, "fwd_odom") == 0) {
        if (Parse_Rpm_Duration(line, "fwd_odom", &rpm, &duration_ms)) {
            Drive_Console_MoveBothOdom(rpm, rpm, duration_ms);
        }
    } else if (strcmp(cmd, "spin_odom") == 0) {
        if (Parse_Rpm_Duration(line, "spin_odom", &rpm, &duration_ms)) {
            Drive_Console_MoveBothOdom(rpm, -rpm, duration_ms);
        }
    } else if (strcmp(cmd, "raw_odom") == 0) {
        if (Parse_Raw_Command(line, &left_rpm, &right_rpm, &duration_ms)) {
            Drive_Console_MoveBothOdom(left_rpm, right_rpm, duration_ms);
        }
    } else if (strcmp(cmd, "move_mm") == 0) {
        if (Parse_Target_Rpm_Command(line, "move_mm", "distance_mm", &target, &rpm)) {
            Drive_MoveMM_ClosedLoop(target, rpm);
        }
    } else if (strcmp(cmd, "turn_deg") == 0) {
        if (Parse_Target_Rpm_Command(line, "turn_deg", "angle_deg", &target, &rpm)) {
            Drive_TurnDeg_ClosedLoop(target, rpm);
        }
    } else if (strcmp(cmd, "cmd_vel") == 0) {
        if (Parse_CmdVel_Command(line, "cmd_vel", &linear_mm_s, &angular_mrad_s)) {
            uart_printf("CONSOLE CMD_VEL linear=%ld angular=%ld\r\n",
                        (long)linear_mm_s,
                        (long)angular_mrad_s);
            Drive_ConsoleApplyCmdVel(linear_mm_s, angular_mrad_s);
        }
    } else if (strcmp(cmd, "node1") == 0) {
        if (Parse_Rpm_Duration(line, "node1", &rpm, &duration_ms)) {
            Drive_Console_MoveNode(LEFT_NODE_ID, rpm, duration_ms);
        }
    } else if (strcmp(cmd, "node2") == 0) {
        if (Parse_Rpm_Duration(line, "node2", &rpm, &duration_ms)) {
            Drive_Console_MoveNode(RIGHT_NODE_ID, rpm, duration_ms);
        }
    } else {
        uart_printf("Unknown command '%s'. Type help.\r\n", cmd);
    }
}

#if ENABLE_JETSON_UART_MODE
#if ENABLE_DANGEROUS_RAW_TEST
static void Jetson_RunRawCommand(int32_t left_rpm,
                                 int32_t right_rpm,
                                 uint32_t duration_ms)
{
    if (Lift_IsMoving()) {
        uart_printf("JETSON RAW rejected: lift is moving\r\n");
        return;
    }

    uart_printf("JETSON RAW left=%ld right=%ld duration=%lu\r\n",
                (long)left_rpm,
                (long)right_rpm,
                (unsigned long)duration_ms);

    Drive_ClearMotionCommand();

    if (!Drive_EnsureEnabled()) {
        uart_printf("JETSON RAW rejected: Drive_EnsureEnabled failed\r\n");
        return;
    }

    if (!Drive_SetMotorRPM(left_rpm, right_rpm)) {
        uart_printf("JETSON RAW failed: Drive_SetMotorRPM failed\r\n");
        Drive_StopBoth();
        return;
    }

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < duration_ms) {
        JetsonTx_Process();
        Drive_SendJetsonStatus_100ms();
        Lift_Update();
        Lift_SendJetsonStatus_100ms();
        HAL_Delay(1);
    }

    Drive_StopBoth();
}
#endif

static void Console_PollLine(void)
{
    uint8_t ch = 0U;

    UartRx_PrintPendingWarnings();

    while (UartRing_Pop(&s_console_rx_ring, &ch)) {
        if ((ch == '\r') || (ch == '\n')) {
            if (s_console_len > 0U) {
                s_console_line[s_console_len] = '\0';
                Drive_Console_Execute(s_console_line);
                s_console_len = 0U;
            }
            continue;
        }

        if ((ch == 0x08U) || (ch == 0x7FU)) {
            if (s_console_len > 0U) {
                s_console_len--;
            }
            continue;
        }

        if ((ch >= 0x20U) && (ch <= 0x7EU) &&
            (s_console_len < (UART_COMMAND_LINE_MAX - 1U))) {
            s_console_line[s_console_len++] = (char)ch;
        }
    }
}

static void Jetson_PollLine(void)
{
    JetsonProtocol_ServiceRx();
    JetsonProtocol_PollLine();
    /* PollLine can discover invalid bytes, line overflow or assembly timeout
     * in main context. Dispatch their common damage stop in this same loop,
     * before Drive_ControlLoop_Update() or Lift_Update() can run. */
    JetsonProtocol_DispatchPendingRxSafetyStop();
}

#if ENABLE_JETSON_SELF_LOOPBACK_TEST
static void USART3_SelfLoopback_Update(void)
{
    static uint32_t last_tx_ms = 0U;
    uint32_t now = HAL_GetTick();

    if ((now - last_tx_ms) >= 1000U) {
        static const char msg[] = "STM32_LOOP\r\n";
        last_tx_ms = now;
        uart_printf("USART3 TX tick\r\n");
        HAL_StatusTypeDef txst = JetsonTx_Send(JETSON_TX_PRIORITY_DEBUG,
                                               msg,
                                               (uint16_t)(sizeof(msg) - 1U)) ?
                                      HAL_OK : HAL_BUSY;
        uart_printf("USART3 TX status=%d\r\n", txst);
    }
}
#endif
#endif

static void Drive_Console_Run(void)
{
    uart_printf("UART DRIVE TEST CONSOLE READY. Type help.\r\n");
#if ENABLE_BLOCKING_COM1_CONSOLE
    uart_printf("COM1 blocking debug console mode active.\r\n");
#else
    uart_printf("JETSON UART CMD_VEL READY ON USART3 460800.\r\n");
#endif
    uart_printf("WARNING: wheels may move after move commands. Lift robot and keep E-stop ready.\r\n");

#if ENABLE_BLOCKING_COM1_CONSOLE
    while (1) {
        char line[UART_COMMAND_LINE_MAX] = {0};

        JetsonTx_Process();
        uart_printf("drive> ");
        UART_ReadLine_Blocking(line, sizeof(line));
        if (line[0] != '\0') {
            Drive_Console_Execute(line);
        }
    }
#else
    uart_printf("drive> ");

    while (1) {
        JetsonTx_Process();
#if ENABLE_JETSON_UART_MODE
        Console_PollLine();
#if ENABLE_JETSON_SELF_LOOPBACK_TEST
        USART3_SelfLoopback_Update();
#else
        Jetson_PollLine();
        Drive_ControlLoop_Update();
        Drive_SendJetsonStatus_100ms();
        Lift_Update();
        Lift_SendJetsonStatus_100ms();
#endif
#endif
        HAL_Delay(1);
    }
#endif
}

static void __attribute__((unused)) CAN1_Diagnostic_Run(void)
{
    CAN1_DiagStatus_t can_status = {0};

    uart_printf("CAN diagnostic mode: FDCAN kernel assumed 48 MHz\r\n");
    Drive_Console_Run();
    (void)CAN1_Print_Status("after drive test", &can_status);

    while (1) {
        HAL_Delay(1000);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  /* Wheel drives and lift share FDCAN1; FDCAN2 is unused in V3. */
  MX_FDCAN1_Init();
  MX_USART3_UART_Init();
  JetsonTx_Init(&huart3);
  CanTrace_Init(&hfdcan1, HAL_GetTick());
  MX_USART2_UART_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  /* Report the physical input even when the button is already pressed at boot. */
  EstopStatus_Init();
  EstopStatus_Update();
  /* Active-low RGB outputs start green; keep this call on CubeMX regeneration. */
  StatusLight_Init();
  const char *banner0 = "\r\nPCB_UART_ALIVE_BEFORE_CAN\r\n";
  FirmwareIdentity_Print();
  (void)JetsonTx_Send(JETSON_TX_PRIORITY_STATUS,
                      banner0, (uint16_t)strlen(banner0));

  const char *banner1 = "PCB_AFTER_FDCAN1_INIT\r\n";
  (void)JetsonTx_Send(JETSON_TX_PRIORITY_STATUS,
                      banner1, (uint16_t)strlen(banner1));

  COM1_Init_For_Log();

  CAN1_Init_AcceptAll();
  /* Capture wheel hard faults before any diagnostic client can run ahead
   * of the first control iteration. */
  Canopen_BootSdoBegin();
  /* Start the independent CANopen master heartbeat immediately after the
   * shared wheel/lift bus becomes usable, before any wheel can consume it. */
  Canopen_HeartbeatService(HAL_GetTick());
  const char *banner3 = "PCB_AFTER_CAN1_START\r\n";
  (void)JetsonTx_Send(JETSON_TX_PRIORITY_STATUS,
                      banner3, (uint16_t)strlen(banner3));
  /* FDCAN2 unused: lift is now on FDCAN1 node 3. Do not start CAN2. */
#if ENABLE_JETSON_UART_MODE
#if ENABLE_DANGEROUS_RAW_TEST
  JetsonProtocol_SetRawCommandCallback(Jetson_RunRawCommand);
#else
  JetsonProtocol_SetRawCommandCallback(NULL);
#endif
  JetsonProtocol_StartUartRxInterrupts();
#endif

  Drive_SetLiftMovingCallback(Lift_IsMoving);

#if APP_ENABLE_IMU
  IMU_Init();
#endif
#if APP_ENABLE_BATTERY
  Battery_Init();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t loop_start_ms = HAL_GetTick();

    CanTrace_RuntimeBegin(CAN_TRACE_RUNTIME_MAIN_LOOP, loop_start_ms);
    CanTrace_SetSubsystemStates(
        Drive_GetStateForDiagnostics(), Lift_GetStateForDiagnostics());
    EstopStatus_Update();
    JetsonTx_Process();
#if ENABLE_JETSON_UART_MODE
    Jetson_PollLine();
#endif
    CanTrace_RuntimeBegin(CAN_TRACE_RUNTIME_DRIVE_UPDATE, HAL_GetTick());
    Drive_ControlLoop_Update();
    CanTrace_RuntimeEnd(CAN_TRACE_RUNTIME_DRIVE_UPDATE, HAL_GetTick());
    CanTrace_SetSubsystemStates(
        Drive_GetStateForDiagnostics(), Lift_GetStateForDiagnostics());
    Drive_SendJetsonStatus_100ms();
    CanTrace_RuntimeBegin(CAN_TRACE_RUNTIME_LIFT_UPDATE, HAL_GetTick());
    Lift_Update();
    CanTrace_RuntimeEnd(CAN_TRACE_RUNTIME_LIFT_UPDATE, HAL_GetTick());
    CanTrace_SetSubsystemStates(
        Drive_GetStateForDiagnostics(), Lift_GetStateForDiagnostics());
    Lift_SendJetsonStatus_100ms();
    StatusLight_Update();
    uint32_t now_ms = HAL_GetTick();
#if APP_ENABLE_IMU
    IMU_Update(now_ms);
#endif
#if APP_ENABLE_BATTERY
    Battery_Update(now_ms);
#endif
#if APP_ENABLE_IMU
    IMU_SendJetsonStatus_50ms();
#endif
#if APP_ENABLE_BATTERY
    Battery_SendJetsonStatus_1000ms();
#endif
    CanTrace_Service(HAL_GetTick());
    CanTrace_RuntimeEnd(CAN_TRACE_RUNTIME_MAIN_LOOP, HAL_GetTick());
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 6;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 8;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 8;
  hfdcan1.Init.TxBuffersNbr = 2;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_WR2_GPIO_Port, RS485_WR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_WR1_GPIO_Port, RS485_WR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /* Preload the active-low startup color before switching the pins to output. */
  HAL_GPIO_WritePin(LIGHT_RED_GPIO_Port, LIGHT_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LIGHT_GREEN_GPIO_Port, LIGHT_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LIGHT_BLUE_GPIO_Port, LIGHT_BLUE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : IMU_INT1_Pin IMU_INT2_Pin */
  GPIO_InitStruct.Pin = IMU_INT1_Pin|IMU_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_WR2_Pin */
  GPIO_InitStruct.Pin = RS485_WR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_WR2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_WR1_Pin */
  GPIO_InitStruct.Pin = RS485_WR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_WR1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIGHT_RED_Pin */
  GPIO_InitStruct.Pin = LIGHT_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIGHT_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIGHT_GREEN_Pin LIGHT_BLUE_Pin */
  GPIO_InitStruct.Pin = LIGHT_GREEN_Pin|LIGHT_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* PD15 / INPUTF is externally driven through the inversion circuit.
   * LOW = normal, HIGH = emergency-stop pressed. Telemetry only. */
  GPIO_InitStruct.Pin = ESTOP_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ESTOP_INPUT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
