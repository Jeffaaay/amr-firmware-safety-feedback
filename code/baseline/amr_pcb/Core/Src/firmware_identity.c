/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : firmware_identity.c
  * @brief          : Compile-time firmware identity implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "firmware_identity.h"

#include "app_config.h"
#include "jetson_tx.h"

void FirmwareIdentity_Print(void)
{
    (void)JetsonTx_Printf(
        JETSON_TX_PRIORITY_STATUS,
        "FW variant=%s build=%s profile=%s optimization=%s date=\"%s\" time=%s protocol=%u txmgr=%u pi=%u raw_test=%u\r\n",
        FIRMWARE_VARIANT,
        FIRMWARE_BUILD_ID,
        FIRMWARE_BUILD_PROFILE,
        FIRMWARE_BUILD_OPTIMIZATION,
        __DATE__,
        __TIME__,
        (unsigned)FIRMWARE_PROTOCOL_VERSION,
        (unsigned)FIRMWARE_TX_MANAGER_ENABLED,
        (DRIVE_YAW_RATE_PI_DEFAULT_ENABLED != 0U) ? 1U : 0U,
        (ENABLE_DANGEROUS_RAW_TEST != 0) ? 1U : 0U);
}
