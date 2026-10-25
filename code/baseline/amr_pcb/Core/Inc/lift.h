/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lift.h
  * @brief          : Lift/top actuator control interfaces.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef LIFT_H
#define LIFT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

bool Lift_IsMoving(void);
/* True after the existing Lift state machine has latched any Lift fault,
 * including node-3 communication loss after external Lift power is removed. */
bool Lift_IsFaulted(void);
void Lift_Update(void);
/* Main-context safety entry point for a command-channel discontinuity.
 * Reuses the normal LIFT_STOP path to cancel pending work and stop motion. */
void Lift_RequestCommunicationStop(void);
void Lift_SendJetsonStatus_100ms(void);
void Lift_SendJetsonStatusNow(void);
bool Lift_HandleCommand(const char *cmd);
/* Protocol v4: the dispatcher stores the validated host req= nonce before
 * calling Lift_HandleCommand so LIFT_FAULT_RESET / LIFT_HOME transaction
 * lines can echo it. */
void Lift_SetCommandRequestNonce(uint32_t request_nonce);

#ifdef AMR_HOST_TEST
void Lift_TestResetState(void);
void Lift_TestSetMoving(uint32_t start_ms);
void Lift_TestSetPendingForEmcy(void);
void Lift_TestSetRecoveryForEmcy(void);
void Lift_TestSetHomed(bool homed, int32_t position_pul);
uint32_t Lift_TestPendingAction(void);
uint32_t Lift_TestPendingPhase(void);
uint32_t Lift_TestState(void);
uint32_t Lift_TestFaultReason(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* LIFT_H */
