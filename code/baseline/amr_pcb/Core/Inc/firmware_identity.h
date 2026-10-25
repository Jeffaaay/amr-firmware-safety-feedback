/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : firmware_identity.h
  * @brief          : Compile-time firmware identity interface.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef FIRMWARE_IDENTITY_H
#define FIRMWARE_IDENTITY_H

#ifdef __cplusplus
extern "C" {
#endif

#define FIRMWARE_VARIANT "AMR_FIRST_SENSOR_HOMING"
/* The TRACKED SOURCE permanently carries the UNSTAMPED identity; no build
 * flow ever edits this header. Real identities are injected on the
 * compiler command line (-DFIRMWARE_BUILD_ID="...") by the build-only
 * BUILD_FIRMWARE flow. It injects a unique UTC-tagged AMR-DEV manual-build
 * identity. clean_build refuses to build without an injected ID, so its ELF
 * can never be UNSTAMPED; a direct IDE build remains visibly UNSTAMPED, and
 * no exit path can leave a stamped identity behind in the source tree. */
#ifndef FIRMWARE_BUILD_ID
#define FIRMWARE_BUILD_ID "UNSTAMPED"
#endif
/* v5: ODOM wheel fields are signed milli-RPM and there is one ODOM grammar.
 * The v4 req=/snap=/gen=/txn= transaction binding remains unchanged. v5
 * hosts must refuse v4 firmware so wheel-field units cannot be confused. */
#define FIRMWARE_PROTOCOL_VERSION 5U
#define FIRMWARE_TX_MANAGER_ENABLED 1U

/* Build profiles. The supported build-only helper selects a profile with
 * -DFIRMWARE_BUILD_PROFILE_ID=<n> and pair it with the matching -O flag:
 *   BENCH_DEBUG  ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â -O0, verbose periodic debug streams enabled.
 *   RELEASE_TEST ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â -Og, periodic CMD/SDO/statusword debug compiled out.
 * FW_VERSION always reports profile= and optimization= so firmwares with
 * different timing behavior are distinguishable on the wire. A hand build
 * without the define is a BENCH_DEBUG build. */
#define FIRMWARE_PROFILE_BENCH_DEBUG 0
#define FIRMWARE_PROFILE_RELEASE_TEST 1
#ifndef FIRMWARE_BUILD_PROFILE_ID
#define FIRMWARE_BUILD_PROFILE_ID FIRMWARE_PROFILE_BENCH_DEBUG
#endif
#if FIRMWARE_BUILD_PROFILE_ID == FIRMWARE_PROFILE_RELEASE_TEST
#define FIRMWARE_BUILD_PROFILE "RELEASE_TEST"
#define FIRMWARE_BUILD_OPTIMIZATION "Og"
#else
#define FIRMWARE_BUILD_PROFILE "BENCH_DEBUG"
#define FIRMWARE_BUILD_OPTIMIZATION "O0"
#endif

void FirmwareIdentity_Print(void);

#ifdef __cplusplus
}
#endif

#endif /* FIRMWARE_IDENTITY_H */
