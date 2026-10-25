#ifndef STATUS_LIGHT_H
#define STATUS_LIGHT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* Main-context only. Initialize once at boot before UART command processing. */
void StatusLight_Init(void);

/* Store the latest valid Jetson request without writing GPIOs. Faults never
 * clear this request; only initialization resets it to the green default. */
void StatusLight_RequestRgb(bool red, bool green, bool blue);

/* Single runtime output owner: Drive/Lift fault -> RED, otherwise the latest
 * request, otherwise GREEN. Call after the Drive and Lift supervisors. */
void StatusLight_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* STATUS_LIGHT_H */
