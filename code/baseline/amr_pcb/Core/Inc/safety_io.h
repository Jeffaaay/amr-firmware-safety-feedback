#ifndef SAFETY_IO_H
#define SAFETY_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* All three RGB channels are active-low. Initialization selects green. */
void SafetyIo_Init(void);

/* Physical RGB outputs only: true = ON, false = OFF. Active-low polarity
 * is handled here; status/fault policy belongs to StatusLight. */
void SafetyIo_SetRgb(bool red, bool green, bool blue);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_IO_H */
