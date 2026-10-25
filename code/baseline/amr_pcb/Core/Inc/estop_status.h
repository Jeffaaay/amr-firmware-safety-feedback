#ifndef ESTOP_STATUS_H
#define ESTOP_STATUS_H

/* Physical PD15 telemetry only. Call after GPIO/JetsonTx init, then once per
 * cooperative loop before JetsonTx_Process(). Never call from an ISR.
 * This module does not change Drive/Lift stop, enable or recovery policy. */
void EstopStatus_Init(void);
void EstopStatus_Update(void);

#endif /* ESTOP_STATUS_H */
