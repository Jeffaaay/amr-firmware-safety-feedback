#ifndef COMMAND_RESULT_H
#define COMMAND_RESULT_H

/* Typed accept/reject result for Jetson-facing state-changing commands.
 *
 * CMD_ACK must always report the REAL rejection reason; "accepted=0
 * reason=OK" is forbidden by the protocol contract. The dispatcher maps
 * these values 1:1 onto the reason= token via
 * JetsonProtocol_CommandResultName(). CMD_RESULT_ACCEPTED is 0 so legacy
 * bool wrappers (result == CMD_RESULT_ACCEPTED) stay trivially correct. */
typedef enum
{
    CMD_RESULT_ACCEPTED = 0,
    CMD_RESULT_BAD_REQ,                /* missing/malformed req= nonce */
    CMD_RESULT_BAD_COMMAND,            /* extra tokens / bad line shape */
    CMD_RESULT_TRANSITION_BUSY,        /* safety transition in flight */
    CMD_RESULT_LIFT_ACTIVE,            /* lift interlock blocks command */
    CMD_RESULT_NOT_CONFIRMED_DISABLED, /* hardware-disable evidence missing */
    CMD_RESULT_FAULT_LATCHED,          /* fault/motion-inhibit latch set */
    CMD_RESULT_NOT_ENABLED,            /* command needs an enabled drive */
    CMD_RESULT_BASE_NOT_SAFE,          /* base-safe predicate false */
    CMD_RESULT_OPERATION_ENABLED,      /* node still operation-enabled */
    CMD_RESULT_INTERNAL_ERROR          /* unexpected internal refusal */
} CommandResult_t;

#endif /* COMMAND_RESULT_H */
