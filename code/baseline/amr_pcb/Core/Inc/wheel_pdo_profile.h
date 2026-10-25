#ifndef WHEEL_PDO_PROFILE_H
#define WHEEL_PDO_PROFILE_H

/* Candidate profile, not an installed-model compatibility assertion.
 * Every boot requires bilateral PDO readback and TPDO qualification.
 * Device identity is NOT_QUERIED; 0x1018 is not a configuration dependency. */

#define WHEEL_RPDO_COMM_INDEX 0x1401U
#define WHEEL_RPDO_MAP_INDEX 0x1601U
#define WHEEL_RPDO_MAP_ENTRY 0x60FF0020UL
#define WHEEL_RPDO_TYPE 0xFEU
#define WHEEL_RPDO_LEFT_COB_ID 0x301U
#define WHEEL_RPDO_RIGHT_COB_ID 0x302U
#endif
