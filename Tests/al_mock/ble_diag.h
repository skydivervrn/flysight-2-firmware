#ifndef BLE_DIAG_H_
#define BLE_DIAG_H_

/* ActiveLook FSM host tests do not exercise the firmware's persistent BLE
 * diagnostic sink. Compile lifecycle trace calls away. */
#define FS_BleDiag_Log(...) ((void)0)

#endif
