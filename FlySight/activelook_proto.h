/***************************************************************************
**  FlySight 2 firmware — ActiveLook protocol helpers (PURE, host-testable)
**
**  These functions contain NO hardware/HAL/BLE dependencies so they can be
**  unit-tested on the host (see Tests/test_activelook.c). The BLE transport
**  (activelook_client.c) and the display logic (activelook_mode0.c) build on
**  top of these.
****************************************************************************/
#ifndef ACTIVELOOK_PROTO_H_
#define ACTIVELOOK_PROTO_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ActiveLook command IDs (subset we use) */
#define AL_CMD_CLEAR          0x01u
#define AL_CMD_BATTERY        0x05u
#define AL_CMD_VERS           0x06u
#define AL_CMD_LUMA           0x10u
#define AL_CMD_COLOR          0x30u
#define AL_CMD_TXT            0x37u
#define AL_CMD_HOLD_FLUSH     0x39u
#define AL_CMD_LAYOUT_SAVE    0x60u
#define AL_CMD_LAYOUT_DISPLAY 0x62u
#define AL_CMD_PAGE_SAVE      0x80u
#define AL_CMD_PAGE_DISPLAY   0x83u
#define AL_CMD_PAGE_CLEARDISP 0x86u
#define AL_CMD_CFG_WRITE      0xD0u
#define AL_CMD_CFG_SET        0xD2u

/* holdFlush actions */
#define AL_HOLD   0x00u
#define AL_FLUSH  0x01u
#define AL_RESET  0xFFu

/* ActiveLook flow-control bytes on the Control characteristic (...CB9) */
#define AL_FLOW_OK            0x01u  /* OK to send                      */
#define AL_FLOW_STOP          0x02u  /* RX buffer >=75%, pause sending  */
#define AL_FLOW_ERR_CMD       0x03u  /* command incomplete/corrupt      */
#define AL_FLOW_ERR_OVERFLOW  0x04u  /* RX queue overflow (engine reset)*/
#define AL_FLOW_ERR_NOCFG     0x06u  /* cfg-modifying cmd w/o cfgWrite  */

/* ActiveLook command-frame overhead: 0xFF cmd fmt len ... 0xAA  (1-byte len) */
#define AL_FRAME_OVERHEAD     5u
#define AL_FRAME_MAX          255u  /* 1-byte length field cap */

/*
 * Build an ActiveLook command frame with a 1-byte length and no QueryID:
 *   0xFF | cmd | 0x00 | total_len | data[datalen] | 0xAA
 * where total_len = datalen + 5 (the whole frame, header+footer included).
 *
 * Writes into `out` (capacity `outcap`). Returns the total frame length, or
 * 0 if it would not fit in `outcap` or exceeds the 255-byte 1-byte-length cap.
 * `data` may be NULL iff `datalen` == 0.
 */
size_t AL_BuildFrame(uint8_t *out, size_t outcap,
                     uint8_t cmd, const uint8_t *data, size_t datalen);

/*
 * Flow-control gate. Returns true only when the link is up AND the last
 * control byte received on ...CB9 permits sending (AL_FLOW_OK). Any STOP/error
 * byte, or link down, returns false.
 */
bool AL_FlowCanSend(uint8_t last_ctrl_byte, bool link_up);

/*
 * Build a complete ActiveLook txt (0x37) command frame:
 *   0xFF 0x37 0x00 len  x(2,BE) y(2,BE) rotation font color  <str> 0x00  0xAA
 * The string is NUL-terminated inside the payload (ActiveLook requires it).
 * Returns the total frame length, or 0 if it won't fit in outcap / exceeds 255.
 * `str` may be NULL (treated as empty).
 */
size_t AL_BuildText(uint8_t *out, size_t outcap,
                    int16_t x, int16_t y,
                    uint8_t rotation, uint8_t font, uint8_t color,
                    const char *str);

/*
 * Battery percentage from millivolts, clamped to [0,100].
 * Linear across [AL_VBAT_EMPTY_MV, AL_VBAT_FULL_MV].
 */
#define AL_VBAT_EMPTY_MV  3300u
#define AL_VBAT_FULL_MV   4200u
uint8_t AL_BatteryPct(uint16_t mv);

#endif /* ACTIVELOOK_PROTO_H_ */
