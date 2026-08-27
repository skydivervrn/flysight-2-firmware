/***************************************************************************
**  FlySight 2 firmware — ActiveLook protocol helpers (PURE, host-testable)
**
**  No hardware/HAL/BLE dependencies. See activelook_proto.h for API docs.
****************************************************************************/
#include "activelook_proto.h"
#include <string.h>

size_t AL_BuildFrame(uint8_t *out, size_t outcap,
                     uint8_t cmd, const uint8_t *data, size_t datalen)
{
    size_t total = datalen + AL_FRAME_OVERHEAD;   /* datalen + 5 */

    /* Guard: would overflow the 1-byte length field or the caller's buffer */
    if (total > AL_FRAME_MAX || total > outcap) {
        return 0;
    }

    out[0] = 0xFFu;
    out[1] = cmd;
    out[2] = 0x00u;                 /* format: 1-byte length, no QueryID */
    out[3] = (uint8_t)total;        /* whole-frame length incl. header+footer */

    if (datalen > 0 && data != NULL) {
        memcpy(&out[4], data, datalen);
    }

    out[total - 1] = 0xAAu;         /* footer */

    return total;
}

bool AL_FlowCanSend(uint8_t last_ctrl_byte, bool link_up)
{
    return link_up && (last_ctrl_byte == AL_FLOW_OK);
}

size_t AL_BuildText(uint8_t *out, size_t outcap,
                    int16_t x, int16_t y,
                    uint8_t rotation, uint8_t font, uint8_t color,
                    const char *str)
{
    size_t slen = (str != NULL) ? strlen(str) : 0u;

    /* payload = x(2) y(2) rotation(1) font(1) color(1) + str + NUL */
    size_t datalen = 7u + slen + 1u;

    uint8_t data[64 + 8];
    if (datalen > sizeof(data)) {
        return 0;   /* string too long for our scratch buffer */
    }

    data[0] = (uint8_t)(((uint16_t)x >> 8) & 0xFFu);
    data[1] = (uint8_t)((uint16_t)x & 0xFFu);
    data[2] = (uint8_t)(((uint16_t)y >> 8) & 0xFFu);
    data[3] = (uint8_t)((uint16_t)y & 0xFFu);
    data[4] = rotation;
    data[5] = font;
    data[6] = color;
    if (slen > 0u) {
        memcpy(&data[7], str, slen);
    }
    data[7 + slen] = 0x00u;   /* ActiveLook requires the string NUL-terminated */

    return AL_BuildFrame(out, outcap, AL_CMD_TXT, data, datalen);
}

size_t AL_BuildShape(uint8_t *out, size_t outcap, uint8_t cmd,
                     int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
    const int16_t coord[4] = { x0, y0, x1, y1 };
    uint8_t data[8];

    for (int i = 0; i < 4; i++) {
        data[i * 2]     = (uint8_t)(((uint16_t)coord[i] >> 8) & 0xFFu);
        data[i * 2 + 1] = (uint8_t)((uint16_t)coord[i] & 0xFFu);
    }

    return AL_BuildFrame(out, outcap, cmd, data, sizeof(data));
}

uint8_t AL_BatteryPct(uint16_t mv)
{
    if (mv <= AL_VBAT_EMPTY_MV) {
        return 0;
    }
    if (mv >= AL_VBAT_FULL_MV) {
        return 100;
    }

    /* Linear map: (mv - EMPTY) / (FULL - EMPTY) * 100, rounded */
    uint32_t numerator = ((uint32_t)(mv - AL_VBAT_EMPTY_MV) * 100u
                          + (AL_VBAT_FULL_MV - AL_VBAT_EMPTY_MV) / 2u);
    uint32_t result = numerator / (AL_VBAT_FULL_MV - AL_VBAT_EMPTY_MV);

    return (uint8_t)result;
}
