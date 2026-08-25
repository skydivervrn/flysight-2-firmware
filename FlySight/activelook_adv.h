/***************************************************************************
**  FlySight 2 firmware — ActiveLook advertising parser (PURE, host-testable)
****************************************************************************/
#ifndef ACTIVELOOK_ADV_H_
#define ACTIVELOOK_ADV_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FS_ACTIVELOOK_ADV_NAME_LEN    16u
#define FS_ACTIVELOOK_ADV_SERIAL_LEN   6u

typedef struct
{
    bool found_uuid;
    bool found_name;
    bool have_serial;
    char name[FS_ACTIVELOOK_ADV_NAME_LEN + 1u];
    char serial[FS_ACTIVELOOK_ADV_SERIAL_LEN + 1u];
} FS_ActiveLookAdvResult;

/*
 * Parse one legacy advertising or scan-response payload.
 *
 * Returns true for a structurally valid payload (including an empty one).
 * Returns false for a truncated/malformed AD element and clears every result
 * field, so a valid-looking element before a bad tail cannot be accepted.
 * `data` may be NULL only when `length` is zero.
 */
bool FS_ActiveLookAdv_Parse(const uint8_t *data, size_t length,
                            FS_ActiveLookAdvResult *result);

#endif /* ACTIVELOOK_ADV_H_ */
