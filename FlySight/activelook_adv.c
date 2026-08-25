/***************************************************************************
**  FlySight 2 firmware — ActiveLook advertising parser (PURE, host-testable)
****************************************************************************/
#include "activelook_adv.h"

#include <string.h>

#define AD_TYPE_128_BIT_SERV_UUID             0x06u
#define AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST  0x07u
#define AD_TYPE_SHORTENED_LOCAL_NAME          0x08u
#define AD_TYPE_COMPLETE_LOCAL_NAME           0x09u

static const uint8_t s_activelookSvcUuid[16] = {
    0xB7, 0x5C, 0x49, 0xD2, 0x04, 0xA3, 0x40, 0x71,
    0xA0, 0xB5, 0x35, 0x85, 0x3E, 0xB0, 0x83, 0x07
};

bool FS_ActiveLookAdv_Parse(const uint8_t *data, size_t length,
                            FS_ActiveLookAdvResult *result)
{
    static const char *prefixes[] = {
        "ENGO", "AL-", "AL ", "ACTIVELOOK", "A.LOOK"
    };
    FS_ActiveLookAdvResult parsed = {0};
    size_t offset = 0;

    if (result == NULL) return false;
    memset(result, 0, sizeof(*result));
    if (data == NULL && length != 0u) return false;

    while (offset < length)
    {
        size_t remaining = length - offset;
        uint8_t adlength;
        uint8_t adtype;

        /* A zero length byte begins the NON-SIGNIFICANT part: the Core spec
         * (Vol 3, Part C, §11) pads the rest of the payload with zeroes, and
         * a controller that reports the padded length hands that padding to
         * us. Stop, and keep whatever the significant part held. The walker
         * this replaced did the same, and calling padding "malformed" would
         * quietly stop real glasses from ever being found. */
        if (data[offset] == 0u) break;

        /* Anything else needs both its length and its type byte. */
        if (remaining < 2u) return false;

        adlength = data[offset];
        if ((size_t)adlength > remaining - 1u)
            return false;

        adtype = data[offset + 1u];

        if (adtype == AD_TYPE_128_BIT_SERV_UUID ||
            adtype == AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST)
        {
            uint8_t uuid_data_len = adlength - 1u;
            const uint8_t *uuid_data = &data[offset + 2u];
            uint8_t ui;

            for (ui = 0; ui + 16u <= uuid_data_len; ui += 16u)
            {
                if (memcmp(&uuid_data[ui], s_activelookSvcUuid, 16u) == 0)
                {
                    parsed.found_uuid = true;
                    break;
                }
            }
        }
        else if (adtype == AD_TYPE_COMPLETE_LOCAL_NAME ||
                 adtype == AD_TYPE_SHORTENED_LOCAL_NAME)
        {
            const uint8_t *name_data = &data[offset + 2u];
            uint8_t name_len = adlength - 1u;

            if (parsed.name[0] == '\0' ||
                adtype == AD_TYPE_COMPLETE_LOCAL_NAME)
            {
                size_t copy_len = name_len < FS_ACTIVELOOK_ADV_NAME_LEN
                    ? name_len : FS_ACTIVELOOK_ADV_NAME_LEN;
                memcpy(parsed.name, name_data, copy_len);
                parsed.name[copy_len] = '\0';
            }

            if (adtype == AD_TYPE_COMPLETE_LOCAL_NAME &&
                name_len >= FS_ACTIVELOOK_ADV_SERIAL_LEN)
            {
                memcpy(parsed.serial,
                       &name_data[name_len - FS_ACTIVELOOK_ADV_SERIAL_LEN],
                       FS_ACTIVELOOK_ADV_SERIAL_LEN);
                parsed.serial[FS_ACTIVELOOK_ADV_SERIAL_LEN] = '\0';
                parsed.have_serial = true;
            }

            for (size_t pi = 0; pi < sizeof(prefixes) / sizeof(prefixes[0]) &&
                                !parsed.found_name; pi++)
            {
                const char *prefix = prefixes[pi];
                size_t prefix_len = strlen(prefix);

                if (name_len >= prefix_len)
                {
                    size_t mi;
                    bool matches = true;

                    for (mi = 0; mi < prefix_len; mi++)
                    {
                        uint8_t c = name_data[mi];
                        if (c >= 'a' && c <= 'z') c = (uint8_t)(c - 32u);
                        if (c != (uint8_t)prefix[mi])
                        {
                            matches = false;
                            break;
                        }
                    }
                    if (matches) parsed.found_name = true;
                }
            }
        }

        offset += (size_t)adlength + 1u;
    }

    *result = parsed;
    return true;
}
