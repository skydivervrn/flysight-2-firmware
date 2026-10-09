/* Mock of STM32_WPAN/App/activelook_client.h for host-testing
 * activelook_mode0.c: the three calls the renderer makes. The test TU plays
 * the radio — it records every packet and can push back like a full CPU2 TX
 * pool. */
#ifndef MOCK_MODE0_ACTIVELOOK_CLIENT_H
#define MOCK_MODE0_ACTIVELOOK_CLIENT_H

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t tBleStatus;
#define BLE_STATUS_SUCCESS  0x00
#define BLE_STATUS_FAILED   0x41

tBleStatus FS_ActiveLook_Client_WriteWithoutResp(const uint8_t *data, uint16_t length);
bool       FS_ActiveLook_Client_CanSend(void);
uint8_t    FS_ActiveLook_Client_GetBatteryLevel(void);

#endif
