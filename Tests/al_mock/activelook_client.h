/* Mock of STM32_WPAN/App/activelook_client.h for host-testing activelook.c.
 * Same signatures as the real header, minus everything that needs ble_types.h
 * and the radio. The test TU implements these so it can play the glasses:
 * assert CB9 STOP, hand back write errors, and watch for ForceDisconnect. */
#ifndef MOCK_ACTIVELOOK_CLIENT_H
#define MOCK_ACTIVELOOK_CLIENT_H

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t tBleStatus;
#define BLE_STATUS_SUCCESS  0x00
#define BLE_STATUS_FAILED   0x41

typedef struct
{
	void (*OnDiscoveryComplete)(void);
} FS_ActiveLook_ClientCb_t;

void       FS_ActiveLook_Client_RegisterCb(const FS_ActiveLook_ClientCb_t *cb);
tBleStatus FS_ActiveLook_Client_WriteWithoutResp(const uint8_t *data, uint16_t length);
uint32_t   FS_ActiveLook_Client_StopStuckMs(void);
uint16_t   FS_ActiveLook_Client_WriteFailStreak(void);
void       FS_ActiveLook_Client_ForceDisconnect(void);
tBleStatus FS_ActiveLook_Client_EnableBatteryNotifications(void);
bool       FS_ActiveLook_Client_CanSend(void);
uint16_t   FS_ActiveLook_Client_GetMTU(void);

#endif
