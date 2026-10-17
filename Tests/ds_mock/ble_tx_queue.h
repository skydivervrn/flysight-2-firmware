/* Mock of FlySight/ble_tx_queue.h for host-testing device_state.c. */
#ifndef MOCK_BLE_TX_QUEUE_H_
#define MOCK_BLE_TX_QUEUE_H_
#include <stdint.h>
typedef void (*BLE_TX_Queue_callback_t)(void);
void BLE_TX_Queue_SendTxPacket(uint8_t char_id, uint8_t *data, uint8_t length,
                               uint8_t *size_ptr, BLE_TX_Queue_callback_t callback);
#endif
