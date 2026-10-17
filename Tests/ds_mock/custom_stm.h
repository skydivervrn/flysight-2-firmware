/* Mock of STM32_WPAN/App/custom_stm.h for host-testing device_state.c. */
#ifndef MOCK_CUSTOM_STM_H_
#define MOCK_CUSTOM_STM_H_
#include <stdint.h>
#include <stdbool.h>
#define CUSTOM_STM_DS_CONTROL_POINT 7
bool Custom_STM_IsDsControlPointNotificationComplete(uint16_t attr_handle);
#endif
