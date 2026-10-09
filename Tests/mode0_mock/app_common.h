/* Mock for host-testing FlySight/activelook_mode0.c: the HAL clock and the one
 * HAL type baro.h needs. The real app_common.h pulls in the whole CubeMX chain.
 * HAL_GetTick is implemented by the test TU so it can move time by hand. */
#ifndef MOCK_MODE0_APP_COMMON_H_
#define MOCK_MODE0_APP_COMMON_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;

uint32_t HAL_GetTick(void);

#endif
