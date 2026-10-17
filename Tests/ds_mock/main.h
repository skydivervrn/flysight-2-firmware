/* Mock of Core/Inc/main.h for host-testing FlySight/device_state.c: PRIMASK
 * intrinsics (no-ops on the host, but counted so the test can see the ring is
 * touched under them) and the timer server. Bodies in the test TU. */
#ifndef MOCK_DS_MAIN_H_
#define MOCK_DS_MAIN_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define CFG_TIM_PROC_ID_ISR 0
#define CFG_TS_TICK_VAL     1000u

typedef enum { hw_ts_SingleShot = 0, hw_ts_Repeated = 1 } HW_TS_Mode_t;
typedef void (*HW_TS_pTimerCb_t)(void);

void HW_TS_Create(uint32_t procId, uint8_t *pId, HW_TS_Mode_t mode, HW_TS_pTimerCb_t cb);
void HW_TS_Start(uint8_t id, uint32_t timeout_ticks);
void HW_TS_Stop(uint8_t id);

uint32_t __get_PRIMASK(void);
void     __disable_irq(void);
void     __set_PRIMASK(uint32_t v);

#endif
