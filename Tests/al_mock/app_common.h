/* Mock for host-testing FlySight/activelook.c: the sequencer task/priority ids
 * and the hardware timer server, which is all activelook.c takes from the real
 * app_common.h chain. The bodies live in the test TU so it can observe them. */
#ifndef MOCK_APP_COMMON_H_
#define MOCK_APP_COMMON_H_

#include <stdint.h>

/* --- app_conf.h: task ids and scheduler priorities --- */
#define CFG_TASK_FS_ACTIVELOOK_ID   0
#define CFG_TASK_START_SCAN_ID      1
#define CFG_TASK_DISCONN_DEV_1_ID   2
#define CFG_SCH_PRIO_0              0

/* --- hw_conf.h: timer server --- */
#define CFG_TIM_PROC_ID_ISR         0
#define CFG_TS_TICK_VAL             1000u   /* us per tick, as on the target */

typedef enum { hw_ts_SingleShot = 0, hw_ts_Repeated = 1 } HW_TS_Mode_t;
typedef void (*HW_TS_pTimerCb_t)(void);

void HW_TS_Create(uint32_t procId, uint8_t *pId, HW_TS_Mode_t mode, HW_TS_pTimerCb_t cb);
void HW_TS_Start(uint8_t id, uint32_t timeout_ticks);
void HW_TS_Stop(uint8_t id);
void HW_TS_Delete(uint8_t id);

#endif
