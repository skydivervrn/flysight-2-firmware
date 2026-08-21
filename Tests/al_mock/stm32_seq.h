/* Mock sequencer for host-testing FlySight/activelook.c. RegTask hands the test
 * the task function pointer, so the FSM can be stepped exactly as the scheduler
 * would step it. Bodies in the test TU. */
#ifndef MOCK_STM32_SEQ_H_
#define MOCK_STM32_SEQ_H_

#include <stdint.h>

#define UTIL_SEQ_RFU 0

void UTIL_SEQ_RegTask(uint32_t taskMask, uint32_t flags, void (*task)(void));
void UTIL_SEQ_SetTask(uint32_t taskMask, uint32_t prio);

#endif
