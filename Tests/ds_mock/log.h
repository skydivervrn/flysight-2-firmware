/* Mock of FlySight/log.h for host-testing device_state.c. */
#ifndef MOCK_DS_LOG_H_
#define MOCK_DS_LOG_H_
#include <stdbool.h>
void FS_Log_WriteEventAsync(const char *format, ...);
bool FS_Log_EventSlotFree(void);
#endif
