/* Minimal log mock for host-testing FlySight/activelook.c. */
#ifndef MOCK_LOG_H_
#define MOCK_LOG_H_
void FS_Log_WriteEventAsync(const char *format, ...);
#endif
