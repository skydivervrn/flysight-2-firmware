/* Minimal log mock for host-testing activelook_mode0.c (the real log.h pulls
 * in every sensor header). The test TU records the lines. */
#ifndef MOCK_MODE0_LOG_H_
#define MOCK_MODE0_LOG_H_
void FS_Log_WriteEventAsync(const char *format, ...);
#endif
