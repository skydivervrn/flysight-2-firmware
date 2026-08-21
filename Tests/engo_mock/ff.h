/* Minimal FatFs mock for host-testing engo_bind.c — single in-memory file. */
#ifndef MOCK_FF_H_
#define MOCK_FF_H_

#include <stddef.h>

/* Values match the real FRESULT enum in Middlewares/Third_Party/FatFs/src/ff.h. */
typedef int FRESULT;
#define FR_OK       0
#define FR_DISK_ERR 1
#define FR_NO_FILE  4
#define FR_DENIED   7
#define FR_EXIST    8

typedef unsigned int UINT;

#define FA_READ        0x01
#define FA_WRITE       0x02
#define FA_CREATE_NEW  0x04
#define FA_CREATE_ALWAYS 0x08
#define FA_OPEN_ALWAYS 0x10
#define FA_OPEN_APPEND 0x30

/* Fake backing store (defined in the test TU). */
extern char   g_fakeContent[256];
extern int    g_fakeExists;
extern size_t g_fakeLen;

typedef struct { int writing; size_t rpos; } FIL;

FRESULT f_open(FIL *fp, const char *path, int mode);
char   *f_gets(char *buff, int len, FIL *fp);
FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw);
FRESULT f_sync(FIL *fp);
FRESULT f_close(FIL *fp);

#endif
