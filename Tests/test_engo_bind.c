/* Host tests for FlySight/engo_bind.c using an in-memory FatFs mock. */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "ff.h"          /* mock (Tests/engo_mock) */
#include "engo_bind.h"   /* real (FlySight) */

/* ---- Fake backing store + FatFs mock implementation ---- */
char   g_fakeContent[256];
int    g_fakeExists;
size_t g_fakeLen;

void FS_Log_WriteEventAsync(const char *format, ...) { (void)format; }

FRESULT f_open(FIL *fp, const char *path, int mode)
{
	(void)path;
	fp->rpos = 0;
	if (mode & FA_CREATE_NEW)
	{
		if (g_fakeExists) return FR_EXIST;   /* never clobber */
		g_fakeExists = 1; g_fakeLen = 0; g_fakeContent[0] = '\0';
		fp->writing = 1;
		return FR_OK;
	}
	/* read */
	if (!g_fakeExists) return FR_NO_FILE;
	fp->writing = 0;
	return FR_OK;
}

char *f_gets(char *buff, int len, FIL *fp)
{
	if (fp->writing) return NULL;
	if (fp->rpos >= g_fakeLen) return NULL;
	int i = 0;
	while (i < len - 1 && fp->rpos < g_fakeLen)
	{
		char c = g_fakeContent[fp->rpos++];
		buff[i++] = c;
		if (c == '\n') break;
	}
	buff[i] = '\0';
	return buff;
}

FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw)
{
	if (!fp->writing) return FR_DENIED;
	memcpy(&g_fakeContent[g_fakeLen], buff, btw);
	g_fakeLen += btw;
	g_fakeContent[g_fakeLen] = '\0';
	*bw = btw;
	return FR_OK;
}

FRESULT f_close(FIL *fp) { (void)fp; return FR_OK; }

/* ---- helpers ---- */
static void set_file(const char *content)
{
	if (content == NULL) { g_fakeExists = 0; g_fakeLen = 0; g_fakeContent[0] = '\0'; return; }
	g_fakeExists = 1;
	g_fakeLen = strlen(content);
	memcpy(g_fakeContent, content, g_fakeLen + 1);
}

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { \
	g_checks++; \
	if (!(cond)) { g_fail++; printf("FAIL line %d: %s\n", __LINE__, #cond); } \
} while (0)

int main(void)
{
	/* 1. No file -> unbound. */
	set_file(NULL);
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);

	/* 2. Plain serial. */
	set_file("123456\n");
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == true);
	CHECK(strcmp(FS_EngoBind_Serial(), "123456") == 0);
	CHECK(FS_EngoBind_SerialMatches("123456") == true);
	CHECK(FS_EngoBind_SerialMatches("999999") == false);

	/* 3. "ID: 123456" -> last 6 chars. */
	set_file("ID: 123456\n");
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == true);
	CHECK(strcmp(FS_EngoBind_Serial(), "123456") == 0);

	/* 4. Full name line -> last 6 chars. */
	set_file("ENGO 3 123456\n");
	FS_EngoBind_Load();
	CHECK(strcmp(FS_EngoBind_Serial(), "123456") == 0);

	/* 5. Trailing spaces trimmed. */
	set_file("123456   \n");
	FS_EngoBind_Load();
	CHECK(strcmp(FS_EngoBind_Serial(), "123456") == 0);

	/* 6. Too short -> unbound. */
	set_file("123\n");
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);

	/* 7. Empty/whitespace -> unbound. */
	set_file("   \n");
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);

	/* 8. Auto-bind: unbound + NotePending + Commit writes the file. */
	set_file(NULL);
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);
	FS_EngoBind_NotePending("123456");
	FS_EngoBind_CommitIfPending();
	CHECK(FS_EngoBind_IsBound() == true);
	CHECK(strcmp(FS_EngoBind_Serial(), "123456") == 0);
	CHECK(g_fakeExists == 1);
	CHECK(strncmp(g_fakeContent, "123456", 6) == 0);

	/* 9. NotePending is a no-op once bound (can't silently re-bind). */
	FS_EngoBind_NotePending("999999");
	FS_EngoBind_CommitIfPending();
	CHECK(strcmp(FS_EngoBind_Serial(), "123456") == 0);

	/* 10. Commit never clobbers an existing file. */
	set_file("AAAAAA\n");      /* simulate a pre-existing engo3.txt */
	FS_EngoBind_Load();        /* binds to AAAAAA */
	CHECK(strcmp(FS_EngoBind_Serial(), "AAAAAA") == 0);
	/* force unbound state but keep file, then try to learn a different serial */
	set_file("AAAAAA\n");
	/* simulate fresh boot reading the file again, then a (wrong) pending learn */
	FS_EngoBind_Load();
	FS_EngoBind_NotePending("BBBBBB");  /* ignored: already bound */
	FS_EngoBind_CommitIfPending();
	CHECK(strncmp(g_fakeContent, "AAAAAA", 6) == 0);  /* file unchanged */

	printf("engo_bind: %d/%d checks passed\n", g_checks - g_fail, g_checks);
	return g_fail ? 1 : 0;
}
