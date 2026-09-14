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

static int g_logLines;
void FS_Log_WriteEventAsync(const char *format, ...) { (void)format; g_logLines++; }

/* Count every attempt to open the binding file for writing. */
static int g_writeOpenCalls;
/* Read-path faults separate a missing binding from an unreadable card. */
static int g_failReadOpen;   /* f_open(FA_READ) -> FR_DISK_ERR, not FR_NO_FILE */
static int g_failRead;       /* f_gets -> NULL with f_error() set              */
/* The nastier shape: the read dies PART WAY through. Real FatFs ends with
 * `return n ? buff : 0`, so it hands back the characters it managed to collect
 * and only f_error() says anything went wrong. 0 = read the whole line. */
static int g_failReadAfterN;

static void card_healthy(void)
{
	g_failReadOpen = g_failRead = g_failReadAfterN = 0;
}

FRESULT f_open(FIL *fp, const char *path, int mode)
{
	(void)path;
	fp->rpos = 0;
	fp->err  = 0;
	if (mode & (FA_WRITE | FA_CREATE_NEW | FA_CREATE_ALWAYS))
	{
		g_writeOpenCalls++;
		return FR_DENIED;
	}
	/* read */
	if (g_failReadOpen) return FR_DISK_ERR;   /* card unhealthy, NOT "no file" */
	if (!g_fakeExists) return FR_NO_FILE;
	fp->writing = 0;
	return FR_OK;
}

char *f_gets(char *buff, int len, FIL *fp)
{
	if (fp->writing) return NULL;
	if (g_failRead) { fp->err = FR_DISK_ERR; return NULL; }
	if (fp->rpos >= g_fakeLen) return NULL;
	int i = 0;
	while (i < len - 1 && fp->rpos < g_fakeLen)
	{
		if (g_failReadAfterN && i == g_failReadAfterN)
		{
			/* Mirrors ff.c: the loop breaks on the failed f_read, err is set,
			 * and `return n ? buff : 0` still returns the partial line. */
			fp->err = FR_DISK_ERR;
			break;
		}
		char c = g_fakeContent[fp->rpos++];
		buff[i++] = c;
		if (c == '\n') break;
	}
	buff[i] = '\0';
	return i ? buff : NULL;
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

	/* Missing file: no public binding API attempts to create it. */
	set_file(NULL);
	card_healthy();
	g_writeOpenCalls = 0;
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);
	CHECK(g_writeOpenCalls == 0);
	CHECK(FS_EngoBind_Serial()[0] == '\0');
	CHECK(FS_EngoBind_SerialMatches("123456") == false);
	g_logLines = 0;
	FS_EngoBind_LogStatus();
	FS_EngoBind_LogLinkedOnce();
	CHECK(g_logLines == 2);
	CHECK(g_writeOpenCalls == 0);
	CHECK(g_fakeExists == 0);

	/* Invalid content remains byte-for-byte unchanged. */
	set_file("123\n");
	char before[sizeof(g_fakeContent)];
	size_t beforeLen = g_fakeLen;
	memcpy(before, g_fakeContent, beforeLen);
	g_writeOpenCalls = 0;
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);
	CHECK(g_fakeLen == beforeLen);
	CHECK(memcmp(g_fakeContent, before, beforeLen) == 0);
	CHECK(g_writeOpenCalls == 0);

	/* An unreadable card never triggers a write or a binding. */
	set_file("123456\n");
	card_healthy();
	g_writeOpenCalls = 0;
	g_failReadOpen = 1;
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);
	CHECK(g_writeOpenCalls == 0);
	card_healthy();
	g_failRead = 1;
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);
	CHECK(g_writeOpenCalls == 0);

	/* A partial read that returned a plausible prefix is still an I/O error. */
	card_healthy();
	set_file("ID:123456\n");
	g_failReadAfterN = 6;
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);
	CHECK(FS_EngoBind_Serial()[0] == '\0');
	CHECK(g_writeOpenCalls == 0);

	/* An over-long line is invalid, never bound to its truncated prefix. */
	card_healthy();
	set_file("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
	         "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA123456\n");
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);
	CHECK(g_writeOpenCalls == 0);

	/* A valid file binds only to its chosen serial. */
	set_file("123456\n");
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == true);
	CHECK(strcmp(FS_EngoBind_Serial(), "123456") == 0);
	CHECK(FS_EngoBind_SerialMatches("123456") == true);
	CHECK(FS_EngoBind_SerialMatches("999999") == false);
	CHECK(g_writeOpenCalls == 0);

	printf("engo_bind: %d/%d checks passed\n", g_checks - g_fail, g_checks);
	return g_fail ? 1 : 0;
}
