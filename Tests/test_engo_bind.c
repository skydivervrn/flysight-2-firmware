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

/* ---- Fault injection for the card. All zero = a healthy card. ----
 * g_writeCalls counts f_write calls since the last reset; the *At knobs name
 * the 1-based call that misbehaves. This is how a full or dying SD card is
 * reproduced on the host: FatFs reports the failure in the FRESULT, or writes
 * fewer bytes than asked and says FR_OK. */
static int g_writeCalls;
static int g_failWriteAt;    /* this f_write returns FR_DISK_ERR   */
static int g_shortWriteAt;   /* this f_write returns FR_OK, bw-1   */
static int g_failSync;       /* f_sync returns FR_DISK_ERR         */
static int g_failClose;      /* f_close returns FR_DISK_ERR        */
/* Read-path faults. These separate "the card says there is no binding" from
 * "the card would not answer", and the whole recovery rule turns on it. */
static int g_failReadOpen;   /* f_open(FA_READ) -> FR_DISK_ERR, not FR_NO_FILE */
static int g_failRead;       /* f_gets -> NULL with f_error() set              */
/* The nastier shape: the read dies PART WAY through. Real FatFs ends with
 * `return n ? buff : 0`, so it hands back the characters it managed to collect
 * and only f_error() says anything went wrong. 0 = read the whole line. */
static int g_failReadAfterN;

static void card_healthy(void)
{
	g_writeCalls = 0;
	g_failWriteAt = g_shortWriteAt = g_failSync = g_failClose = 0;
	g_failReadOpen = g_failRead = g_failReadAfterN = 0;
}

FRESULT f_open(FIL *fp, const char *path, int mode)
{
	(void)path;
	fp->rpos = 0;
	fp->err  = 0;
	if (mode & FA_CREATE_NEW)
	{
		if (g_fakeExists) return FR_EXIST;   /* never clobber */
		g_fakeExists = 1; g_fakeLen = 0; g_fakeContent[0] = '\0';
		fp->writing = 1;
		return FR_OK;
	}
	if (mode & FA_CREATE_ALWAYS)
	{
		/* Truncate-or-create: this is the self-heal path, and it is allowed to
		 * land on an existing file precisely because the caller has already
		 * established that the file is rubbish. */
		g_fakeExists = 1; g_fakeLen = 0; g_fakeContent[0] = '\0';
		fp->writing = 1;
		return FR_OK;
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

FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw)
{
	if (!fp->writing) return FR_DENIED;

	g_writeCalls++;

	if (g_writeCalls == g_failWriteAt) { *bw = 0; return FR_DISK_ERR; }

	if (g_writeCalls == g_shortWriteAt && btw > 0)
		btw--;   /* card full: FatFs writes what fits and still says FR_OK */

	memcpy(&g_fakeContent[g_fakeLen], buff, btw);
	g_fakeLen += btw;
	g_fakeContent[g_fakeLen] = '\0';
	*bw = btw;
	return FR_OK;
}

FRESULT f_sync(FIL *fp)  { (void)fp; return g_failSync  ? FR_DISK_ERR : FR_OK; }
FRESULT f_close(FIL *fp) { (void)fp; return g_failClose ? FR_DISK_ERR : FR_OK; }

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

	/* ---------------------------------------------------------------------
	 * 11-15. A card that does not take the write must not leave the session
	 * thinking it is bound. Before the check, CommitIfPending ignored every
	 * FRESULT and every byte count: on a full card it set s_bound anyway, the
	 * flight ran as if pinned to these glasses, and the truth only surfaced at
	 * the next boot — an engo3.txt that will not parse reads as "unbound",
	 * i.e. connect to whatever glasses answer first.
	 * ------------------------------------------------------------------- */

	/* 11. The serial write fails outright. */
	set_file(NULL);
	card_healthy();
	FS_EngoBind_Load();
	g_failWriteAt = 1;
	FS_EngoBind_NotePending("123456");
	FS_EngoBind_CommitIfPending();
	CHECK(FS_EngoBind_IsBound() == false);
	CHECK(FS_EngoBind_Serial()[0] == '\0');

	/* 12. The serial write is short — FR_OK, but fewer bytes than asked
	 *     (the classic "card is full" shape). */
	set_file(NULL);
	card_healthy();
	FS_EngoBind_Load();
	g_shortWriteAt = 1;
	FS_EngoBind_NotePending("123456");
	FS_EngoBind_CommitIfPending();
	CHECK(FS_EngoBind_IsBound() == false);
	CHECK(g_fakeLen == FS_ENGO_SERIAL_LEN - 1);   /* the truncation is real */

	/* 13. The serial lands but the trailing newline does not. */
	set_file(NULL);
	card_healthy();
	FS_EngoBind_Load();
	g_failWriteAt = 2;
	FS_EngoBind_NotePending("123456");
	FS_EngoBind_CommitIfPending();
	CHECK(FS_EngoBind_IsBound() == false);

	/* 14. Both writes report success but the flush fails, so nothing reached
	 *     the card. */
	set_file(NULL);
	card_healthy();
	FS_EngoBind_Load();
	g_failSync = 1;
	FS_EngoBind_NotePending("123456");
	FS_EngoBind_CommitIfPending();
	CHECK(FS_EngoBind_IsBound() == false);

	/* 15. Close fails: FatFs writes the last sector and the directory entry
	 *     there, so this is a real loss of the file too. */
	set_file(NULL);
	card_healthy();
	FS_EngoBind_Load();
	g_failClose = 1;
	FS_EngoBind_NotePending("123456");
	FS_EngoBind_CommitIfPending();
	CHECK(FS_EngoBind_IsBound() == false);

	/* 16. And the healthy card still binds, with the whole "123456\n" on it —
	 *     the checks above must not have made success unreachable. */
	set_file(NULL);
	card_healthy();
	FS_EngoBind_Load();
	FS_EngoBind_NotePending("123456");
	FS_EngoBind_CommitIfPending();
	CHECK(FS_EngoBind_IsBound() == true);
	CHECK(strcmp(FS_EngoBind_Serial(), "123456") == 0);
	CHECK(g_fakeLen == FS_ENGO_SERIAL_LEN + 1);
	CHECK(strcmp(g_fakeContent, "123456\n") == 0);

	/* 17. A failed commit says so in the event log — this is the only way the
	 *     user ever learns the binding did not stick. */
	set_file(NULL);
	card_healthy();
	FS_EngoBind_Load();
	g_failWriteAt = 1;
	g_logLines = 0;
	FS_EngoBind_NotePending("123456");
	FS_EngoBind_CommitIfPending();
	CHECK(g_logLines == 1);

	/* ---------------------------------------------------------------------
	 * 18-23. Recovery. A binding that fails to write used to be a one-way
	 * door: the stub left on the card is rejected by Load(), and the next
	 * attempt's FA_CREATE_NEW then bounces off it with FR_EXIST, boot after
	 * boot, until someone deleted the file over USB. The device could not be
	 * re-bound in the field at all.
	 *
	 * The escape is deliberately narrow. "Not bound" is not enough to license
	 * an overwrite, because a card that will not answer looks exactly like a
	 * card full of garbage — and guessing wrong re-pins the device to whatever
	 * glasses are switched on nearby. Only a file that was read AND closed
	 * cleanly, and still did not parse, may be replaced.
	 * ------------------------------------------------------------------- */

	/* 18. Short write, then a reboot: the stub is recognised as rubbish and the
	 *     next bind replaces it. This is the sequence that used to wedge. */
	set_file(NULL);
	card_healthy();
	FS_EngoBind_Load();
	g_shortWriteAt = 1;
	FS_EngoBind_NotePending("123456");
	FS_EngoBind_CommitIfPending();
	CHECK(FS_EngoBind_IsBound() == false);
	CHECK(g_fakeExists == 1);                       /* the stub is on the card */
	card_healthy();
	FS_EngoBind_Load();                             /* reboot */
	CHECK(FS_EngoBind_IsBound() == false);          /* stub does not parse */
	FS_EngoBind_NotePending("123456");
	FS_EngoBind_CommitIfPending();
	CHECK(FS_EngoBind_IsBound() == true);
	CHECK(strcmp(g_fakeContent, "123456\n") == 0);

	/* 19. A power cut leaves the same stub without any cleanup having run —
	 *     no f_unlink, no rename, nothing. Recovery must not depend on the
	 *     failing session getting a chance to tidy up after itself. */
	set_file("123");                                 /* truncated, as if mid-write */
	card_healthy();
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);
	FS_EngoBind_NotePending("654321");
	FS_EngoBind_CommitIfPending();
	CHECK(FS_EngoBind_IsBound() == true);
	CHECK(strcmp(g_fakeContent, "654321\n") == 0);

	/* 20. A valid binding is still never replaced automatically. */
	set_file("AAAAAA\n");
	card_healthy();
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == true);
	FS_EngoBind_NotePending("BBBBBB");
	FS_EngoBind_CommitIfPending();
	CHECK(strcmp(g_fakeContent, "AAAAAA\n") == 0);

	/* 21. The card refuses to open the file for reading. That is NOT "no
	 *     binding": a real serial may be sitting there. Nothing may be written,
	 *     or a flaky reader would silently re-pin the device. */
	set_file("AAAAAA\n");
	card_healthy();
	g_failReadOpen = 1;
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);          /* unbound for this session */
	g_failReadOpen = 0;                             /* card recovers mid-flight */
	FS_EngoBind_NotePending("BBBBBB");
	FS_EngoBind_CommitIfPending();
	CHECK(strcmp(g_fakeContent, "AAAAAA\n") == 0);  /* binding survived */

	/* 22. The open succeeds but the read faults. Same rule. */
	set_file("AAAAAA\n");
	card_healthy();
	g_failRead = 1;
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);
	g_failRead = 0;
	FS_EngoBind_NotePending("BBBBBB");
	FS_EngoBind_CommitIfPending();
	CHECK(strcmp(g_fakeContent, "AAAAAA\n") == 0);

	/* 23. The file read as garbage, but the close failed — so we do not
	 *     actually know the read was sound. Refuse, rather than call it
	 *     invalid and overwrite. An empty file is the sharpest case: it looks
	 *     exactly like a stub we would be entitled to replace. */
	set_file("");
	g_fakeExists = 1;                               /* present but zero length */
	card_healthy();
	g_failClose = 1;
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);
	g_failClose = 0;
	g_logLines = 0;
	FS_EngoBind_NotePending("BBBBBB");
	FS_EngoBind_CommitIfPending();
	CHECK(g_fakeLen == 0);                          /* nothing written */
	CHECK(g_logLines == 1);                         /* and the refusal is logged */

	/* ---------------------------------------------------------------------
	 * 24-26. A read that dies PART WAY through the line. This is the shape
	 * that survives an f_gets != NULL check: FatFs ends with
	 * `return n ? buff : 0`, so the call hands back whatever it collected
	 * before the fault and only f_error() says anything is wrong. Checking
	 * the flag solely on the NULL branch therefore parses a stump as if it
	 * were the whole file.
	 * ------------------------------------------------------------------- */

	/* 24. Fewer than six characters survive. The stump looks like a corrupt
	 *     file, and calling it invalid would license FA_CREATE_ALWAYS over a
	 *     binding that is perfectly good. */
	set_file("123456\n");
	card_healthy();
	g_failReadAfterN = 3;                            /* "123" then the card dies */
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);
	g_failReadAfterN = 0;
	FS_EngoBind_NotePending("BBBBBB");
	FS_EngoBind_CommitIfPending();
	CHECK(strcmp(g_fakeContent, "123456\n") == 0);   /* untouched */

	/* 25. Six or more characters survive, and the stump passes serial_valid()
	 *     on its own. Without the flag check the device pins itself to a serial
	 *     that does not exist: "ID:123456" cut to "ID:123". */
	set_file("ID:123456\n");
	card_healthy();
	g_failReadAfterN = 6;
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == false);           /* NOT bound to "ID:123" */
	CHECK(FS_EngoBind_Serial()[0] == '\0');
	g_failReadAfterN = 0;
	FS_EngoBind_NotePending("BBBBBB");
	FS_EngoBind_CommitIfPending();
	CHECK(strcmp(g_fakeContent, "ID:123456\n") == 0);

	/* 26. The same file read without a fault still binds to the real serial —
	 *     the check must not have made the normal path unreachable. */
	set_file("ID:123456\n");
	card_healthy();
	FS_EngoBind_Load();
	CHECK(FS_EngoBind_IsBound() == true);
	CHECK(strcmp(FS_EngoBind_Serial(), "123456") == 0);

	printf("engo_bind: %d/%d checks passed\n", g_checks - g_fail, g_checks);
	return g_fail ? 1 : 0;
}
