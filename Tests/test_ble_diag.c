/* Host tests for FlySight/ble_diag.c — the ring buffer and the line formatter.
 *
 * This is the whole pure half of the BLE reconnect diagnostic: the sink
 * (ble_diag_sink.c) is only a clock and a FatFs append, and cannot be linked
 * here. What matters and is checked here is that no record can grow past its
 * line, that a full ring loses the NEWEST record rather than corrupting the
 * history it already holds, that the loss is countable, and that a device name
 * straight off the air cannot smuggle a newline into a one-event-per-line file. */

#include <stdio.h>
#include <string.h>

#include "ble_diag.h"

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { \
	g_checks++; \
	if (!(cond)) { g_fail++; printf("FAIL line %d: %s\n", __LINE__, #cond); } \
} while (0)

/* Skip the "<seq> <ms> " prefix that every line carries. */
static const char *body(const char *line)
{
	int spaces = 0;
	while (*line && spaces < 2)
	{
		if (*line == ' ') spaces++;
		line++;
	}
	return line;
}

int main(void)
{
	const char *l;
	char addr[18];
	char big[400];
	uint32_t i;

	/* 1. Empty ring: nothing to peek, nothing to drop, no crash on Drop. */
	FS_BleDiag_Reset();
	CHECK(FS_BleDiag_Pending() == 0);
	CHECK(FS_BleDiag_Peek() == NULL);
	FS_BleDiag_Drop();
	CHECK(FS_BleDiag_Pending() == 0);
	CHECK(FS_BleDiag_TakeDropped() == 0);

	/* 2. A record comes back with its sequence number and timestamp in front,
	 *    zero-padded so the file sorts and diffs as text. */
	FS_BleDiag_Emit(1234, "WL get_bonded=0x%02X n=%u", 0, 0);
	CHECK(FS_BleDiag_Pending() == 1);
	l = FS_BleDiag_Peek();
	CHECK(l != NULL);
	CHECK(strcmp(l, "000000 00001234 WL get_bonded=0x00 n=0") == 0);

	/* 3. FIFO order, and Drop advances it. */
	FS_BleDiag_Emit(1235, "second");
	CHECK(FS_BleDiag_Pending() == 2);
	CHECK(strcmp(body(FS_BleDiag_Peek()), "WL get_bonded=0x00 n=0") == 0);
	FS_BleDiag_Drop();
	CHECK(FS_BleDiag_Pending() == 1);
	CHECK(strcmp(body(FS_BleDiag_Peek()), "second") == 0);
	FS_BleDiag_Drop();
	CHECK(FS_BleDiag_Pending() == 0);
	CHECK(FS_BleDiag_Peek() == NULL);

	/* 4. The sequence number keeps counting across drains — it identifies the
	 *    record, it is not an index into the ring. */
	FS_BleDiag_Emit(9, "third");
	CHECK(strncmp(FS_BleDiag_Peek(), "000002 ", 7) == 0);
	FS_BleDiag_Drop();

	/* 5. A record longer than the line is TRUNCATED, never wrapped: two lines
	 *    for one event would be read as two events. */
	FS_BleDiag_Reset();
	memset(big, 'A', sizeof(big) - 1);
	big[sizeof(big) - 1] = '\0';
	FS_BleDiag_Emit(0, "%s", big);
	l = FS_BleDiag_Peek();
	CHECK(strlen(l) == FS_BLE_DIAG_LINE_LEN - 1);
	CHECK(strchr(l, '\n') == NULL);

	/* 6. Non-printable bytes are replaced, so an advertised name carrying a
	 *    newline or a NUL-adjacent control byte cannot split a record. */
	FS_BleDiag_Reset();
	FS_BleDiag_Emit(0, "ENGO found: '%s'", "EN\r\nGO\t\x01 3");
	l = body(FS_BleDiag_Peek());
	CHECK(strcmp(l, "ENGO found: 'EN..GO.. 3'") == 0);
	CHECK(strchr(l, '\n') == NULL && strchr(l, '\r') == NULL);

	/* 7. A full ring drops the NEWEST record and counts it. Everything already
	 *    buffered survives intact — the history is what we came for. */
	FS_BleDiag_Reset();
	for (i = 0; i < FS_BLE_DIAG_LINES; i++)
		FS_BleDiag_Emit(i, "line %lu", (unsigned long) i);
	CHECK(FS_BleDiag_Pending() == FS_BLE_DIAG_LINES);
	CHECK(FS_BleDiag_TakeDropped() == 0);

	FS_BleDiag_Emit(999, "overflow me");
	FS_BleDiag_Emit(999, "and me");
	CHECK(FS_BleDiag_Pending() == FS_BLE_DIAG_LINES);
	CHECK(strcmp(body(FS_BleDiag_Peek()), "line 0") == 0);
	CHECK(FS_BleDiag_TakeDropped() == 2);
	CHECK(FS_BleDiag_TakeDropped() == 0);  /* taking clears the counter */

	/* 7b. ...and the sequence numbers still show the gap, so the file says how
	 *     many records went missing even if the LOST marker is itself lost. */
	FS_BleDiag_Drop();  /* discard "line 0", freeing one slot */
	FS_BleDiag_Emit(1000, "after the gap");
	for (i = 0; i < FS_BLE_DIAG_LINES - 1; i++) FS_BleDiag_Drop();
	l = FS_BleDiag_Peek();
	CHECK(strcmp(body(l), "after the gap") == 0);
	/* 48 lines (seq 0..47) + 2 dropped (48, 49) => this one is 50 */
	CHECK(strncmp(l, "000050 ", 7) == 0);

	/* 8. The ring keeps working after wrapping right round. */
	FS_BleDiag_Reset();
	for (i = 0; i < FS_BLE_DIAG_LINES * 3; i++)
	{
		FS_BleDiag_Emit(i, "wrap %lu", (unsigned long) i);
		CHECK(FS_BleDiag_Pending() == 1);
		CHECK(FS_BleDiag_Peek() != NULL);
		FS_BleDiag_Drop();
	}
	CHECK(FS_BleDiag_TakeDropped() == 0);

	/* 9. Addresses print most-significant byte first, the way the BT spec and
	 *    every scanner show them, so they can be compared against nRF Connect
	 *    by eye. The array is little-endian as it comes off the controller. */
	{
		const uint8_t a[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };
		CHECK(FS_BleDiag_FormatAddr(addr, sizeof(addr), a) == 17);
		CHECK(strcmp(addr, "66:55:44:33:22:11") == 0);
	}

	/* 9b. A buffer that cannot hold the address is refused, not half-filled. */
	{
		const uint8_t a[6] = { 0, 0, 0, 0, 0, 0 };
		char small[17];
		memset(small, 'x', sizeof(small));
		CHECK(FS_BleDiag_FormatAddr(small, sizeof(small), a) == 0);
		CHECK(small[0] == 'x');
		CHECK(FS_BleDiag_FormatAddr(NULL, 64, a) == 0);
	}

	/* 10. Every record the firmware actually emits has to survive whole, at the
	 *     WORST timestamp — HAL_GetTick past ten digits, which is where the
	 *     first attempt at this buffer quietly started clipping. A truncated
	 *     line is not a cosmetic problem: these are the lines the diagnosis is
	 *     read off, and the fields that decide it sit at the END of them. */
	{
		static const char *const worst[] = {
			"CFG privacy=2 id_addr=1 own_addr=2 bonding=1 mitm=0 io=3 sc=1 adv_filter=0 central=1",
			"STATE open=OK reset_ble=0 enable_ble=1 name='123456789012345678901234567890' irk=AABBCCDD",
			"STATE open=FAIL fr=1 defaults reset_ble=0 enable_ble=1",
			"ADV path=undirected_connectable filter=0x03 ret=0x00 status=1 bonds=-1 int=128-160",
			"ADV path=limited_discoverable filter=0 ret=0x00 status=1 bonds=-1 int=128-160",
			"BLE_RESET clear_security_db=0x00 (ALL BONDS ERASED)",
			"WL get_bonded=0x00 n=0",
			"BOND 16/16 type=1 addr=66:55:44:33:22:11",
			"RL add_devices=0x00 n=16",
			"CONN st=0x00 role=1 handle=0x0801 ptype=1 paddr=66:55:44:33:22:11",
			"CONN handle=0x0801 peer_rpa=66:55:44:33:22:11 int=24 to=500",
			"DISC handle=0x0801 reason=0x08 peer=phone status=0x00",
			"PAIR_CPLT handle=0x0801 status=0x02 reason=0x03",
			"PAIR_BONDS get_bonded=0x00 n=1",
			"ADDR_NOT_RESOLVED handle=0x0801",
			"BOND_LOST (peer re-pairing over an existing bond)",
		};
		for (i = 0; i < sizeof(worst) / sizeof(worst[0]); i++)
		{
			FS_BleDiag_Reset();
			FS_BleDiag_Emit(4294967295UL, "%s", worst[i]);
			l = FS_BleDiag_Peek();
			CHECK(strcmp(body(l), worst[i]) == 0);
			CHECK(strlen(l) < FS_BLE_DIAG_LINE_LEN - 1);
		}
	}

	printf("%d checks, %d failures\n", g_checks, g_fail);
	return g_fail ? 1 : 0;
}
