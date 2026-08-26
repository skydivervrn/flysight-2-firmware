#include <stdio.h>
#include <string.h>

#include "crs_mock.h"
#include "crs.h"

enum
{
	CRS_READ = 0x02,
	CRS_WRITE = 0x03,
	CRS_FILE_DATA = 0x10,
	CRS_FILE_ACK = 0x12,
	CRS_ACK = 0xf1,
	CRS_NAK = 0xf0,
	CRS_CANCEL = 0xff
};

uint8_t SizeFt_Packet_Out;

static void (*g_update)(void);
static Custom_CRS_Packet_t g_rx;
static int g_rx_ready;
static uint8_t g_tx_staging[245];
static uint8_t g_tx[16][245];
static uint8_t g_tx_len[16];
static int g_tx_count;

static FRESULT g_write_result;
static int g_short_write;
static FRESULT g_close_result;
static FRESULT g_seek_result;
static int g_open_count;
static int g_close_count;
static int g_release_count;
static const uint8_t *g_packet_begin;
static const uint8_t *g_packet_end;
static int g_path_was_internal;
static int g_path_was_terminated;
static size_t g_path_length;

static int g_checks;
static int g_failures;

#define CHECK(condition) do { \
	g_checks++; \
	if (!(condition)) { \
		g_failures++; \
		printf("FAIL %s:%d: %s\n", __func__, __LINE__, #condition); \
	} \
} while (0)

#define RUN_TEST(test) do { \
	int failures_before = g_failures; \
	test(); \
	printf("%s: %s\n", #test, \
		g_failures == failures_before ? "PASS" : "FAIL"); \
} while (0)

uint8_t *BLE_TX_Queue_GetNextTxPacket(void)
{
	return g_tx_count < 16 ? g_tx_staging : NULL;
}

void BLE_TX_Queue_SendNextTxPacket(Custom_STM_Char_Opcode_t opcode,
		uint8_t length, uint8_t *size_ptr, BLE_TX_Queue_callback_t callback)
{
	(void) opcode;
	(void) size_ptr;
	(void) callback;
	memcpy(g_tx[g_tx_count], g_tx_staging, length);
	g_tx_len[g_tx_count++] = length;
}

uint8_t Custom_APP_IsConnected(void) { return 1; }

Custom_CRS_Packet_t *Custom_CRS_GetNextRxPacket(void)
{
	if (!g_rx_ready) return NULL;
	g_rx_ready = 0;
	return &g_rx;
}

FRESULT f_open(FIL *fp, const TCHAR *path, uint8_t mode)
{
	const uint8_t *p = (const uint8_t *) path;
	(void) mode;
	g_open_count++;
	g_path_was_internal = p < g_packet_begin || p >= g_packet_end;
	g_path_length = strnlen(path, 245);
	g_path_was_terminated = g_path_length < 245;
	fp->open = 1;
	fp->pos = 0;
	return FR_OK;
}

FRESULT f_close(FIL *fp)
{
	g_close_count++;
	if (g_close_result == FR_OK) fp->open = 0;
	return g_close_result;
}

FRESULT f_lseek(FIL *fp, FSIZE_t offset)
{
	if (g_seek_result == FR_OK) fp->pos = offset;
	return g_seek_result;
}

FRESULT f_read(FIL *fp, void *buffer, UINT count, UINT *br)
{
	(void) fp; (void) buffer; (void) count; *br = 0; return FR_OK;
}

FRESULT f_write(FIL *fp, const void *buffer, UINT count, UINT *bw)
{
	(void) fp; (void) buffer;
	*bw = g_short_write && count ? count - 1 : count;
	return g_write_result;
}

FRESULT f_unlink(const TCHAR *path) { (void) path; return FR_OK; }
FRESULT f_mkdir(const TCHAR *path) { (void) path; return FR_OK; }
FRESULT f_opendir(DIR *dp, const TCHAR *path) { (void) path; dp->open = 1; return FR_OK; }
FRESULT f_readdir(DIR *dp, FILINFO *fno) { (void) dp; memset(fno, 0, sizeof(*fno)); return FR_OK; }
FRESULT f_closedir(DIR *dp) { dp->open = 0; return FR_OK; }

FS_ResourceManager_Result_t FS_ResourceManager_RequestResource(FS_Resource_t resource)
{
	(void) resource; return FS_RESOURCE_MANAGER_SUCCESS;
}

void FS_ResourceManager_ReleaseResource(FS_Resource_t resource)
{
	(void) resource; g_release_count++;
}

void UTIL_SEQ_RegTask(uint32_t task_id, uint32_t flags, void (*callback)(void))
{
	(void) task_id; (void) flags; g_update = callback;
}
void UTIL_SEQ_SetTask(uint32_t task_id, uint32_t priority) { (void) task_id; (void) priority; }
void HW_TS_Create(uint32_t proc_id, uint8_t *timer_id, HW_TS_Mode_t mode,
		HW_TS_pTimerCb_t callback)
{
	(void) proc_id; (void) mode; (void) callback; *timer_id = 1;
}
void HW_TS_Start(uint8_t timer_id, uint32_t timeout_ticks) { (void) timer_id; (void) timeout_ticks; }
void HW_TS_Stop(uint8_t timer_id) { (void) timer_id; }

static void reset_observations(void)
{
	memset(&g_rx, 0, sizeof(g_rx));
	g_rx_ready = 0;
	g_tx_count = 0;
	g_write_result = FR_OK;
	g_short_write = 0;
	g_close_result = FR_OK;
	g_seek_result = FR_OK;
	g_open_count = 0;
	g_close_count = 0;
	g_release_count = 0;
	g_packet_begin = g_rx.data;
	g_packet_end = g_rx.data + sizeof(g_rx.data);
	g_path_was_internal = 0;
	g_path_was_terminated = 0;
	g_path_length = 0;
}

static void send_packet(const uint8_t *data, size_t length)
{
	memcpy(g_rx.data, data, length <= sizeof(g_rx.data) ? length : sizeof(g_rx.data));
	g_rx.length = (uint8_t) length;
	g_rx_ready = 1;
	g_update();
}

static int sent(uint8_t command, uint8_t payload)
{
	int i;
	for (i = 0; i < g_tx_count; ++i)
		if (g_tx_len[i] == 2 && g_tx[i][0] == command && g_tx[i][1] == payload)
			return 1;
	return 0;
}

static void begin_write(void)
{
	uint8_t packet[10] = { CRS_WRITE, '/', 'T', '.', 'B', 'I', 'N', 0, 0, 0 };
	send_packet(packet, sizeof(packet));
	CHECK(sent(CRS_ACK, CRS_WRITE));
	g_tx_count = 0;
}

static void cancel_write(void)
{
	uint8_t packet[] = { CRS_CANCEL };
	g_close_result = FR_OK;
	send_packet(packet, sizeof(packet));
}

static void test_crs_write_short_path_is_accepted(void)
{
	/* WRITE carries no offset field: the path starts right after the opcode.
	 * The phone sends short paths -- "/T.BIN" is the harmless target the flash
	 * runbook uploads to before touching anything real -- and a minimum-length
	 * gate borrowed from READ would NAK every one of them. */
	uint8_t packet[] = { CRS_WRITE, '/', 'T', '.', 'B', 'I', 'N', 0 };
	reset_observations();
	send_packet(packet, sizeof(packet));
	CHECK(g_open_count == 1);
	CHECK(sent(CRS_ACK, CRS_WRITE));
	CHECK(g_path_was_internal);
	CHECK(g_path_was_terminated);
	cancel_write();
}

static void test_crs_write_max_length(void)
{
	uint8_t packet[244];
	reset_observations();
	memset(packet, 'P', sizeof(packet));
	packet[0] = CRS_WRITE;
	packet[1] = '/';
	send_packet(packet, sizeof(packet));
	CHECK(g_open_count == 1);
	CHECK(g_rx.length == sizeof(packet));
	CHECK(g_path_was_internal);
	CHECK(g_path_was_terminated);
	CHECK(g_path_length == sizeof(packet) - 1);
	cancel_write();
}

static void test_crs_read_short_packet(void)
{
	uint8_t packet[] = { CRS_READ, 0, 0, 0, 0 };
	reset_observations();
	send_packet(packet, sizeof(packet));
	CHECK(g_open_count == 0);
	CHECK(sent(CRS_NAK, CRS_READ));
}

static void test_crs_write_partial(void)
{
	uint8_t packet[] = { CRS_FILE_DATA, 0, 'a', 'b', 'c' };
	reset_observations();
	begin_write();
	g_short_write = 1;
	send_packet(packet, sizeof(packet));
	CHECK(sent(CRS_NAK, CRS_FILE_DATA));
	CHECK(!sent(CRS_FILE_ACK, 0));
	g_tx_count = 0;
	g_short_write = 0;
	send_packet(packet, sizeof(packet));
	CHECK(sent(CRS_FILE_ACK, 0));
	cancel_write();
}

static void test_crs_write_disk_full(void)
{
	uint8_t packet[] = { CRS_FILE_DATA, 0, 'x' };
	reset_observations();
	begin_write();
	g_write_result = FR_DISK_ERR;
	send_packet(packet, sizeof(packet));
	CHECK(sent(CRS_NAK, CRS_FILE_DATA));
	CHECK(!sent(CRS_FILE_ACK, 0));
	cancel_write();
}

static void test_crs_write_eof_close_fails(void)
{
	uint8_t packet[] = { CRS_FILE_DATA, 0 };
	reset_observations();
	begin_write();
	g_close_result = FR_DISK_ERR;
	send_packet(packet, sizeof(packet));
	CHECK(g_close_count == 1);
	CHECK(sent(CRS_NAK, CRS_FILE_DATA));
	CHECK(!sent(CRS_FILE_ACK, 0));
}

static void test_crs_read_seek_fail(void)
{
	uint8_t packet[] = { CRS_READ, 0, 0, 0, 0, 0, 0, 0, 0, '/', 'R' };
	reset_observations();
	g_seek_result = FR_DISK_ERR;
	send_packet(packet, sizeof(packet));
	CHECK(g_open_count == 1);
	CHECK(g_close_count == 1);
	CHECK(g_release_count == 1);
	CHECK(sent(CRS_NAK, CRS_READ));
	CHECK(!sent(0xf1, CRS_READ));
}

int main(void)
{
	FS_CRS_Init();
	RUN_TEST(test_crs_write_short_path_is_accepted);
	RUN_TEST(test_crs_write_max_length);
	RUN_TEST(test_crs_write_partial);
	RUN_TEST(test_crs_write_disk_full);
	RUN_TEST(test_crs_write_eof_close_fails);
	RUN_TEST(test_crs_read_seek_fail);
	/* The original implementation accepts this malformed request and enters
	 * READ state, so keep it last to preserve isolation of the other cases. */
	RUN_TEST(test_crs_read_short_packet);
	printf("CRS: %d checks, %d failures\n", g_checks, g_failures);
	return g_failures ? 1 : 0;
}
