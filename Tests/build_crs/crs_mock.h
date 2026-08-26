#ifndef TEST_CRS_MOCK_H_
#define TEST_CRS_MOCK_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define CFG_TASK_FS_CRS_UPDATE_ID 3
#define CFG_SCH_PRIO_1 1
#define CFG_TIM_PROC_ID_ISR 0
#define CFG_TS_TICK_VAL 1
#define UTIL_SEQ_RFU 0
#define CUSTOM_STM_FT_PACKET_OUT 7

typedef enum { hw_ts_SingleShot = 0 } HW_TS_Mode_t;
typedef void (*HW_TS_pTimerCb_t)(void);
typedef void (*BLE_TX_Queue_callback_t)(void);
typedef int Custom_STM_Char_Opcode_t;

typedef struct
{
	uint8_t data[244];
	uint8_t length;
} Custom_CRS_Packet_t;

typedef int FRESULT;
#define FR_OK       0
#define FR_DISK_ERR 1
#define FR_NO_FILE  4

typedef unsigned int UINT;
typedef uint32_t FSIZE_t;
typedef char TCHAR;

#define FA_READ          0x01
#define FA_WRITE         0x02
#define FA_CREATE_NEW    0x04
#define FA_CREATE_ALWAYS 0x08

typedef struct { int open; FSIZE_t pos; } FIL;
typedef struct { int open; } DIR;
typedef struct
{
	FSIZE_t fsize;
	uint16_t fdate;
	uint16_t ftime;
	uint8_t fattrib;
	char fname[13];
} FILINFO;

#define f_eof(fp) ((fp)->pos >= 1)

typedef enum { FS_RESOURCE_FATFS = 2 } FS_Resource_t;
typedef enum
{
	FS_RESOURCE_MANAGER_SUCCESS,
	FS_RESOURCE_MANAGER_FAILURE
} FS_ResourceManager_Result_t;

extern uint8_t SizeFt_Packet_Out;

uint8_t *BLE_TX_Queue_GetNextTxPacket(void);
void BLE_TX_Queue_SendNextTxPacket(Custom_STM_Char_Opcode_t opcode,
		uint8_t length, uint8_t *size_ptr, BLE_TX_Queue_callback_t callback);

uint8_t Custom_APP_IsConnected(void);
Custom_CRS_Packet_t *Custom_CRS_GetNextRxPacket(void);

FRESULT f_open(FIL *fp, const TCHAR *path, uint8_t mode);
FRESULT f_close(FIL *fp);
FRESULT f_lseek(FIL *fp, FSIZE_t offset);
FRESULT f_read(FIL *fp, void *buffer, UINT count, UINT *br);
FRESULT f_write(FIL *fp, const void *buffer, UINT count, UINT *bw);
FRESULT f_unlink(const TCHAR *path);
FRESULT f_mkdir(const TCHAR *path);
FRESULT f_opendir(DIR *dp, const TCHAR *path);
FRESULT f_readdir(DIR *dp, FILINFO *fno);
FRESULT f_closedir(DIR *dp);

FS_ResourceManager_Result_t FS_ResourceManager_RequestResource(FS_Resource_t resource);
void FS_ResourceManager_ReleaseResource(FS_Resource_t resource);

void UTIL_SEQ_RegTask(uint32_t task_id, uint32_t flags, void (*callback)(void));
void UTIL_SEQ_SetTask(uint32_t task_id, uint32_t priority);
void HW_TS_Create(uint32_t proc_id, uint8_t *timer_id, HW_TS_Mode_t mode,
		HW_TS_pTimerCb_t callback);
void HW_TS_Start(uint8_t timer_id, uint32_t timeout_ticks);
void HW_TS_Stop(uint8_t timer_id);

#endif
