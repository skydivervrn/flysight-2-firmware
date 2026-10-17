/* Mock of FlySight/state.h: device_state.c reads only device_id. */
#ifndef MOCK_STATE_H_
#define MOCK_STATE_H_
#include <stdint.h>
typedef struct { uint32_t device_id[3]; } FS_State_Data_t;
const FS_State_Data_t *FS_State_Get(void);
#endif
