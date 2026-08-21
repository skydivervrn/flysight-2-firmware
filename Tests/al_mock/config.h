/* Mock config for host-testing FlySight/activelook.c — only the two fields the
 * FSM reads (al_mode picks the mode-ops row, al_rate arms the update timer). */
#ifndef MOCK_CONFIG_H_
#define MOCK_CONFIG_H_

#include <stdint.h>

typedef struct
{
	uint8_t  al_mode;
	uint16_t al_rate;   /* ms between HUD updates */
} FS_Config_Data_t;

const FS_Config_Data_t *FS_Config_Get(void);

#endif
