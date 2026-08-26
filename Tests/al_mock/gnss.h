/* Host mock: the FSM test links activelook.c without the GNSS driver.
 *
 * activelook.c only asks two questions of it — is the newest complete sample
 * too old, and how old — so the mock answers "fresh" and lets the test drive
 * the answer when it needs to. Keeping it in a header avoids a second
 * translation unit for two functions. */
#ifndef AL_MOCK_GNSS_H_
#define AL_MOCK_GNSS_H_

#include <stdbool.h>
#include <stdint.h>

/* Set by a test that wants to exercise the stale branch. */
extern bool g_mockGnssStale;
extern uint32_t g_mockGnssAgeMs;

static inline uint32_t FS_GNSS_MsSinceUpdate(void) { return g_mockGnssAgeMs; }
static inline bool FS_GNSS_IsStale(void) { return g_mockGnssStale; }

#endif /* AL_MOCK_GNSS_H_ */
