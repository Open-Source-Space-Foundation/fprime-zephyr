/* Force-included when compiling RalSessionImpl.cpp / RalSessionSt.cpp (all variants).
 * FAULT_OPEN=1 makes the call site see smtc_rac_open_radio() return RAC_INVALID_RADIO_ID, which is the
 * "failed init" path of claim B.  Without the env var it forwards to the real function. */
#include <stdlib.h>
#include <smtc_rac_api.h>
static inline uint8_t sim_open_radio(smtc_rac_priority_t p) {
    if (getenv("FAULT_OPEN")) return RAC_INVALID_RADIO_ID;
    return smtc_rac_open_radio(p);
}
#define smtc_rac_open_radio sim_open_radio
