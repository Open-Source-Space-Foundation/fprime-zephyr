/* Force-included ONLY when compiling zephyr_usp_thread.c.  The real loop is
 *   run_engine(); if (smtc_rac_is_irq_flag_pending()) continue; else sleep
 * With no kernel calls inside (upstream v1.2.1 has an empty protect()), one iteration would take zero virtual
 * time and a busy loop could not be simulated.  Charge the modelled CPU cost of one iteration to the flag test. */
#include <stdbool.h>
#include "zsim.h"
bool smtc_rac_is_irq_flag_pending(void);
static inline bool zs_loop_flag_test(void) { zs_burn(1); return smtc_rac_is_irq_flag_pending(); }
#define smtc_rac_is_irq_flag_pending zs_loop_flag_test

/* Optional trace of the loop's sleep decisions (SIM_TRACE_LOOP=1). */
#include <stdio.h>
#include <stdlib.h>
void smtc_modem_hal_interruptible_msleep(k_timeout_t timeout);
static inline void zs_msleep_trace(k_timeout_t t) {
    int64_t t0 = zs_now_us();
    smtc_modem_hal_interruptible_msleep(t);
    if (getenv("SIM_TRACE_LOOP")) fprintf(stderr, "[loop] slept %lld us (asked %lld) woke at %lld\n", (long long)(zs_now_us() - t0), (long long)t.us, (long long)zs_now_us());
}
#define smtc_modem_hal_interruptible_msleep zs_msleep_trace
