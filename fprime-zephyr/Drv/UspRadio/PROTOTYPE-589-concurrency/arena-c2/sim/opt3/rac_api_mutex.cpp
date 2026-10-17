// usp_zephyr subsys/usp/smtc_sw_platform_helper.c:84-88 — the mutex that
// CONFIG_USP_THREADS_MUTEXES=y defines.
#include <zephyr/kernel.h>
extern "C" { K_MUTEX_DEFINE(rac_api_mutex); }
