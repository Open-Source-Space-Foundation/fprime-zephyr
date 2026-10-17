/* Host stubs for the few usp_zephyr symbols that live in files we do not compile
 * (smtc_modem_hal_dbg_trace.c, smtc_modem_hal_storage.c, Zephyr reboot). */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "zsim.h"

int g_panic_count = 0;
char g_panic_msg[256];
int g_reboot_count = 0;

void smtc_modem_hal_print_trace(const char* fmt, ...) {
    if (!getenv("SIM_TRACE_RP")) return;
    va_list ap; va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap);
}
void smtc_modem_hal_crashlog_store(const uint8_t* s, uint8_t n) {
    g_panic_count++;
    size_t m = n < sizeof(g_panic_msg) - 1 ? n : sizeof(g_panic_msg) - 1;
    memcpy(g_panic_msg, s, m);
    g_panic_msg[m] = 0;
    fprintf(stderr, "PANIC @%lld us: %s\n", (long long)zs_now_us(), g_panic_msg);
}
void sys_reboot(int t) {
    (void)t;
    g_reboot_count++;
    fprintf(stderr, "REBOOT requested @%lld us (after panic: %s)\n", (long long)zs_now_us(), g_panic_msg);
    zs_stop();
    for (;;) k_sleep(K_FOREVER);
}

#include <ctype.h>
/* Copy of smtc_str_trim_end() from usp_zephyr smtc_modem_hal_dbg_trace.c (that file needs Zephyr logging). */
void smtc_str_trim_end(char* text) {
    char* end = text + strlen(text) - 1;
    while (end > text && isspace((unsigned char)*end)) end--;
    end[1] = '\0';
}

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#if defined(CONFIG_USP_MAIN_THREAD) && defined(CONFIG_USP_THREADS_MUTEXES)
/* from smtc_sw_platform_helper.c (not compiled here: it needs devicetree LED nodes) */
K_MUTEX_DEFINE(rac_api_mutex);
#endif
