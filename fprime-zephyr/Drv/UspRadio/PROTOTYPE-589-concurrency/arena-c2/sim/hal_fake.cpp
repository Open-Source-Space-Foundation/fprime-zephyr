// Fake smtc_modem_hal: the parts the radio planner / smtc_rac call.  Shapes
// follow usp_zephyr modules/smtc_modem_hal/smtc_modem_hal.c (fork 0bf3e20):
// k_timer for the RAC timer, a semaphore for the engine wake-up, and
// protect/unprotect either as the upstream no-op stub or the fork's recursive
// mutex (bbdd729), selected at runtime by sim_hal_set_protect_mode().
#include "sim_chip.h"
#include <zephyr/kernel.h>

extern "C" {
#include <smtc_modem_hal.h>
#include <smtc_rac_api.h>
#include <ral_sx126x_bsp.h>
#include <sx126x.h>
#include <radio_planner.h>
}

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
K_SEM_DEFINE(lbm_main_loop_sem, 0, 1);
K_MUTEX_DEFINE(prv_rac_api_mutex);
std::atomic<int> g_protect_mode{1};
std::atomic<uint64_t> g_wakeups{0};
std::atomic<bool> g_force_open_failure{false};

void* g_timer_ctx = nullptr;
void (*g_timer_cb)(void*) = nullptr;
void* g_radio_irq_ctx = nullptr;
void (*g_radio_irq_cb)(void*) = nullptr;

void timer_handler(struct k_timer*) {
    if (g_timer_cb) g_timer_cb(g_timer_ctx);
}
K_TIMER_DEFINE(prv_smtc_modem_hal_timer, timer_handler, NULL);
}  // namespace

extern "C" {

void sim_hal_set_protect_mode(int mode) { g_protect_mode = mode; }
uint64_t sim_hal_wakeups(void) { return g_wakeups.load(); }
void sim_rac_force_open_failure(bool fail) { g_force_open_failure = fail; }
struct k_sem* smtc_modem_hal_get_event_sem(void) { return &lbm_main_loop_sem; }

// Called by the fake chip's sysworkq thread on a DIO1 edge.
void sim_hal_radio_irq_fire(void) {
    if (g_radio_irq_cb) g_radio_irq_cb(g_radio_irq_ctx);
}

// smtc_rac.c is compiled with -Dsmtc_rac_open_radio=sim_real_smtc_rac_open_radio
// so this wrapper can force the INVALID_ID path (claim B).
uint8_t sim_real_smtc_rac_open_radio(smtc_rac_priority_t priority);
#undef smtc_rac_open_radio
uint8_t smtc_rac_open_radio(smtc_rac_priority_t priority) {
    if (g_force_open_failure.load()) return RAC_INVALID_RADIO_ID;
    return sim_real_smtc_rac_open_radio(priority);
}

void zephyr_usp_initialization_wait(void) {}

// ---- engine / enqueue overlap counter (claim E) ----
// smtc_rac.c is compiled with -Drp_callback=sim_wrapped_rp_callback and
// -Drp_task_enqueue=sim_wrapped_rp_task_enqueue.  Each wrapper marks itself
// "inside" and checks whether the other is inside at entry: a nonzero count
// is a measured concurrent execution of an engine pass and a
// lock_radio_access enqueue on two threads, with no lock between them.
#include <pthread.h>
static std::atomic<pthread_t> g_engine_owner{nullptr};
static std::atomic<pthread_t> g_enqueue_owner{nullptr};
static std::atomic<uint64_t> g_overlap_engine_vs_enqueue{0};
static std::atomic<uint64_t> g_overlap_enqueue_vs_engine{0};
static std::atomic<uint64_t> g_engine_calls{0};
static std::atomic<uint64_t> g_enqueue_calls{0};
static std::atomic<uint64_t> g_nested_same_thread{0};
void sim_wrapped_rp_callback(radio_planner_t* rp) {
    g_engine_calls++;
    const pthread_t me = pthread_self();
    const pthread_t q = g_enqueue_owner.load();
    if (q != nullptr && !pthread_equal(q, me)) g_overlap_engine_vs_enqueue++;
    g_engine_owner.store(me);
    rp_callback(rp);
    g_engine_owner.store(nullptr);
}
rp_hook_status_t sim_wrapped_rp_task_enqueue(radio_planner_t* rp, const rp_task_t* task, uint8_t* payload,
                                             uint16_t payload_buffer_size, const rp_radio_params_t* radio_params) {
    g_enqueue_calls++;
    const pthread_t me = pthread_self();
    const pthread_t e = g_engine_owner.load();
    if (e != nullptr) {
        if (pthread_equal(e, me)) g_nested_same_thread++; else g_overlap_enqueue_vs_engine++;
    }
    const pthread_t prev = g_enqueue_owner.exchange(me);
    rp_hook_status_t st = rp_task_enqueue(rp, task, payload, payload_buffer_size, radio_params);
    g_enqueue_owner.store(prev);
    return st;
}
void sim_overlap_get(uint64_t* engine_calls, uint64_t* enqueue_calls, uint64_t* overlaps) {
    *engine_calls = g_engine_calls.load();
    *enqueue_calls = g_enqueue_calls.load();
    *overlaps = g_overlap_engine_vs_enqueue.load() + g_overlap_enqueue_vs_engine.load();
}
uint64_t sim_overlap_nested_same_thread(void) { return g_nested_same_thread.load(); }

// ---- time ----
uint32_t smtc_modem_hal_get_time_in_s(void) { return k_uptime_seconds(); }
uint32_t smtc_modem_hal_get_time_in_ms(void) { return k_uptime_get_32(); }
void smtc_modem_hal_set_offset_to_test_wrapping(const uint32_t) {}
void smtc_modem_hal_interruptible_msleep(k_timeout_t timeout) { k_sem_take(&lbm_main_loop_sem, timeout); }
void smtc_modem_hal_wake_up(void) { k_sem_give(&lbm_main_loop_sem); }
void smtc_modem_hal_user_lbm_irq(void) {
    g_wakeups++;
    smtc_modem_hal_wake_up();
}

// ---- timer ----
void smtc_modem_hal_start_timer(const uint32_t milliseconds, void (*callback)(void*), void* context) {
    g_timer_cb = callback;
    g_timer_ctx = context;
    k_timer_start(&prv_smtc_modem_hal_timer, K_MSEC(milliseconds), K_NO_WAIT);
}
void smtc_modem_hal_stop_timer(void) { k_timer_stop(&prv_smtc_modem_hal_timer); }
void smtc_modem_hal_disable_modem_irq(void) {}
void smtc_modem_hal_enable_modem_irq(void) {}

// ---- radio irq ----
void smtc_modem_hal_irq_config_radio_irq(void (*callback)(void*), void* context) {
    g_radio_irq_ctx = context;
    g_radio_irq_cb = callback;
}
void smtc_modem_hal_radio_irq_clear_pending(void) {}
bool smtc_modem_external_stack_currently_use_radio(void) { return false; }
void smtc_modem_hal_start_radio_tcxo(void) {}
void smtc_modem_hal_stop_radio_tcxo(void) {}
uint32_t smtc_modem_hal_get_radio_tcxo_startup_delay_ms(void) { return 10; }
void smtc_modem_hal_set_ant_switch(bool) {}

// ---- RAC API serialization ----
void smtc_modem_hal_protect_api_call(void) {
    if (std::getenv("SIM_DEBUG")) std::fprintf(stderr, "[%s] protect\n", sim_thread_name_get());
    if (g_protect_mode.load() == 1) k_mutex_lock(&prv_rac_api_mutex, K_FOREVER);
}
void smtc_modem_hal_unprotect_api_call(void) {
    if (g_protect_mode.load() == 1) k_mutex_unlock(&prv_rac_api_mutex);
}

// ---- misc ----
void smtc_modem_hal_reset_mcu(void) {
    std::fprintf(stderr, "[hal] reset_mcu requested\n");
    std::abort();
}
void smtc_modem_hal_reload_wdog(void) {}
void smtc_modem_hal_on_panic(uint8_t* func, uint32_t line, const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    std::fprintf(stderr, "[hal] PANIC %s:%u %s\n", reinterpret_cast<const char*>(func), line, buf);
    std::abort();
}
uint32_t smtc_modem_hal_get_random_nb_in_range(const uint32_t a, const uint32_t b) {
    const uint32_t lo = a < b ? a : b;
    const uint32_t hi = a < b ? b : a;
    return lo + static_cast<uint32_t>(std::rand()) % (hi - lo + 1);
}
void smtc_modem_hal_print_trace(const char* fmt, ...) {
    if (!std::getenv("SIM_TRACE")) return;
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(stderr, fmt, ap);
    va_end(ap);
}
uint8_t smtc_modem_hal_get_battery_level(void) { return 100; }
int8_t smtc_modem_hal_get_board_delay_ms(void) { return 0; }
int8_t smtc_modem_hal_get_temperature(void) { return 25; }
uint16_t smtc_modem_hal_get_voltage_mv(void) { return 3300; }
void smtc_modem_hal_context_restore(const modem_context_type_t, uint32_t, uint8_t*, const uint32_t) {}
void smtc_modem_hal_context_store(const modem_context_type_t, uint32_t, const uint8_t*, const uint32_t) {}
void smtc_modem_hal_context_flash_pages_erase(const modem_context_type_t, uint32_t, uint8_t) {}
void smtc_modem_hal_crashlog_store(const uint8_t*, uint8_t) {}
void smtc_modem_hal_crashlog_restore(uint8_t*, uint8_t*) {}
void smtc_modem_hal_crashlog_set_status(bool) {}
bool smtc_modem_hal_crashlog_get_status(void) { return false; }

// ---- RAL BSP for the SX1262 (values from usp_zephyr sx126x_ral_bsp.c + the v5e DT node) ----
void ral_sx126x_bsp_get_reg_mode(const void*, sx126x_reg_mod_t* reg_mode) { *reg_mode = SX126X_REG_MODE_LDO; }
void ral_sx126x_bsp_get_rf_switch_cfg(const void*, bool* dio2) { *dio2 = false; }
void ral_sx126x_bsp_get_tx_cfg(const void*, const ral_sx126x_bsp_tx_cfg_input_params_t* in,
                               ral_sx126x_bsp_tx_cfg_output_params_t* out) {
    int8_t p = in->system_output_pwr_in_dbm;
    if (p > 22) p = 22;
    if (p < -9) p = -9;
    out->pa_cfg.pa_lut = 0x01;
    out->pa_cfg.device_sel = 0x00;
    out->pa_cfg.hp_max = 0x07;
    out->pa_cfg.pa_duty_cycle = 0x04;
    out->chip_output_pwr_in_dbm_configured = p;
    out->chip_output_pwr_in_dbm_expected = p;
    out->pa_ramp_time = SX126X_RAMP_40_US;
}
void ral_sx126x_bsp_get_xosc_cfg(const void*, ral_xosc_cfg_t* xosc, sx126x_tcxo_ctrl_voltages_t* v,
                                 uint32_t* startup_ticks) {
    *xosc = RAL_XOSC_CFG_TCXO_RADIO_CTRL;
    *v = SX126X_TCXO_CTRL_1_8V;
    *startup_ticks = sx126x_convert_timeout_in_ms_to_rtc_step(10);
}
void ral_sx126x_bsp_get_trim_cap(const void*, uint8_t*, uint8_t*) {}
void ral_sx126x_bsp_get_rx_boost_cfg(const void*, bool* on) { *on = true; }
void ral_sx126x_bsp_get_ocp_value(const void*, uint8_t*) {}
void ral_sx126x_bsp_get_lora_cad_det_peak(const void*, ral_lora_sf_t, ral_lora_bw_t, ral_lora_cad_symbs_t, uint8_t*) {}
ral_status_t ral_sx126x_bsp_get_instantaneous_tx_power_consumption(const void*,
                                                                   const ral_sx126x_bsp_tx_cfg_output_params_t*,
                                                                   sx126x_reg_mod_t, uint32_t* ua) {
    *ua = 100000;
    return RAL_STATUS_OK;
}
ral_status_t ral_sx126x_bsp_get_instantaneous_gfsk_rx_power_consumption(const void*, sx126x_reg_mod_t, bool,
                                                                        uint32_t* ua) {
    *ua = 5000;
    return RAL_STATUS_OK;
}
ral_status_t ral_sx126x_bsp_get_instantaneous_lora_rx_power_consumption(const void*, sx126x_reg_mod_t, bool,
                                                                        uint32_t* ua) {
    *ua = 5000;
    return RAL_STATUS_OK;
}

}  // extern "C"
