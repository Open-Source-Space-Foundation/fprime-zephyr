// Fake SX1262 behind the RAL driver table, plus the two "interrupt" contexts
// the real board has: a system-workqueue thread that runs the DIO1 callback
// (EVENT_TRIGGER_GLOBAL_THREAD=y on v5e) and a timer thread that fires the
// HAL k_timer callback and TX_DONE.
#include "sim.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ral.h"
#include "ralf_sx126x.h"

// ---------------------------------------------------------------------------
// Deferred-work + timer threads ("ISR" contexts)
// ---------------------------------------------------------------------------
typedef struct {
    uint64_t due_us;
    void (*fn)(void*);
    void* arg;
    int id;
    bool live;
} timer_item_t;

#define MAX_TIMERS 16
static pthread_mutex_t s_tm = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_tc = PTHREAD_COND_INITIALIZER;
static timer_item_t s_timers[MAX_TIMERS];

void sim_timer_start(int id, uint32_t delay_us, void (*fn)(void*), void* arg) {
    pthread_mutex_lock(&s_tm);
    s_timers[id].due_us = k_uptime_us() + delay_us;
    s_timers[id].fn = fn;
    s_timers[id].arg = arg;
    s_timers[id].id = id;
    s_timers[id].live = true;
    pthread_cond_signal(&s_tc);
    pthread_mutex_unlock(&s_tm);
}
void sim_timer_stop(int id) {
    pthread_mutex_lock(&s_tm);
    s_timers[id].live = false;
    pthread_mutex_unlock(&s_tm);
}
static void* timer_thread(void* unused) {
    (void)unused;
    pthread_setname_np("k_timer(isr)");
    pthread_mutex_lock(&s_tm);
    for (;;) {
        uint64_t now = k_uptime_us();
        int due = -1;
        uint64_t next = UINT64_MAX;
        for (int i = 0; i < MAX_TIMERS; i++) {
            if (!s_timers[i].live) continue;
            if (s_timers[i].due_us <= now) { due = i; break; }
            if (s_timers[i].due_us < next) next = s_timers[i].due_us;
        }
        if (due >= 0) {
            s_timers[due].live = false;
            void (*fn)(void*) = s_timers[due].fn;
            void* arg = s_timers[due].arg;
            pthread_mutex_unlock(&s_tm);
            fn(arg);
            pthread_mutex_lock(&s_tm);
            continue;
        }
        if (next == UINT64_MAX) {
            pthread_cond_wait(&s_tc, &s_tm);
        } else {
            uint64_t wait_us = next - now;
            struct timespec dl;
            clock_gettime(CLOCK_REALTIME, &dl);
            uint64_t ns = (uint64_t)dl.tv_nsec + wait_us * 1000ULL;
            dl.tv_sec += (time_t)(ns / 1000000000ULL);
            dl.tv_nsec = (long)(ns % 1000000000ULL);
            pthread_cond_timedwait(&s_tc, &s_tm, &dl);
        }
    }
    return NULL;
}

// sysworkq: a FIFO of DIO1 work items
static struct k_sem s_dio_sem;
static void* sysworkq_thread(void* unused) {
    (void)unused;
    pthread_setname_np("sysworkq(dio1)");
    for (;;) {
        k_sem_take(&s_dio_sem, K_FOREVER);
        hal_fake_radio_irq();  // -> rp_radio_irq_callback (flags + event sem)
    }
    return NULL;
}

// ---------------------------------------------------------------------------
// Chip model
// ---------------------------------------------------------------------------
chip_t g_chip;
static __thread uint64_t t_ral_calls;
uint64_t chip_ral_calls_self(void) { return t_ral_calls; }

static uint32_t s_spi_us;  // modelled per-command SPI+BUSY time

// Attribution for overlapping calls: which thread/function was inside the RAL
// when another thread entered it.
static const char* volatile s_owner_fn;
static char s_owner_thread[32];
static void chip_enter_fn(const char* fn) {
    int n = __atomic_add_fetch(&g_chip.in_call, 1, __ATOMIC_SEQ_CST);
    if (n == 1) {
        s_owner_fn = fn;
        pthread_getname_np(pthread_self(), s_owner_thread, sizeof s_owner_thread);
    }
    if (n > 1) {
        uint64_t k = __atomic_add_fetch(&g_chip.overlapping_calls, 1, __ATOMIC_SEQ_CST);
        if (k <= 20) {
            char me[32];
            pthread_getname_np(pthread_self(), me, sizeof me);
            fprintf(stderr, "OVERLAP\t%s:%s\twhile\t%s:%s\n", me, fn, s_owner_thread, s_owner_fn);
        }
    }
    t_ral_calls++;
    __atomic_add_fetch(&g_chip.ral_calls, 1, __ATOMIC_SEQ_CST);
    if (s_spi_us) k_busy_wait(s_spi_us);
}
static void chip_leave(void) { __atomic_sub_fetch(&g_chip.in_call, 1, __ATOMIC_SEQ_CST); }

static void set_mode_locked(chip_mode_t m) {
    const uint64_t now = k_uptime_us();
    if (g_chip.mode == CHIP_RX && m != CHIP_RX) g_chip.rx_us_total += now - g_chip.rx_since_us;
    if (g_chip.mode != CHIP_RX && m == CHIP_RX) g_chip.rx_since_us = now;
    g_chip.mode = m;
}
double chip_rx_seconds(void) {
    pthread_mutex_lock(&g_chip.m);
    uint64_t t = g_chip.rx_us_total + (g_chip.mode == CHIP_RX ? k_uptime_us() - g_chip.rx_since_us : 0);
    pthread_mutex_unlock(&g_chip.m);
    return t / 1e6;
}
static void raise_irq_locked(ral_irq_t bits) {
    ral_irq_t before = g_chip.irq_status & g_chip.irq_mask;
    g_chip.irq_status |= bits;
    ral_irq_t after = g_chip.irq_status & g_chip.irq_mask;
    if (after && !before) {
        k_sem_give(&s_dio_sem);  // DIO1 rising edge -> GPIO callback -> sysworkq
    }
}

static void tx_done_cb(void* arg) {
    (void)arg;
    pthread_mutex_lock(&g_chip.m);
    if (g_chip.mode == CHIP_TX) {
        set_mode_locked(CHIP_STDBY);
        g_chip.tx_done_count++;
        raise_irq_locked(RAL_IRQ_TX_DONE);
    }
    pthread_mutex_unlock(&g_chip.m);
}

void chip_init(void) {
    memset(&g_chip, 0, sizeof(g_chip));
    pthread_mutex_init(&g_chip.m, NULL);
    g_chip.mode = CHIP_SLEEP;
    g_chip.tx_airtime_us = 25000;
    const char* s = getenv("SIM_SPI_US");
    s_spi_us = s ? (uint32_t)atoi(s) : 20;
    k_sem_init(&s_dio_sem, 0, 1000);
    pthread_t a, b;
    pthread_create(&a, NULL, timer_thread, NULL);
    pthread_create(&b, NULL, sysworkq_thread, NULL);
}

uint32_t chip_sig(void) {
    pthread_mutex_lock(&g_chip.m);
    uint32_t s = g_chip.sig;
    pthread_mutex_unlock(&g_chip.m);
    return s;
}

int chip_air_frame(uint32_t sig, const uint8_t* data, uint16_t len) {
    int ok = 0;
    pthread_mutex_lock(&g_chip.m);
    if (g_chip.mode == CHIP_RX && g_chip.rx_sig == sig) {
        memcpy(g_chip.fifo, data, len);
        g_chip.fifo_len = len;
        g_chip.fifo_sig = sig;
        g_chip.frames_received++;
        raise_irq_locked(RAL_IRQ_RX_DONE);
        ok = 1;
    } else {
        g_chip.frames_missed++;
    }
    pthread_mutex_unlock(&g_chip.m);
    return ok;
}

#define chip_enter() chip_enter_fn(__func__)
#define BODY(stmt)                 \
    chip_enter();                  \
    pthread_mutex_lock(&g_chip.m); \
    stmt;                          \
    pthread_mutex_unlock(&g_chip.m); \
    chip_leave();                  \
    return RAL_STATUS_OK

ral_status_t fake_ral_reset(const void* c) { (void)c; BODY(set_mode_locked(CHIP_STDBY)); }
ral_status_t fake_ral_init(const void* c) { (void)c; BODY((void)0); }
ral_status_t fake_ral_wakeup(const void* c) { (void)c; BODY(if (g_chip.mode == CHIP_SLEEP) set_mode_locked(CHIP_STDBY)); }
ral_status_t fake_ral_set_sleep(const void* c, const bool r) {
    (void)c; (void)r;
    BODY(if (g_chip.mode == CHIP_RX) g_chip.sleep_from_rx++; set_mode_locked(CHIP_SLEEP); g_chip.irq_status = 0);
}
ral_status_t fake_ral_set_standby(const void* c, ral_standby_cfg_t cfg) { (void)c; (void)cfg; BODY(set_mode_locked(CHIP_STDBY)); }
ral_status_t fake_ral_set_tx(const void* c) {
    (void)c;
    chip_enter();
    pthread_mutex_lock(&g_chip.m);
    if (g_chip.mode == CHIP_RX) g_chip.tx_from_rx++;  // anomaly-B precondition
    set_mode_locked(CHIP_TX);
    g_chip.tx_count++;
    pthread_mutex_unlock(&g_chip.m);
    sim_timer_start(TIMER_TX_DONE, g_chip.tx_airtime_us, tx_done_cb, NULL);
    chip_leave();
    return RAL_STATUS_OK;
}
ral_status_t fake_ral_set_rx(const void* c, const uint32_t t) {
    (void)c; (void)t;
    BODY(set_mode_locked(CHIP_RX); g_chip.rx_sig = g_chip.sig; g_chip.rx_arms++);
}
ral_status_t fake_ral_cfg_rx_boosted(const void* c, const bool en) { (void)c; (void)en; BODY((void)0); }
ral_status_t fake_ral_set_tx_cw(const void* c) { (void)c; BODY(set_mode_locked(CHIP_CW); g_chip.cw_count++); }
ral_status_t fake_ral_set_tx_cfg(const void* c, const int8_t p, const uint32_t f) { (void)c; (void)p; (void)f; BODY((void)0); }
ral_status_t fake_ral_set_pkt_payload(const void* c, const uint8_t* b, const uint16_t n) {
    (void)c; BODY(memcpy(g_chip.txbuf, b, n); g_chip.txlen = n);
}
ral_status_t fake_ral_get_pkt_payload(const void* c, uint16_t max, uint8_t* b, uint16_t* n) {
    (void)c;
    BODY(uint16_t k = g_chip.fifo_len < max ? g_chip.fifo_len : max; memcpy(b, g_chip.fifo, k); *n = k;
         g_chip.last_read_sig = g_chip.fifo_sig);
}
ral_status_t fake_ral_get_irq_status(const void* c, ral_irq_t* irq) { (void)c; BODY(*irq = g_chip.irq_status); }
ral_status_t fake_ral_clear_irq_status(const void* c, const ral_irq_t irq) { (void)c; BODY(g_chip.irq_status &= ~irq); }
ral_status_t fake_ral_get_and_clear_irq_status(const void* c, ral_irq_t* irq) {
    (void)c; BODY(*irq = g_chip.irq_status; g_chip.irq_status = 0);
}
ral_status_t fake_ral_set_dio_irq_params(const void* c, const ral_irq_t irq) { (void)c; BODY(g_chip.irq_mask = irq); }
ral_status_t fake_ral_set_rf_freq(const void* c, const uint32_t f) { (void)c; (void)f; BODY((void)0); }
ral_status_t fake_ral_set_pkt_type(const void* c, const ral_pkt_type_t t) { (void)c; BODY(g_chip.pkt_type = t); }
ral_status_t fake_ral_get_pkt_type(const void* c, ral_pkt_type_t* t) { (void)c; BODY(*t = g_chip.pkt_type); }
ral_status_t fake_ral_set_gfsk_mod_params(const void* c, const ral_gfsk_mod_params_t* p) {
    (void)c; BODY(g_chip.sig = SIG_GFSK(p->br_in_bps));
}
ral_status_t fake_ral_set_gfsk_pkt_params(const void* c, const ral_gfsk_pkt_params_t* p) { (void)c; (void)p; BODY((void)0); }
ral_status_t fake_ral_set_lora_mod_params(const void* c, const ral_lora_mod_params_t* p) {
    (void)c; BODY(g_chip.sig = SIG_LORA(p->sf));
}
ral_status_t fake_ral_set_lora_pkt_params(const void* c, const ral_lora_pkt_params_t* p) { (void)c; (void)p; BODY((void)0); }
ral_status_t fake_ral_get_gfsk_rx_pkt_status(const void* c, ral_gfsk_rx_pkt_status_t* s) {
    (void)c; BODY(memset(s, 0, sizeof(*s)); s->rssi_avg_in_dbm = -90);
}
ral_status_t fake_ral_get_lora_rx_pkt_status(const void* c, ral_lora_rx_pkt_status_t* s) {
    (void)c;
    chip_enter();
    pthread_mutex_lock(&g_chip.m);
    ral_status_t st = (g_chip.pkt_type == RAL_PKT_TYPE_LORA) ? RAL_STATUS_OK : RAL_STATUS_ERROR;
    memset(s, 0, sizeof(*s));
    s->rssi_pkt_in_dbm = -80;
    s->snr_pkt_in_db = 5;
    pthread_mutex_unlock(&g_chip.m);
    chip_leave();
    return st;
}
ral_status_t fake_ral_set_gfsk_sync_word(const void* c, const uint8_t* w, const uint8_t n) { (void)c; (void)w; (void)n; BODY((void)0); }
ral_status_t fake_ral_set_lora_sync_word(const void* c, const uint8_t w) { (void)c; (void)w; BODY((void)0); }
ral_status_t fake_ral_set_gfsk_crc_params(const void* c, const uint32_t s, const uint32_t p) { (void)c; (void)s; (void)p; BODY((void)0); }
ral_status_t fake_ral_set_gfsk_whitening_seed(const void* c, const uint16_t s) { (void)c; (void)s; BODY((void)0); }
ral_status_t fake_ral_get_tx_consumption_in_ua(const void* c, const int8_t p, const uint32_t f, uint32_t* ua) {
    (void)c; (void)p; (void)f; *ua = 100000; return RAL_STATUS_OK;
}
ral_status_t fake_ral_get_gfsk_rx_consumption_in_ua(const void* c, const uint32_t br, const uint32_t bw, const bool b,
                                                    uint32_t* ua) {
    (void)c; (void)br; (void)bw; (void)b; *ua = 5000; return RAL_STATUS_OK;
}
ral_status_t fake_ral_get_lora_rx_consumption_in_ua(const void* c, const ral_lora_bw_t bw, const bool b, uint32_t* ua) {
    (void)c; (void)bw; (void)b; *ua = 5000; return RAL_STATUS_OK;
}
ral_status_t fake_ral_handle_rx_done(const void* c) { (void)c; BODY((void)0); }
ral_status_t fake_ral_handle_tx_done(const void* c) { (void)c; BODY((void)0); }
