// SX1262 chip + SPI bus + GPIO model: the hardware edge for the host simulation.
//
// The REAL usp_zephyr sx126x_hal.c, sx126x_ral_bsp.c, sx126x.c, ral_sx126x.c, ralf_sx126x.c sit above
// this file; they issue genuine SX126x SPI commands through spi_write_dt/spi_transceive_dt and poll
// BUSY through gpio_pin_get_dt.  This file executes those commands against a small behavioural model
// of the chip (modes, XOSC/TCXO start-up, IRQ flags, RX/TX buffers, over-the-air frames).
//
// MODEL INPUTS (assumptions, all overridable through chip_cfg; none measured on this board):
//   spi_hz        125000   board DTS: spi-max-frequency = <125000> (v5e dts, scratch/v5e-upstream-pins-588)
//   spi_cpu_us    15       driver CPU overhead per transfer
//   tcxo_ms       10       board DTS: tcxo-wakeup-time = <10> (DIO3 TCXO); BUSY high while XOSC starts
//   wake_busy_us  340      datasheet t_woff (warm start) - the HAL comment quotes the same figure
//   mode_busy_us  60       BUSY after SetRx/SetTx/SetStandby once XOSC is running
//   cmd_busy_us   8        BUSY after ordinary configuration commands
//   irq_latency_us 25      DIO1 edge -> ISR -> k_work_submit
//   spi_poll      1        pcr-588 build .config: CONFIG_SPI_PL022=y, SPI_PL022_INTERRUPT and SPI_PL022_DMA both unset, so
//                          spi_pl022_transceive_impl() polls the FIFO in the calling thread: a transfer BURNS CPU at the
//                          caller's priority for its whole duration (set CHIP_SPI_POLL=0 to model an IRQ/DMA driver that blocks).
#include "chip.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

extern "C" {
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/usp/lora_lbm_transceiver.h>
#include "sx126x_hal_context.h"
}

chip_cfg_t chip_cfg = {125000, 15, 10, 340, 60, 8, 25, 1};
chip_stats_t chip_stats;

namespace {

enum Mode { SLEEP, STBY_RC, STBY_XOSC, RX, TX, TXCW };

constexpr int PIN_BUSY = 13, PIN_DIO1 = 14, PIN_CS = 1, PIN_TXEN = 21, PIN_RXEN = 22;
constexpr uint16_t IRQ_TXDONE = 1 << 0, IRQ_RXDONE = 1 << 1, IRQ_CRCERR = 1 << 6, IRQ_TIMEOUT = 1 << 9;

struct Frame { int profile; int len; uint32_t tag; };

struct Chip {
    Mode mode = STBY_RC;
    bool xosc = false;
    int64_t busy_until = 0;
    int64_t listen_from = 0;
    uint64_t cfg_epoch = 0;
    int pkt_type = 1;  // 0 GFSK 1 LoRa
    int lora_sf = 8;
    uint32_t gfsk_br_reg = 0;
    int prof = 0;            // profile id decoded from modulation params
    int tx_len = 0;          // from SetPacketParams
    uint8_t buf[256] = {};
    int rx_len = 0;
    bool rx_unread = false;
    uint16_t irq = 0, mask = 0;
    bool dio1 = false;
    bool int_enabled = false;
    event_cb_t cb = nullptr;
    const struct device* dev = nullptr;
    struct k_work work;
    bool woken = true;       // false after SetSleep until CS glitch
    uint64_t cur_rx_epoch = 0;
    int64_t read_busy_until = 0;
    int64_t dio_toggle_cnt = 0;
} C;
std::vector<chip_rxrec_t> g_rxrecs;
std::vector<chip_ev_t> g_chipev;
void cev(int code) { g_chipev.push_back({zs_now_us(), code, C.prof}); }

int g_irq_latency() { return chip_cfg.irq_latency_us; }

double airtime_s(int profile, int len) {
    using namespace Zephyr;
    const LinkProfile& p = LINK_PROFILE_TABLE[profile];
    if (p.mod == ModKind::LORA) {
        const double sf = p.lora.sf, bw = p.lora.bw_hz;
        const double tsym = std::pow(2.0, sf) / bw;
        const int de = p.lora.ldro ? 1 : 0;
        const double npre = p.lora.preamble_len + 4.25;
        const double num = 8.0 * len - 4.0 * sf + 28 + 16 /*crc*/ - 0 /*explicit hdr*/;
        const double den = 4.0 * (sf - 2 * de);
        const double npay = 8 + std::max(std::ceil(num / den) * (p.lora.cr), 0.0);
        return (npre + npay) * tsym;
    }
    const double bits = p.gfsk.preamble_len_bits + 8.0 * p.gfsk.sync_word_len + 8 /*len byte*/ + 8.0 * len + 16 /*crc*/;
    return bits / p.gfsk.bitrate_bps;
}

void raise_dio1_if_edge() {
    bool level = (C.irq & C.mask) != 0;
    if (level && !C.dio1) {
        C.dio1 = true;
        C.dio_toggle_cnt++;
        if (C.int_enabled) {
            zs_event_at(zs_now_us() + g_irq_latency(), [](void*) { k_work_submit(&C.work); }, nullptr);
        }
    } else if (!level) {
        C.dio1 = false;
    }
}

void set_irq(uint16_t bits) { C.irq |= bits; raise_dio1_if_edge(); }

void decode_profile() {
    using namespace Zephyr;
    for (int i = 0; i < LINK_PROFILE_COUNT; i++) {
        const LinkProfile& p = LINK_PROFILE_TABLE[i];
        if (C.pkt_type == 1 && p.mod == ModKind::LORA && p.lora.sf == C.lora_sf) { C.prof = i; return; }
        if (C.pkt_type == 0 && p.mod == ModKind::GFSK && C.gfsk_br_reg) {
            const double br = 32.0 * 32e6 / C.gfsk_br_reg;
            if (std::fabs(br - p.gfsk.bitrate_bps) < 0.01 * p.gfsk.bitrate_bps) { C.prof = i; return; }
        }
    }
}

void bump_epoch() { C.cfg_epoch++; }

void end_tx(void* arg) {
    uint64_t ep = (uint64_t)(uintptr_t)arg;
    if (C.mode == TX && C.cfg_epoch == ep) {
        C.mode = STBY_XOSC;  // default fallback after TX
        chip_stats.tx_done++;
        cev(4);
        set_irq(IRQ_TXDONE);
    }
}

void frame_end(void* arg) {
    auto* f = (Frame*)arg;
    uint64_t ep = (uint64_t)f->tag >> 32;  // epoch snapshot stored in high bits
    (void)ep;
    delete f;
}

struct AirEv { Frame f; uint64_t epoch; int64_t end_us; };

void air_end(void* arg) {
    auto* a = (AirEv*)arg;
    if (C.mode == RX && C.cfg_epoch == a->epoch && C.prof == a->f.profile) {
        if (C.rx_unread) chip_stats.rx_overwritten_unread++;
        if (zs_now_us() < C.read_busy_until) chip_stats.rx_torn_read++;
        C.rx_len = a->f.len;
        memset(C.buf, 0, sizeof(C.buf));
        // payload byte 0..3: tag (little-endian); byte 4: profile the frame was sent on
        memcpy(C.buf, &a->f.tag, 4);
        C.buf[4] = (uint8_t)a->f.profile;
        C.rx_unread = true;
        chip_stats.rx_complete++;
        {
            uint32_t tg; memcpy(&tg, C.buf, 4);
            g_rxrecs.push_back({tg, a->f.profile, zs_now_us(), 0, 0, 0});
        }
        set_irq(IRQ_RXDONE);
    } else {
        chip_stats.rx_missed_cfg_changed++;
    }
    delete a;
}

void air_start(void* arg) {
    auto* f = (Frame*)arg;
    int64_t now = zs_now_us();
    if (C.mode == RX && now >= C.listen_from && C.prof == f->profile) {
        auto* a = new AirEv{*f, C.cfg_epoch, 0};
        int64_t dur = (int64_t)(airtime_s(f->profile, f->len) * 1e6);
        zs_event_at(now + dur, air_end, a);
        chip_stats.rx_started++;
    } else {
        chip_stats.rx_missed_not_listening++;
    }
    delete f;
}

void busy_for(int64_t us) { C.busy_until = std::max(C.busy_until, zs_now_us()) + us; }

void ensure_xosc() {
    if (!C.xosc) {
        C.xosc = true;
        busy_for((int64_t)chip_cfg.tcxo_ms * 1000 + 100);
        chip_stats.xosc_starts++;
    }
}

// execute one command at the END of its SPI transfer
void exec(const uint8_t* cmd, size_t clen, const uint8_t* wdata, size_t wlen, uint8_t* rd, size_t rlen) {
    const uint8_t op = cmd[0];
    // gather write payload = cmd[1..] ++ wdata
    std::vector<uint8_t> a(cmd + 1, cmd + clen);
    a.insert(a.end(), wdata, wdata + wlen);
    chip_stats.cmds++;
    int64_t cb = chip_cfg.cmd_busy_us;
    switch (op) {
        case 0x80:  // SetStandby
            bump_epoch();
            if (C.mode == RX) { chip_stats.rx_aborted++; cev(2); }
            if (a.size() && a[0] == 1) { C.mode = STBY_XOSC; ensure_xosc(); busy_for(chip_cfg.mode_busy_us); }
            else { C.mode = STBY_RC; C.xosc = false; busy_for(cb); }
            break;
        case 0x84:  // SetSleep
            bump_epoch();
            if (C.mode == RX) cev(2);
            C.mode = SLEEP; C.xosc = false; C.woken = false; chip_stats.sleeps++;
            break;
        case 0x82: {  // SetRx
            bump_epoch();
            bool was_xosc = C.xosc;
            ensure_xosc();
            (void)was_xosc;
            busy_for(chip_cfg.mode_busy_us);
            C.mode = RX;
            C.listen_from = C.busy_until;
            chip_stats.rx_arms++;
            cev(1);
            break;
        }
        case 0x83: {  // SetTx
            bump_epoch();
            ensure_xosc();
            busy_for(chip_cfg.mode_busy_us);
            C.mode = TX;
            chip_stats.tx_starts++;
            cev(3);
            int len = C.tx_len ? C.tx_len : 1;
            int64_t dur = (int64_t)(airtime_s(C.prof, len) * 1e6);
            // the PA ramps only after XOSC/TCXO start-up: TX airtime starts when BUSY drops (busy_until), not at the SPI write
            if (!getenv("CHIP_TX_HANG")) zs_event_at(C.busy_until + dur, end_tx, (void*)(uintptr_t)C.cfg_epoch);  // HWIL anomaly B: chip hung in TX, no TX_DONE
            break;
        }
        case 0xD1: bump_epoch(); ensure_xosc(); C.mode = TXCW; busy_for(chip_cfg.mode_busy_us); break;
        case 0x8A: C.pkt_type = a.size() ? a[0] : 1; decode_profile(); busy_for(cb); break;
        case 0x8B:  // SetModulationParams
            if (C.pkt_type == 1 && a.size() >= 1) C.lora_sf = a[0];
            else if (a.size() >= 3) C.gfsk_br_reg = (a[0] << 16) | (a[1] << 8) | a[2];
            decode_profile(); busy_for(cb); break;
        case 0x8C:  // SetPacketParams
            if (C.pkt_type == 1 && a.size() >= 4) C.tx_len = a[3];
            else if (C.pkt_type == 0 && a.size() >= 7) C.tx_len = a[6];
            busy_for(cb); break;
        case 0x0E:  // WriteBuffer(offset, data)
            if (a.size() >= 1) for (size_t i = 1; i < a.size() && a[0] + i - 1 < 256; i++) C.buf[a[0] + i - 1] = a[i];
            busy_for(cb); break;
        case 0x08:  // SetDioIrqParams
            if (a.size() >= 4) { C.mask = (a[2] << 8) | a[3]; }
            raise_dio1_if_edge(); busy_for(cb); break;
        case 0x02:  // ClearIrqStatus
            if (a.size() >= 2) C.irq &= ~((a[0] << 8) | a[1]);
            raise_dio1_if_edge(); busy_for(cb); break;
        case 0x12:  // GetIrqStatus
            if (rlen >= 2) { rd[0] = C.irq >> 8; rd[1] = C.irq & 0xFF; }
            break;
        case 0x13:  // GetRxBufferStatus
            if (rlen >= 2) { rd[0] = (uint8_t)C.rx_len; rd[1] = 0; }
            break;
        case 0x14:  // GetPacketStatus
            if (rlen >= 3) { rd[0] = 0x60; rd[1] = 0x60; rd[2] = 0x60; }
            break;
        case 0x1E:  // ReadBuffer(offset, nop) data
            if (rlen) {
                for (size_t i = 0; i < rlen; i++) rd[i] = C.buf[(cmd[1] + i) & 0xFF];
                C.rx_unread = false;
                chip_stats.bytes_read += rlen;
            }
            break;
        case 0xC0:  // GetStatus
            if (rlen) rd[0] = (C.mode == RX ? 5 : C.mode == TX ? 6 : 3) << 4;
            break;
        case 0x17: if (rlen >= 2) { rd[0] = rd[1] = 0; } break;  // GetDeviceErrors
        case 0x15: if (rlen) rd[0] = 0x60; break;
        default: busy_for(cb); break;
    }
    if (rlen && (op != 0x12 && op != 0x13 && op != 0x14 && op != 0x1E && op != 0xC0 && op != 0x17 && op != 0x15))
        memset(rd, 0, rlen);
}

}  // namespace

extern "C" {

static int64_t s_last_cmd = 0;
static char s_lost[256];
static int s_lostn = 0;
static void chip_note_lost(uint8_t op) {
    if (s_lostn < 6) {
        int n = strlen(s_lost);
        snprintf(s_lost + n, sizeof(s_lost) - n, "%s%s:%02x@%lld", n ? "," : "", zs_thread_name(), op, (long long)zs_now_us());
    }
    s_lostn++;
}
const char* chip_lost_detail(void) { return s_lost; }
int64_t chip_last_cmd_us(void) { return s_last_cmd; }
void chip_raise_irq(uint16_t bits) { set_irq(bits); }
const chip_rxrec_t* chip_rx_records(int* n) { *n = (int)g_rxrecs.size(); return g_rxrecs.data(); }
const chip_ev_t* chip_events(int* n) { *n = (int)g_chipev.size(); return g_chipev.data(); }

int chip_gpio_get(const struct gpio_dt_spec* s) {
    if (s->pin == PIN_BUSY) {
        if (C.mode == SLEEP) return 1;
        return zs_now_us() < C.busy_until ? 1 : 0;
    }
    if (s->pin == PIN_DIO1) return C.dio1;
    return 0;
}

int chip_gpio_set(const struct gpio_dt_spec* s, int v) {
    if (getenv("CHIP_TRACE")) fprintf(stderr, "[%lld] %s gpio_set pin=%d v=%d mode=%d\n", (long long)zs_now_us(), zs_thread_name(), s->pin, v, (int)C.mode);
    if (s->pin == 6 && v == 0) {  // NRESET released: chip restarts in STDBY_RC
        C.mode = STBY_RC; C.xosc = false; C.irq = 0; C.mask = 0; C.dio1 = false; C.woken = true; C.cfg_epoch++;
        C.busy_until = zs_now_us() + 3500;
        chip_stats.resets++;
    }
    if (s->pin == PIN_CS && v == 1 && C.mode == SLEEP) {  // wake-up glitch on NSS
        C.mode = STBY_RC;
        C.woken = true;
        C.busy_until = zs_now_us() + chip_cfg.wake_busy_us;
        chip_stats.wakes++;
    }
    return 0;
}

static int chip_spi_transceive_locked(const struct spi_buf_set* tx, const struct spi_buf_set* rx);
static K_SEM_DEFINE(s_bus, 1, 1);  // spi_context_lock(): a binary k_sem, K_FOREVER, NO priority inheritance
int chip_spi_transceive(const struct spi_dt_spec*, const struct spi_buf_set* tx, const struct spi_buf_set* rx) {
    k_sem_take(&s_bus, K_FOREVER);
    int r = chip_spi_transceive_locked(tx, rx);
    k_sem_give(&s_bus);
    return r;
}
static int chip_spi_transceive_locked(const struct spi_buf_set* tx, const struct spi_buf_set* rx) {
    size_t total = 0;
    uint8_t cmd[16];
    size_t clen = 0;
    const uint8_t* wd = nullptr;
    size_t wl = 0;
    for (size_t i = 0; i < tx->count; i++) total += tx->buffers[i].len;
    if (tx->buffers[0].len > sizeof(cmd)) return -1;
    clen = tx->buffers[0].len;
    memcpy(cmd, tx->buffers[0].buf, clen);
    if (tx->count > 1) { wd = (const uint8_t*)tx->buffers[1].buf; wl = wd ? tx->buffers[1].len : 0; }
    uint8_t* rd = nullptr;
    size_t rl = 0;
    if (rx && rx->count > 1) { rd = (uint8_t*)rx->buffers[1].buf; rl = rx->buffers[1].len; }
    int64_t dur = (int64_t)total * 8 * 1000000 / chip_cfg.spi_hz;
    bool is_rd = (cmd[0] == 0x1E);
    if (is_rd) C.read_busy_until = zs_now_us() + dur;
    for (auto it = g_rxrecs.rbegin(); it != g_rxrecs.rend(); ++it) {
        if (it->first_spi_us == 0) it->first_spi_us = zs_now_us();
        else break;
    }
    int64_t t_op_start = zs_now_us();
    chip_stats.spi_us += dur;
    chip_stats.spi_bytes += total;
    zs_burn(chip_cfg.spi_cpu_us);
    if (chip_cfg.spi_poll) zs_burn(dur);   // polled FIFO loop: CPU busy, preemptible by higher priorities
    else zs_block_us(dur);                 // IRQ/DMA driver: CPU released while the bus clocks
    if (C.mode == SLEEP && !C.woken) {  // command into a sleeping chip is ignored
        chip_stats.cmds_lost_asleep++;
        chip_note_lost(cmd[0]);
        if (rd) memset(rd, 0, rl);
        return 0;
    }
    if (getenv("CHIP_TRACE")) fprintf(stderr, "[%lld] %s spi op=%02x clen=%zu wlen=%zu rlen=%zu mode=%d\n", (long long)zs_now_us(), zs_thread_name(), cmd[0], clen, wl, rl, (int)C.mode);
    if (cmd[0] == 0x1E && rl) {  // ReadBuffer of a received frame: note start/end for the matching record
        uint32_t tg; memcpy(&tg, C.buf, 4);
        for (auto it = g_rxrecs.rbegin(); it != g_rxrecs.rend(); ++it)
            if (it->tag == tg && it->read_end_us == 0) { it->read_start_us = t_op_start; it->read_end_us = zs_now_us(); break; }
    }
    s_last_cmd = zs_now_us();
    exec(cmd, clen, wd, wl, rd, rl);
    return 0;
}

void lora_transceiver_board_attach_interrupt(const struct device* dev, event_cb_t cb) { C.cb = cb; C.dev = dev; }
void lora_transceiver_board_enable_interrupt(const struct device*) { C.int_enabled = true; }
void lora_transceiver_board_disable_interrupt(const struct device*) { C.int_enabled = false; }
uint32_t lora_transceiver_get_tcxo_startup_delay_ms(const struct device*) { return chip_cfg.tcxo_ms; }
int32_t lora_transceiver_get_model(const struct device*) { return 0; }

static void work_cb(struct k_work*) {
    if (C.cb) C.cb(C.dev);
}

// ---- device objects mirroring the v5e DTS node ----
static struct sx126x_hal_context_data_t s_data;
static struct sx126x_hal_context_cfg_t s_cfg;
struct device zs_dev_obj;
#define s_dev zs_dev_obj
static struct device s_gpio_dev;
static struct device s_spi_dev;

const struct device* chip_device(void) {
    static bool inited = false;
    if (!inited) {
        inited = true;
        s_gpio_dev.name = "gpio0";
        s_spi_dev.name = "spi1";
        memset(&s_cfg, 0, sizeof(s_cfg));
        s_cfg.spi.bus = &s_spi_dev;
        s_cfg.spi.config.frequency = chip_cfg.spi_hz;
        s_cfg.spi.config.cs.gpio = {&s_gpio_dev, PIN_CS, 0};
        s_cfg.reset = {&s_gpio_dev, 6, 0};
        s_cfg.busy = {&s_gpio_dev, PIN_BUSY, 0};
        s_cfg.dio1 = {&s_gpio_dev, PIN_DIO1, 0};
#ifdef NO_RF_SWITCH
        static sx126x_pa_pwr_cfg_t pa_tbl[32];
        for (int i = 0; i < 32; i++) { pa_tbl[i].power = (int8_t)(i - 9); pa_tbl[i].pa_duty_cycle = 0x04; pa_tbl[i].hp_max = 0x07; }
        s_cfg.pa_cfg_table = pa_tbl;  // v1.2.1 takes the PA table from the config (DT in the real driver)
#else
        s_cfg.tx_enable = {&s_gpio_dev, PIN_TXEN, 0};
        s_cfg.rx_enable = {&s_gpio_dev, PIN_RXEN, 0};
#endif
        s_cfg.tcxo_cfg.xosc_cfg = RAL_XOSC_CFG_TCXO_RADIO_CTRL;
        s_cfg.tcxo_cfg.voltage = SX126X_TCXO_CTRL_1_8V;
        s_cfg.tcxo_cfg.wakeup_time_ms = chip_cfg.tcxo_ms;
        s_cfg.capa_xta = 0xFF;
        s_cfg.capa_xtb = 0xFF;
        s_cfg.reg_mode = SX126X_REG_MODE_LDO;
        s_cfg.rx_boosted = true;
        s_cfg.pa_ramp_time = (sx126x_ramp_time_t)0x02;
        s_data.radio_status = RADIO_AWAKE;
        s_dev.name = "sx1262_usp";
        s_dev.config = &s_cfg;
        s_dev.data = &s_data;
        C.work.handler = work_cb;
        C.work.pending = 0;
    }
    return &s_dev;
}

void chip_inject_frame(int64_t start_us, int profile, int len, uint32_t tag) {
    auto* f = new Frame{profile, len, tag};
    zs_event_at(start_us, air_start, f);
}

double chip_airtime_us(int profile, int len) { return airtime_s(profile, len) * 1e6; }

int chip_current_profile(void) { return C.prof; }
int chip_mode(void) { return (int)C.mode; }
bool chip_is_rx(void) { return C.mode == RX; }

}  // extern "C"
