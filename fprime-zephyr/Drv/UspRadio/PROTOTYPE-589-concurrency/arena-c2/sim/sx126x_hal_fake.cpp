// Fake SX1262 behind the real sx126x driver's 4-function HAL
// (sx126x_hal_write/read/reset/wakeup).  Opcode-level model: modes, IRQ
// status/mask, RX/TX buffer, DIO1, and a per-byte SPI time cost at the v5e
// bus clock.  Mirrors what usp_zephyr drivers/usp/sx126x/sx126x_hal.c does
// around each transaction (busy wait before and after, 500 us after SetSleep).
#include "sim_chip.h"
#include <zephyr/kernel.h>

extern "C" {
#include <sx126x_hal.h>
void sim_hal_radio_irq_fire(void);
}

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>

namespace {

struct Chip {
    std::mutex m;
    sim_chip_mode_t mode = SIM_MODE_STDBY;
    uint16_t irq = 0;
    uint16_t irq_mask = 0;
    uint16_t dio1_mask = 0;
    uint8_t rx_buf[256];
    uint8_t rx_len = 0;
    uint8_t tx_buf[256];
    uint8_t tx_len = 0;
    uint8_t profile = 0xFF;          // latched at SetModulationParams
    uint8_t rx_frame_profile = 0xFF; // tag of frame in rx_buf
    uint64_t tx_generation = 0;
    bool rx_done_unread = false;
    int64_t rx_delivered_us = 0;
    sim_chip_stats stats{};
};
Chip g_chip;
std::atomic<uint32_t> g_spi_hz{0};
std::atomic<uint32_t> g_tx_airtime_ms{5};
std::atomic<uint32_t> g_dio1_latency_us{50};
std::atomic<uint8_t> g_profile_being_applied{0xFF};
thread_local uint64_t t_spi_bytes = 0;

// System-workqueue stand-in: delivers DIO1 edges to the HAL callback.
struct Workq {
    std::mutex m;
    std::condition_variable cv;
    std::deque<int64_t> due_us;  // absolute times
    std::thread th;
    bool started = false;
};
Workq g_wq;

int64_t now_us() {
    static const auto start = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();
}
void spin_us(int64_t us) {
    const int64_t end = now_us() + us;
    while (now_us() < end) {
    }
}

void workq_main() {
    sim_thread_name("sysworkq");
    std::unique_lock<std::mutex> lk(g_wq.m);
    for (;;) {
        g_wq.cv.wait(lk, [] { return !g_wq.due_us.empty(); });
        const int64_t due = g_wq.due_us.front();
        g_wq.due_us.pop_front();
        lk.unlock();
        const int64_t wait = due - now_us();
        if (wait > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(wait));
        }
        sim_hal_radio_irq_fire();  // -> rp_radio_irq_callback on this thread
        lk.lock();
    }
}

// Raise DIO1: the fake "GPIO ISR -> sysworkq" path with a small latency.
void raise_dio1_locked() {
    g_chip.stats.dio1_raised++;
    std::lock_guard<std::mutex> lk(g_wq.m);
    if (!g_wq.started) {
        g_wq.started = true;
        g_wq.th = std::thread(workq_main);
        g_wq.th.detach();
    }
    g_wq.due_us.push_back(now_us() + g_dio1_latency_us.load());
    g_wq.cv.notify_one();
}

const bool g_dbg = std::getenv("SIM_DEBUG") != nullptr;
void account(uint32_t bytes) {
    if (g_dbg) std::fprintf(stderr, "[%s] spi %u B\n", sim_thread_name_get(), bytes);
    g_chip.stats.spi_transactions++;
    g_chip.stats.spi_bytes += bytes;
    t_spi_bytes += bytes;
    const uint32_t hz = g_spi_hz.load();
    if (hz != 0) {
        const int64_t us = static_cast<int64_t>(bytes) * 8 * 1000000 / hz;
        g_chip.stats.spi_busy_us += us;
        spin_us(us);  // PL022 polled mode: the calling thread spins for the transfer
    }
}

void tx_done_later(uint64_t gen) {
    std::thread([gen] {
        sim_thread_name("fake-air-tx");
        std::this_thread::sleep_for(std::chrono::milliseconds(g_tx_airtime_ms.load()));
        std::lock_guard<std::mutex> lk(g_chip.m);
        if (g_chip.mode == SIM_MODE_TX && g_chip.tx_generation == gen) {
            g_chip.mode = SIM_MODE_STDBY;
            g_chip.irq |= 0x0001;  // TX_DONE
            if (g_chip.irq & g_chip.dio1_mask) {
                raise_dio1_locked();
            }
        }
    }).detach();
}

}  // namespace

extern "C" {

void sim_chip_set_spi_hz(uint32_t hz) { g_spi_hz = hz; }
void sim_chip_set_tx_airtime_ms(uint32_t ms) { g_tx_airtime_ms = ms; }
void sim_chip_set_dio1_latency_us(uint32_t us) { g_dio1_latency_us = us; }
void sim_chip_note_profile_being_applied(uint8_t idx) { g_profile_being_applied = idx; }
uint8_t sim_chip_profile(void) {
    std::lock_guard<std::mutex> lk(g_chip.m);
    return g_chip.profile;
}
uint8_t sim_chip_rx_frame_profile(void) {
    std::lock_guard<std::mutex> lk(g_chip.m);
    return g_chip.rx_frame_profile;
}
sim_chip_mode_t sim_chip_mode(void) {
    std::lock_guard<std::mutex> lk(g_chip.m);
    return g_chip.mode;
}
void sim_chip_stats_get(struct sim_chip_stats* out) {
    std::lock_guard<std::mutex> lk(g_chip.m);
    *out = g_chip.stats;
}
void sim_chip_stats_reset(void) {
    std::lock_guard<std::mutex> lk(g_chip.m);
    g_chip.stats = sim_chip_stats{};
    t_spi_bytes = 0;
}
uint64_t sim_chip_spi_bytes_this_thread(void) { return t_spi_bytes; }

bool sim_air_deliver(const uint8_t* data, uint16_t len) {
    std::lock_guard<std::mutex> lk(g_chip.m);
    if (g_chip.mode != SIM_MODE_RX) {
        g_chip.stats.frames_lost_not_in_rx++;
        return false;
    }
    if (g_chip.rx_done_unread) {
        g_chip.stats.rx_overwritten++;  // continuous RX: the next packet overwrites the buffer
    }
    if (len > 255) len = 255;
    std::memcpy(g_chip.rx_buf, data, len);
    g_chip.rx_len = static_cast<uint8_t>(len);
    g_chip.rx_frame_profile = g_chip.profile;
    g_chip.rx_done_unread = true;
    g_chip.rx_delivered_us = now_us();
    g_chip.irq |= 0x0002;  // RX_DONE
    g_chip.stats.frames_delivered++;
    if (g_chip.irq & g_chip.dio1_mask) {
        raise_dio1_locked();
    }
    return true;
}

sx126x_hal_status_t sx126x_hal_write(const void*, const uint8_t* command, const uint16_t command_length,
                                     const uint8_t* data, const uint16_t data_length) {
    account(command_length + data_length);
    std::lock_guard<std::mutex> lk(g_chip.m);
    switch (command[0]) {
        case 0x84:  // SetSleep
            g_chip.mode = SIM_MODE_SLEEP;
            g_chip.stats.set_sleep++;
            if (g_spi_hz.load() != 0) spin_us(500);  // sx126x_hal.c: k_usleep(500) after SetSleep
            break;
        case 0x80:  // SetStandby
            g_chip.mode = SIM_MODE_STDBY;
            g_chip.stats.set_standby++;
            break;
        case 0xC1:  // SetFs
            g_chip.mode = SIM_MODE_FS;
            break;
        case 0x83:  // SetTx
            g_chip.mode = SIM_MODE_TX;
            g_chip.stats.set_tx++;
            g_chip.tx_generation++;
            tx_done_later(g_chip.tx_generation);
            break;
        case 0x82:  // SetRx
            g_chip.mode = SIM_MODE_RX;
            g_chip.stats.set_rx++;
            break;
        case 0xD1:  // SetTxContinuousWave
            g_chip.mode = SIM_MODE_CW;
            break;
        case 0x8B:  // SetModulationParams: latch the profile the component says it is applying
            g_chip.profile = g_profile_being_applied.load();
            g_chip.stats.set_mod_params++;
            break;
        case 0x08:  // SetDioIrqParams
            g_chip.irq_mask = static_cast<uint16_t>((command[1] << 8) | command[2]);
            g_chip.dio1_mask = static_cast<uint16_t>((command[3] << 8) | command[4]);
            break;
        case 0x02:  // ClearIrqStatus
            g_chip.irq &= static_cast<uint16_t>(~((command[1] << 8) | command[2]));
            g_chip.stats.clear_irq++;
            break;
        case 0x0E:  // WriteBuffer
            if (data_length <= 255) {
                std::memcpy(g_chip.tx_buf, data, data_length);
                g_chip.tx_len = static_cast<uint8_t>(data_length);
            }
            g_chip.stats.write_buffer++;
            break;
        default:
            break;
    }
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void*, const uint8_t* command, const uint16_t command_length,
                                    uint8_t* data, const uint16_t data_length) {
    account(command_length + data_length);
    std::lock_guard<std::mutex> lk(g_chip.m);
    std::memset(data, 0, data_length);
    switch (command[0]) {
        case 0x12:  // GetIrqStatus
            if (data_length >= 2) {
                data[0] = static_cast<uint8_t>(g_chip.irq >> 8);
                data[1] = static_cast<uint8_t>(g_chip.irq);
            }
            break;
        case 0x13:  // GetRxBufferStatus
            if (data_length >= 2) {
                data[0] = g_chip.rx_len;
                data[1] = 0;
            }
            break;
        case 0x1E:  // ReadBuffer
            std::memcpy(data, g_chip.rx_buf + command[1], data_length);
            if (g_chip.rx_done_unread) {
                const uint64_t d = static_cast<uint64_t>(now_us() - g_chip.rx_delivered_us);
                g_chip.stats.fifo_read_us_sum += d;
                g_chip.stats.fifo_read_n++;
                if (d > g_chip.stats.fifo_read_us_max) g_chip.stats.fifo_read_us_max = d;
            }
            g_chip.rx_done_unread = false;
            g_chip.stats.read_buffer++;
            break;
        case 0x14:  // GetPacketStatus: rssi = -60 dBm, snr = +8 dB
            if (data_length >= 2) {
                data[0] = 120;
                data[1] = 32;
            }
            break;
        case 0xC0: {  // GetStatus: chip mode bits [6:4]
            uint8_t mode = 0x2;  // STDBY_RC
            if (g_chip.mode == SIM_MODE_TX || g_chip.mode == SIM_MODE_CW) mode = 0x6;
            if (g_chip.mode == SIM_MODE_RX) mode = 0x5;
            if (g_chip.mode == SIM_MODE_FS) mode = 0x4;
            data[0] = static_cast<uint8_t>(mode << 4);
            break;
        }
        default:
            break;
    }
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void*) {
    std::lock_guard<std::mutex> lk(g_chip.m);
    g_chip.mode = SIM_MODE_STDBY;
    g_chip.irq = 0;
    return SX126X_HAL_STATUS_OK;
}
sx126x_hal_status_t sx126x_hal_wakeup(const void*) {
    std::lock_guard<std::mutex> lk(g_chip.m);
    if (g_chip.mode == SIM_MODE_SLEEP) g_chip.mode = SIM_MODE_STDBY;
    return SX126X_HAL_STATUS_OK;
}

}  // extern "C"
