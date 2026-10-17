// Shared declarations for the #589 host harness.
#pragma once
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#ifdef __cplusplus
extern "C" {
#endif

enum { TIMER_HAL = 0, TIMER_TX_DONE = 1 };
void sim_timer_start(int id, uint32_t delay_us, void (*fn)(void*), void* arg);
void sim_timer_stop(int id);

typedef enum { CHIP_SLEEP, CHIP_STDBY, CHIP_RX, CHIP_TX, CHIP_CW } chip_mode_t;
#define SIG_LORA(sf) (0x10000u | (uint32_t)(sf))
#define SIG_GFSK(br) ((uint32_t)(br))
typedef struct {
    pthread_mutex_t m;
    chip_mode_t mode;
    int pkt_type;
    uint32_t sig;       // modulation currently programmed
    uint32_t rx_sig;    // modulation the live RX was armed with
    uint32_t irq_mask, irq_status;
    uint8_t fifo[256];
    uint16_t fifo_len;
    uint32_t fifo_sig;  // modulation the FIFO frame was received on
    uint32_t last_read_sig;
    uint8_t txbuf[256];
    uint16_t txlen;
    uint32_t tx_airtime_us;
    // counters
    int in_call;
    uint64_t overlapping_calls, ral_calls;
    uint64_t frames_received, frames_missed, rx_arms, tx_count, tx_done_count, cw_count;
    uint64_t tx_from_rx, sleep_from_rx;
    uint64_t rx_us_total, rx_since_us;  // time spent in RX mode

} chip_t;
extern chip_t g_chip;
void chip_init(void);
int chip_air_frame(uint32_t sig, const uint8_t* data, uint16_t len);
uint64_t chip_ral_calls_self(void);
double chip_rx_seconds(void);

// HAL
void hal_fake_radio_irq(void);
void hal_fake_start_usp_thread(void);
extern volatile uint64_t g_engine_passes, g_engine_spins, g_engine_sleeps;
extern volatile int g_panic_count;
extern k_hold_stats_t g_usp_hold;
extern char g_panic_where[128];

#ifdef __cplusplus
}
#endif
