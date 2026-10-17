// SX1262 + SPI + GPIO model for the host simulation (see sx126x_chip.cpp).
#ifndef CHIP_H
#define CHIP_H
#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#ifdef __cplusplus
#include "fprime-zephyr/Drv/UspRadio/LinkProfiles.hpp"
extern "C" {
#endif
typedef struct {
    int spi_hz, spi_cpu_us, tcxo_ms, wake_busy_us, mode_busy_us, cmd_busy_us, irq_latency_us, spi_poll;
} chip_cfg_t;
extern chip_cfg_t chip_cfg;
typedef struct {
    int64_t cmds, spi_us, spi_bytes, bytes_read;
    int64_t rx_started, rx_complete, rx_aborted, rx_arms, tx_starts, tx_done;
    int64_t rx_missed_not_listening, rx_missed_cfg_changed, rx_overwritten_unread, rx_torn_read;
    int64_t xosc_starts, sleeps, wakes, cmds_lost_asleep, resets;
} chip_stats_t;
extern chip_stats_t chip_stats;
const struct device* chip_device(void);
void chip_inject_frame(int64_t start_us, int profile, int len, uint32_t tag);
double chip_airtime_us(int profile, int len);
int chip_current_profile(void);
int chip_mode(void);
bool chip_is_rx(void);
const char* chip_lost_detail(void);
int64_t chip_last_cmd_us(void);
void chip_raise_irq(uint16_t bits);  /* synthetic stray IRQ flag (stress): sets bits in IrqStatus, DIO1 edge if unmasked */
/* instrumentation */
typedef struct { uint32_t tag; int profile; int64_t done_us, first_spi_us, read_start_us, read_end_us; } chip_rxrec_t;
const chip_rxrec_t* chip_rx_records(int* n);
typedef struct { int64_t t; int code; int profile; } chip_ev_t;  /* 1 RX_LISTEN 2 RX_STOP 3 TX_START 4 TX_DONE 5 SLEEP 6 WAKE */
const chip_ev_t* chip_events(int* n);
#ifdef __cplusplus
}
#endif
#endif
