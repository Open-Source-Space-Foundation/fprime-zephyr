// Fake SX1262 + fake smtc_modem_hal: the hardware edge of the simulation.
// Everything above this line (sx126x driver, RAL, RALF, radio planner,
// smtc_rac, RalSessionImpl) is real code.
#ifndef ARENA_C2_SIM_CHIP_H
#define ARENA_C2_SIM_CHIP_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- knobs (set before the radio starts) ----
// Modelled SPI clock.  0 = no SPI time cost (race-hunting builds);
// 125000 = the v5e devicetree value (spi-max-frequency = <125000>).
void sim_chip_set_spi_hz(uint32_t hz);
// Time from SetTx to TX_DONE (the fake air), ms.
void sim_chip_set_tx_airtime_ms(uint32_t ms);
// Delay between the chip raising DIO1 and the HAL's radio irq callback
// running on the "system workqueue" thread, us.  Models gpio ISR -> workq hop.
void sim_chip_set_dio1_latency_us(uint32_t us);

// ---- the air ----
// The component calls this right before it applies profile `idx` so the chip
// can latch "which profile the chip is listening on" at SetModulationParams.
void sim_chip_note_profile_being_applied(uint8_t idx);
// Deliver a frame to the chip.  Returns true if the chip was in RX and took
// it (RX_DONE raised); false if the chip was not listening (frame lost).  The
// frame is tagged with the latched profile index so the consumer can tell
// which profile it was actually received on.
bool sim_air_deliver(const uint8_t* data, uint16_t len);
// Profile tag of the frame currently in the chip RX buffer (set at delivery).
uint8_t sim_chip_rx_frame_profile(void);
// Chip's latched profile (what it is configured for right now).
uint8_t sim_chip_profile(void);

// ---- observation ----
typedef enum { SIM_MODE_SLEEP, SIM_MODE_STDBY, SIM_MODE_FS, SIM_MODE_TX, SIM_MODE_RX, SIM_MODE_CW } sim_chip_mode_t;
sim_chip_mode_t sim_chip_mode(void);
struct sim_chip_stats {
    uint64_t spi_transactions;
    uint64_t spi_bytes;
    uint64_t spi_busy_us;        // modelled bus time spent spinning
    uint64_t set_rx, set_tx, set_standby, set_sleep, set_mod_params, read_buffer, write_buffer, clear_irq;
    uint64_t frames_delivered, frames_lost_not_in_rx;
    uint64_t dio1_raised;
    uint64_t rx_overwritten;     // frame delivered while a previous RX_DONE was still unread
    uint64_t fifo_read_us_max, fifo_read_us_sum, fifo_read_n;  // RX_DONE -> ReadBuffer (chip-side service latency)
};
void sim_chip_stats_get(struct sim_chip_stats* out);
void sim_chip_stats_reset(void);
// Per-thread SPI accounting: bytes moved by the calling thread since reset.
uint64_t sim_chip_spi_bytes_this_thread(void);

// ---- fake HAL knobs ----
// 0 = bare (upstream v1.0.0 stub: no serialization),
// 1 = fork bbdd729 (recursive mutex in protect/unprotect_api_call).
void sim_hal_set_protect_mode(int mode);
// Event semaphore given by DIO1 / timer callbacks (smtc_modem_hal_get_event_sem).
struct k_sem* smtc_modem_hal_get_event_sem(void);
// Count of smtc_modem_hal_user_lbm_irq() calls (DIO1 + timer wakeups).
uint64_t sim_hal_wakeups(void);
// Engine-pass vs enqueue overlap counter (claim E).
void sim_overlap_get(uint64_t* engine_calls, uint64_t* enqueue_calls, uint64_t* overlaps);
uint64_t sim_overlap_nested_same_thread(void);
// Force smtc_rac_open_radio to fail (claim B).
void sim_rac_force_open_failure(bool fail);

#ifdef __cplusplus
}
#endif
#endif
