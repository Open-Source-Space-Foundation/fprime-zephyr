// Host replacement for usp's ralf_sx126x.h.  The radio is faked at the RAL
// driver table (ral_drv_t), the hardware edge: everything above it -- ral.h
// inline dispatch, radio_planner.c, smtc_rac.c, RalSessionImpl.cpp -- is the
// real source.  The fake chip lives in sim/src/chip_fake.c.
#ifndef RALF_SX126X_H__
#define RALF_SX126X_H__
#include "ralf.h"
#ifdef __cplusplus
extern "C" {
#endif
extern const ral_drv_t fake_ral_drv;
#define RALF_SX126X_INSTANTIATE(ctx) \
    { .ral = { .context = (ctx), .driver = FAKE_RAL_DRV_INIT }, .ralf_drv = { 0 } }
#define FAKE_RAL_DRV_INIT                                                                  \
    {                                                                                      \
        .reset = fake_ral_reset, .init = fake_ral_init, .wakeup = fake_ral_wakeup,         \
        .set_sleep = fake_ral_set_sleep, .set_standby = fake_ral_set_standby,              \
        .set_tx = fake_ral_set_tx, .set_rx = fake_ral_set_rx,                              \
        .cfg_rx_boosted = fake_ral_cfg_rx_boosted, .set_tx_cw = fake_ral_set_tx_cw,        \
        .set_tx_cfg = fake_ral_set_tx_cfg, .set_pkt_payload = fake_ral_set_pkt_payload,    \
        .get_pkt_payload = fake_ral_get_pkt_payload, .get_irq_status = fake_ral_get_irq_status, \
        .clear_irq_status = fake_ral_clear_irq_status,                                     \
        .get_and_clear_irq_status = fake_ral_get_and_clear_irq_status,                     \
        .set_dio_irq_params = fake_ral_set_dio_irq_params, .set_rf_freq = fake_ral_set_rf_freq, \
        .set_pkt_type = fake_ral_set_pkt_type, .get_pkt_type = fake_ral_get_pkt_type,      \
        .set_gfsk_mod_params = fake_ral_set_gfsk_mod_params,                               \
        .set_gfsk_pkt_params = fake_ral_set_gfsk_pkt_params,                               \
        .set_lora_mod_params = fake_ral_set_lora_mod_params,                               \
        .set_lora_pkt_params = fake_ral_set_lora_pkt_params,                               \
        .get_gfsk_rx_pkt_status = fake_ral_get_gfsk_rx_pkt_status,                         \
        .get_lora_rx_pkt_status = fake_ral_get_lora_rx_pkt_status,                         \
        .set_gfsk_sync_word = fake_ral_set_gfsk_sync_word,                                 \
        .set_lora_sync_word = fake_ral_set_lora_sync_word,                                 \
        .set_gfsk_crc_params = fake_ral_set_gfsk_crc_params,                               \
        .set_gfsk_whitening_seed = fake_ral_set_gfsk_whitening_seed,                       \
        .get_tx_consumption_in_ua = fake_ral_get_tx_consumption_in_ua,                     \
        .get_gfsk_rx_consumption_in_ua = fake_ral_get_gfsk_rx_consumption_in_ua,           \
        .get_lora_rx_consumption_in_ua = fake_ral_get_lora_rx_consumption_in_ua,           \
        .handle_rx_done = fake_ral_handle_rx_done, .handle_tx_done = fake_ral_handle_tx_done, \
    }
ral_status_t fake_ral_reset(const void* c);
ral_status_t fake_ral_init(const void* c);
ral_status_t fake_ral_wakeup(const void* c);
ral_status_t fake_ral_set_sleep(const void* c, const bool retain);
ral_status_t fake_ral_set_standby(const void* c, ral_standby_cfg_t cfg);
ral_status_t fake_ral_set_tx(const void* c);
ral_status_t fake_ral_set_rx(const void* c, const uint32_t timeout_ms);
ral_status_t fake_ral_cfg_rx_boosted(const void* c, const bool en);
ral_status_t fake_ral_set_tx_cw(const void* c);
ral_status_t fake_ral_set_tx_cfg(const void* c, const int8_t pwr, const uint32_t f);
ral_status_t fake_ral_set_pkt_payload(const void* c, const uint8_t* b, const uint16_t n);
ral_status_t fake_ral_get_pkt_payload(const void* c, uint16_t max, uint8_t* b, uint16_t* n);
ral_status_t fake_ral_get_irq_status(const void* c, ral_irq_t* irq);
ral_status_t fake_ral_clear_irq_status(const void* c, const ral_irq_t irq);
ral_status_t fake_ral_get_and_clear_irq_status(const void* c, ral_irq_t* irq);
ral_status_t fake_ral_set_dio_irq_params(const void* c, const ral_irq_t irq);
ral_status_t fake_ral_set_rf_freq(const void* c, const uint32_t f);
ral_status_t fake_ral_set_pkt_type(const void* c, const ral_pkt_type_t t);
ral_status_t fake_ral_get_pkt_type(const void* c, ral_pkt_type_t* t);
ral_status_t fake_ral_set_gfsk_mod_params(const void* c, const ral_gfsk_mod_params_t* p);
ral_status_t fake_ral_set_gfsk_pkt_params(const void* c, const ral_gfsk_pkt_params_t* p);
ral_status_t fake_ral_set_lora_mod_params(const void* c, const ral_lora_mod_params_t* p);
ral_status_t fake_ral_set_lora_pkt_params(const void* c, const ral_lora_pkt_params_t* p);
ral_status_t fake_ral_get_gfsk_rx_pkt_status(const void* c, ral_gfsk_rx_pkt_status_t* s);
ral_status_t fake_ral_get_lora_rx_pkt_status(const void* c, ral_lora_rx_pkt_status_t* s);
ral_status_t fake_ral_set_gfsk_sync_word(const void* c, const uint8_t* w, const uint8_t n);
ral_status_t fake_ral_set_lora_sync_word(const void* c, const uint8_t w);
ral_status_t fake_ral_set_gfsk_crc_params(const void* c, const uint32_t s, const uint32_t p);
ral_status_t fake_ral_set_gfsk_whitening_seed(const void* c, const uint16_t s);
ral_status_t fake_ral_get_tx_consumption_in_ua(const void* c, const int8_t p, const uint32_t f, uint32_t* ua);
ral_status_t fake_ral_get_gfsk_rx_consumption_in_ua(const void* c, const uint32_t br, const uint32_t bw,
                                                    const bool boost, uint32_t* ua);
ral_status_t fake_ral_get_lora_rx_consumption_in_ua(const void* c, const ral_lora_bw_t bw, const bool boost,
                                                    uint32_t* ua);
ral_status_t fake_ral_handle_rx_done(const void* c);
ral_status_t fake_ral_handle_tx_done(const void* c);
#ifdef __cplusplus
}
#endif
#endif
