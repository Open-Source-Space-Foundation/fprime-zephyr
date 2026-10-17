// The bodies single-queue/RalSessionSt.cpp marks "unchanged from
// RalSessionImpl.cpp" and leaves out.  Each is a copy of the cited
// RalSessionImpl.cpp lines with self-> replaced by this->; nothing else.
#include "RalSessionSt.hpp"

extern "C" {
#include <ral_defs.h>
}

namespace Zephyr {

RalSessionSt* RalSessionSt::s_instance = nullptr;

RalSessionSt::RalSessionSt(uint32_t freq_hz, int8_t tx_power_dbm)
    : m_freq_hz(freq_hz), m_tx_power_dbm(tx_power_dbm), m_pktType(RAL_PKT_TYPE_LORA) {
    m_loraPktParams = {};
    m_gfskPktParams = {};
}

// RalSessionImpl.cpp:309-313
void RalSessionSt::restorePktParams() {
    if (m_pktType == RAL_PKT_TYPE_GFSK) {
        ral_set_gfsk_pkt_params(m_ral, &m_gfskPktParams);
    } else {
        ral_set_lora_pkt_params(m_ral, &m_loraPktParams);
    }
}

// RalSessionImpl.cpp:441-472 (onPreTx body), result into m_lastError
void RalSessionSt::programTx() {
    ral_status_t lenStatus;
    if (m_pktType == RAL_PKT_TYPE_GFSK) {
        ral_gfsk_pkt_params_t pkt = m_gfskPktParams;
        pkt.pld_len_in_bytes = static_cast<uint16_t>(m_tx.size);
        lenStatus = ral_set_gfsk_pkt_params(m_ral, &pkt);
    } else {
        ral_lora_pkt_params_t pkt = m_loraPktParams;
        pkt.pld_len_in_bytes = static_cast<uint8_t>(m_tx.size);
        lenStatus = ral_set_lora_pkt_params(m_ral, &pkt);
    }
    if (lenStatus != RAL_STATUS_OK ||
        ral_set_pkt_payload(m_ral, m_tx.data, static_cast<uint16_t>(m_tx.size)) != RAL_STATUS_OK) {
        m_lastError = -EIO;
        smtc_rac_unlock_radio_access(m_radio.id);
        return;
    }
    ral_set_dio_irq_params(m_ral, RAL_IRQ_TX_DONE);
    if (ral_set_tx(m_ral) != RAL_STATUS_OK) {
        m_lastError = -EIO;
        smtc_rac_unlock_radio_access(m_radio.id);
    }
}

// RalSessionImpl.cpp:341-358
void RalSessionSt::harvestRx() {
    uint8_t buf[255];
    uint16_t len = 0;
    ral_handle_rx_done(m_ral);
    ral_get_pkt_payload(m_ral, sizeof buf, buf, &len);
    ral_lora_rx_pkt_status_t lora_status = {};
    ral_gfsk_rx_pkt_status_t gfsk_status = {};
    int16_t rssi = 0;
    int8_t snr = 0;
    if (ral_get_lora_rx_pkt_status(m_ral, &lora_status) == RAL_STATUS_OK) {
        rssi = static_cast<int16_t>(lora_status.rssi_pkt_in_dbm);
        snr = static_cast<int8_t>(lora_status.snr_pkt_in_db);
    } else if (ral_get_gfsk_rx_pkt_status(m_ral, &gfsk_status) == RAL_STATUS_OK) {
        rssi = static_cast<int16_t>(gfsk_status.rssi_avg_in_dbm);
    }
    m_sink->rxFrame(buf, len, rssi, snr);
}

// RalSessionImpl.cpp:200-232
int RalSessionSt::applyLoRa(const LoRaParams& p) {
    ral_set_pkt_type(m_ral, RAL_PKT_TYPE_LORA);
    ral_lora_mod_params_t mod = {};
    mod.sf = static_cast<ral_lora_sf_t>(p.sf);
    mod.bw = RAL_LORA_BW_125_KHZ;
    mod.cr = static_cast<ral_lora_cr_t>(p.cr - 4);
    mod.ldro = p.ldro ? 1 : 0;
    if (ral_set_lora_mod_params(m_ral, &mod) != RAL_STATUS_OK) return -EIO;
    ral_lora_pkt_params_t pkt = {};
    pkt.preamble_len_in_symb = p.preamble_len;
    pkt.header_type = RAL_LORA_PKT_EXPLICIT;
    pkt.pld_len_in_bytes = 0xFF;
    pkt.crc_is_on = true;
    pkt.invert_iq_is_on = false;
    if (ral_set_lora_pkt_params(m_ral, &pkt) != RAL_STATUS_OK) return -EIO;
    m_pktType = RAL_PKT_TYPE_LORA;
    m_loraPktParams = pkt;
    if (ral_set_lora_sync_word(m_ral, static_cast<uint8_t>(p.sync_word)) != RAL_STATUS_OK) return -EIO;
    if (ral_set_rf_freq(m_ral, m_freq_hz) != RAL_STATUS_OK) return -EIO;
    if (ral_set_tx_cfg(m_ral, m_tx_power_dbm, 0) != RAL_STATUS_OK) return -EIO;
    return 0;
}

// RalSessionImpl.cpp:234-292 (pulse-shape / CRC switch reduced to the values the table uses)
int RalSessionSt::applyGfsk(const GfskParams& p) {
    ral_set_pkt_type(m_ral, RAL_PKT_TYPE_GFSK);
    ral_gfsk_mod_params_t mod = {};
    mod.br_in_bps = p.bitrate_bps;
    mod.fdev_in_hz = p.fdev_hz;
    mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_05;
    mod.bw_dsb_in_hz = p.bw_dsb_hz;
    if (ral_set_gfsk_mod_params(m_ral, &mod) != RAL_STATUS_OK) return -EIO;
    ral_gfsk_pkt_params_t pkt = {};
    pkt.preamble_len_in_bits = p.preamble_len_bits;
    pkt.preamble_detector = RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
    pkt.sync_word_len_in_bits = p.sync_word_len * 8;
    pkt.header_type = RAL_GFSK_PKT_VAR_LEN;
    pkt.pld_len_in_bytes = 0xFF;
    pkt.crc_type = RAL_GFSK_CRC_2_BYTES;
    pkt.dc_free = p.whitening ? RAL_GFSK_DC_FREE_WHITENING : RAL_GFSK_DC_FREE_OFF;
    if (ral_set_gfsk_pkt_params(m_ral, &pkt) != RAL_STATUS_OK) return -EIO;
    m_pktType = RAL_PKT_TYPE_GFSK;
    m_gfskPktParams = pkt;
    if (ral_set_gfsk_sync_word(m_ral, p.sync_word, p.sync_word_len) != RAL_STATUS_OK) return -EIO;
    if (ral_set_gfsk_crc_params(m_ral, 0x1D0FU, 0x1021U) != RAL_STATUS_OK) return -EIO;
    if (p.whitening && ral_set_gfsk_whitening_seed(m_ral, 0x01FFU) != RAL_STATUS_OK) return -EIO;
    if (ral_set_rf_freq(m_ral, m_freq_hz) != RAL_STATUS_OK) return -EIO;
    if (ral_set_tx_cfg(m_ral, m_tx_power_dbm, 0) != RAL_STATUS_OK) return -EIO;
    return 0;
}

}  // namespace Zephyr

extern "C" void lorawan_smtc_modem_hal_init(const void* transceiver) {
    (void)transceiver;
    zephyr_usp_initialization_notify();
}
