// Extracted verbatim from RalSessionImpl.cpp:200-296 (applyLoRa/applyGfsk), class renamed. Unchanged per the sketch.
int RalSessionSt::applyLoRa(const LoRaParams& p) {
    // 1. Packet type
    ral_set_pkt_type(m_ral, RAL_PKT_TYPE_LORA);

    // 2. Modulation params
    ral_lora_mod_params_t mod = {};
    mod.sf   = static_cast<ral_lora_sf_t>(p.sf);  // RAL_LORA_SF5..SF12
    mod.bw   = RAL_LORA_BW_125_KHZ;               // all v1 profiles
    mod.cr   = static_cast<ral_lora_cr_t>(p.cr - 4);  // 5→CR4/5=0, etc.
    mod.ldro = p.ldro ? 1 : 0;
    if (ral_set_lora_mod_params(m_ral, &mod) != RAL_STATUS_OK) return -EIO;

    // 3. Packet params
    ral_lora_pkt_params_t pkt = {};
    pkt.preamble_len_in_symb = p.preamble_len;
    pkt.header_type          = RAL_LORA_PKT_EXPLICIT;
    pkt.pld_len_in_bytes     = 0xFF;   // max; actual payload set per TX
    pkt.crc_is_on            = true;
    pkt.invert_iq_is_on      = false;
    if (ral_set_lora_pkt_params(m_ral, &pkt) != RAL_STATUS_OK) return -EIO;
    m_pktType       = RAL_PKT_TYPE_LORA;
    m_loraPktParams = pkt;

    // 4. Sync word (1-byte convention: 0x12 = private, 0x34 = public;
    //    ral_set_lora_sync_word takes uint8_t; LinkProfiles stores the value in U16)
    if (ral_set_lora_sync_word(m_ral, static_cast<uint8_t>(p.sync_word)) != RAL_STATUS_OK) return -EIO;

    // 5. Frequency + PA config
    if (ral_set_rf_freq(m_ral, m_freq_hz) != RAL_STATUS_OK) return -EIO;
    if (ral_set_tx_cfg(m_ral, m_tx_power_dbm, 0) != RAL_STATUS_OK) return -EIO;

    return 0;
}

int RalSessionSt::applyGfsk(const GfskParams& p) {
    // 1. Packet type
    ral_set_pkt_type(m_ral, RAL_PKT_TYPE_GFSK);

    // 2. Modulation params
    ral_gfsk_mod_params_t mod = {};
    mod.br_in_bps    = p.bitrate_bps;
    mod.fdev_in_hz   = p.fdev_hz;
    // Map GfskPulseShape enum to RAL pulse shape
    switch (p.pulse_shape) {
        case GfskPulseShape::NONE:  mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_OFF;    break;
        case GfskPulseShape::BT_03: mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_03;  break;
        case GfskPulseShape::BT_05: mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_05;  break;
        case GfskPulseShape::BT_07: mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_07;  break;
        default:                    mod.pulse_shape = RAL_GFSK_PULSE_SHAPE_BT_05;  break;
    }
    mod.bw_dsb_in_hz = p.bw_dsb_hz;
    if (ral_set_gfsk_mod_params(m_ral, &mod) != RAL_STATUS_OK) return -EIO;

    // 3. Packet params
    ral_gfsk_pkt_params_t pkt = {};
    pkt.preamble_len_in_bits     = p.preamble_len_bits;
    pkt.preamble_detector        = RAL_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
    pkt.sync_word_len_in_bits    = p.sync_word_len * 8;
    pkt.header_type              = RAL_GFSK_PKT_VAR_LEN;
    pkt.pld_len_in_bytes         = 0xFF;
    switch (p.crc_type) {
        case GfskCrcType::OFF:    pkt.crc_type = RAL_GFSK_CRC_OFF;    break;
        case GfskCrcType::BYTE_1: pkt.crc_type = RAL_GFSK_CRC_1_BYTE; break;
        case GfskCrcType::BYTE_2: pkt.crc_type = RAL_GFSK_CRC_2_BYTES; break;
        default:                  pkt.crc_type = RAL_GFSK_CRC_2_BYTES; break;
    }
    pkt.dc_free = p.whitening ? RAL_GFSK_DC_FREE_WHITENING : RAL_GFSK_DC_FREE_OFF;
    if (ral_set_gfsk_pkt_params(m_ral, &pkt) != RAL_STATUS_OK) return -EIO;
    m_pktType       = RAL_PKT_TYPE_GFSK;
    m_gfskPktParams = pkt;

    // 4. Sync word
    if (ral_set_gfsk_sync_word(m_ral, p.sync_word, p.sync_word_len) != RAL_STATUS_OK) return -EIO;

    // 4b. CRC seed/polynomial (must be set after pkt_params; not set by ral_set_gfsk_pkt_params).
    //     Standard CCITT values — matches Phase-0 known-good reference (app_per_fsk.h defaults).
    //     seed=0x1D0F, polynomial=0x1021 for RAL_GFSK_CRC_2_BYTES (non-inverted).
    if (pkt.crc_type != RAL_GFSK_CRC_OFF) {
        if (ral_set_gfsk_crc_params(m_ral, 0x1D0FU, 0x1021U) != RAL_STATUS_OK) return -EIO;
    }

    // 4c. Whitening seed (must be set after pkt_params; not set by ral_set_gfsk_pkt_params).
    //     Standard value 0x01FF — matches Phase-0 known-good reference.
    if (p.whitening) {
        if (ral_set_gfsk_whitening_seed(m_ral, 0x01FFU) != RAL_STATUS_OK) return -EIO;
    }

    // 5. Frequency + PA config
    if (ral_set_rf_freq(m_ral, m_freq_hz) != RAL_STATUS_OK) return -EIO;
    if (ral_set_tx_cfg(m_ral, m_tx_power_dbm, 0) != RAL_STATUS_OK) return -EIO;

    return 0;
}

// ---------------------------------------------------------------------------
// startReceive()
// ---------------------------------------------------------------------------
