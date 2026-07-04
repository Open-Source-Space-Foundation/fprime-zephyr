# ======================================================================
# LinkProfiles.fpp
# FPP mirror of the versioned Link Profile table for dictionary visibility.
#
# These enums drive Phase 3 UspRadio command arguments; keep minimal —
# only what the component surface actually needs.  The authoritative RF
# parameters live in LinkProfiles.hpp; this file provides the F' type
# system layer so GDS/YAMCS can label profile indices by name.
#
# Append-only: never renumber existing enum values (ADR 0002).
# ======================================================================

module Zephyr {

    @ Profile index enumeration — matches LINK_PROFILE_TABLE[] positions in LinkProfiles.hpp.
    @ Append-only; renaming existing members requires a table version bump.
    enum LinkProfileId : U8 {
        @ LoRa SF8/125k/CR4:5 — boot default; parity with legacy LoRa link
        P0_LORA_SF8  = 0
        @ LoRa SF10/125k — long-range / link-degraded fallback
        P1_LORA_SF10 = 1
        @ LoRa SF5/125k — fast LoRa (~15 kbps); SX126x-only spreading factor
        P2_LORA_SF5  = 2
        @ GFSK 38400 bps, fdev 20 kHz, BT=0.5 — conservative bulk downlink
        P3_GFSK_38K  = 3
        @ GFSK 75000 bps, fdev 25 kHz, BT=0.5 — max-rate bulk downlink (OBW 112.5 kHz)
        P4_GFSK_75K  = 4
    }

    @ Modulation kind tag — mirrors ModKind in LinkProfiles.hpp
    enum LinkModKind : U8 {
        LORA = 0
        GFSK = 1
    }

}
