#!/usr/bin/env python3
"""Frame airtime per LinkProfiles.hpp profile = the option-2 RX service budget.

In continuous RX the SX126x writes each new packet from the RX base address, so a
frame left in the buffer is overwritten when the next back-to-back frame ends:
the engine must harvest within one airtime of the *next* frame.  Sizes: 20 B
(single command), 100 B, 252 B (MAX_PACKET_SIZE).  LoRa uses the Semtech AN1200.13
formula (BW125, CR4/5, explicit header, CRC on, LDRO per profile; SF5/6 use the
SX126x +2-symbol variant) and is approximate to ~1 symbol.  GFSK: preamble + sync
+ 1 length byte + payload + 2-byte CRC at the bit rate.
"""
import math
LORA = {'P0': (8, False), 'P1': (10, True), 'P2': (5, False)}
GFSK = {'P3': 38400, 'P4': 75000, 'P5': 83333}
SIZES = [20, 100, 252]

def lora_ms(sf, ldro, pl, pre=8, bw=125e3, cr=1):
    tsym = (2 ** sf) / bw
    de = 1 if ldro else 0
    if sf <= 6:
        num = 8 * pl + 16 - 4 * sf + 20
        npay = 8 + max(math.ceil(num / (4 * sf)), 0) * (cr + 4)
        tpre = (pre + 6.25) * tsym
    else:
        num = 8 * pl - 4 * sf + 28 + 16
        npay = 8 + max(math.ceil(num / (4 * (sf - 2 * de))) * (cr + 4), 0)
        tpre = (pre + 4.25) * tsym
    return (tpre + npay * tsym) * 1e3

def gfsk_ms(br, pl):
    bits = 32 + 32 + 8 + 8 * pl + 16
    return bits / br * 1e3

print('profile\t' + '\t'.join(f'{s}B_ms' for s in SIZES))
for p, (sf, ldro) in LORA.items():
    print(p + '\t' + '\t'.join(f'{lora_ms(sf, ldro, s):.1f}' for s in SIZES))
for p, br in GFSK.items():
    print(p + '\t' + '\t'.join(f'{gfsk_ms(br, s):.1f}' for s in SIZES))
