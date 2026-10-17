# Claim G budget arithmetic.  Parameters copied from LinkProfiles.hpp (P5 row, line ~211-231).
# Host arithmetic only: no RF, no target timing.
import sys
BITRATE_P5 = 83333        # bit/s, GMSK P5
PREAMBLE_BITS = 32
SYNC_BITS = 4 * 8
CRC_BITS = 16             # GfskCrcType::BYTE_2
LEN_FIELD_BITS = 8        # assumed packet length field (not in the profile struct; assumption)
for payload in (16, 64, 252):
    bits = PREAMBLE_BITS + SYNC_BITS + LEN_FIELD_BITS + payload * 8 + CRC_BITS
    ms = bits / BITRATE_P5 * 1000
    print(f"payload={payload:4d} B  bits={bits:5d}  airtime_P5={ms:6.2f} ms  (budget for kick latency = this)")
