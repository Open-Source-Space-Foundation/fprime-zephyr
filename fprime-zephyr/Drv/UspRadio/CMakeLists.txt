####
# F Prime CMakeLists.txt — Drv/UspRadio (Phase 2: Link Profile table)
#
# This module is header-only (LinkProfiles.hpp) plus the FPP enum mirror
# (LinkProfiles.fpp).  No .cpp sources are compiled in Phase 2; the
# UspRadio component itself arrives in Phase 3.
#
# The register_fprime_library() call makes the FPP enums visible to the
# autocoder (dictionary generation) and makes the include path available
# to consumers.  This mirrors the pattern used by Drv/LoRa.
####

register_fprime_library(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/LinkProfiles.fpp"
)
