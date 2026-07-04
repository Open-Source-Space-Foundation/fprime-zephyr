####
# F Prime CMakeLists.txt — Drv/UspRadio (Phase 3: UspRadio component)
#
# SOURCES:
#   - ProfilePolicy.cpp: host-compilable state machine (no USP/Zephyr deps)
#   - UspRadio.cpp: active component implementation (no direct USP headers)
#   - RalSessionImpl.cpp: on-target RAL/RAC implementation (guarded by
#       CONFIG_LORA_BASICS_MODEM_DRIVERS — excluded on v5c/v5d/host builds)
#
# The FPP file generates the component autocoder output (UspRadioComponentAc.*).
# LinkProfiles.fpp is already registered by this same module (Phase 2 carry).
#
# DEPENDS:
#   - default_zephyr-config: provides zephyr-config/LoRaCfg.hpp for freq/power
#   - Zephyr kernel headers in Zephyr builds (included via #ifdef __ZEPHYR__)
####

set(PHASE3_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/ProfilePolicy.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/UspRadio.cpp"
)

# RalSessionImpl compiles only on Zephyr builds with USP enabled.
# The file itself is guarded by #ifdef CONFIG_LORA_BASICS_MODEM_DRIVERS but
# we also exclude it from host builds entirely to avoid linker confusion.
if(DEFINED CONFIG_LORA_BASICS_MODEM_DRIVERS)
    list(APPEND PHASE3_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/RalSessionImpl.cpp"
    )
endif()

register_fprime_library(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/LinkProfiles.fpp"
        "${CMAKE_CURRENT_LIST_DIR}/UspRadio.fpp"
    SOURCES
        ${PHASE3_SOURCES}
    DEPENDS
        default_zephyr-config
)
