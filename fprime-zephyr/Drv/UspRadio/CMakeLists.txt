# F Prime CMakeLists.txt — Drv/UspRadio (UHF radio component on Semtech USP)

set(USPRADIO_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/ProfilePolicy.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/UspRadio.cpp"
)

# RalSessionImpl compiles only on Zephyr builds with USP enabled.
if(DEFINED CONFIG_LORA_BASICS_MODEM_DRIVERS)
    list(APPEND USPRADIO_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/RalSessionImpl.cpp"
    )
endif()

register_fprime_library(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/LinkProfiles.fpp"
        "${CMAKE_CURRENT_LIST_DIR}/UspRadio.fpp"
    SOURCES
        ${USPRADIO_SOURCES}
    DEPENDS
        default_zephyr-config
)
