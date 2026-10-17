// Host copy of usp_zephyr include/zephyr/usp/smtc_sw_platform_helper.h:59-97
// (macros only; same text).  rac_api_mutex is defined in sim/src/hal_fake.c.
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/usp/smtc_zephyr_usp_api.h>
#ifdef __cplusplus
extern "C" {
#endif
void lorawan_smtc_modem_hal_init(const void* transceiver);
#ifdef __cplusplus
}
#endif
#if defined(CONFIG_USP_MAIN_THREAD)
#define SMTC_SW_PLATFORM_INIT() \
    do {                        \
        zephyr_usp_initialization_wait(); \
    } while (0)
#if defined(CONFIG_USP_THREADS_MUTEXES)
#ifdef __cplusplus
extern "C" {
#endif
extern struct k_mutex rac_api_mutex;
#ifdef __cplusplus
}
#endif
#define SMTC_SW_PLATFORM(call)                   \
    ({                                           \
        k_mutex_lock(&rac_api_mutex, K_FOREVER); \
        __auto_type __result = (call);           \
        k_mutex_unlock(&rac_api_mutex);          \
        __result;                                \
    })
#define SMTC_SW_PLATFORM_VOID(call)              \
    ({                                           \
        k_mutex_lock(&rac_api_mutex, K_FOREVER); \
        (call);                                  \
        k_mutex_unlock(&rac_api_mutex);          \
    })
#else
#define SMTC_SW_PLATFORM(call) call
#define SMTC_SW_PLATFORM_VOID(call) call
#endif
#else
static const void* const transceiver = 0;
#define SMTC_SW_PLATFORM_INIT() \
    do {                        \
        lorawan_smtc_modem_hal_init(transceiver); \
    } while (0)
#define SMTC_SW_PLATFORM(call) call
#define SMTC_SW_PLATFORM_VOID(call) call
#endif
