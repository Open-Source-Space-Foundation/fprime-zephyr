// Host copy of the SMTC_SW_PLATFORM macros from usp_zephyr v1.0.0
// include/zephyr/usp/smtc_sw_platform_helper.h:60-87, verbatim in shape.
// Option 3 compiles with -DCONFIG_USP_MAIN_THREAD -DCONFIG_USP_THREADS_MUTEXES.
// NOTE: upstream writes `__auto_type`, which GCC rejects in C++ (it is a C-only
// extension) -- see RESULTS.md.  SIM_SW_PLATFORM_CXX_FIX swaps it for `auto`.
#ifndef ARENA_C2_SMTC_SW_PLATFORM_HELPER_H
#define ARENA_C2_SMTC_SW_PLATFORM_HELPER_H
#include <zephyr/kernel.h>
#include <zephyr/usp/smtc_zephyr_usp_api.h>

#if defined(CONFIG_USP_MAIN_THREAD)
#define SMTC_SW_PLATFORM_INIT() do { zephyr_usp_initialization_wait(); } while (0)
#if defined(CONFIG_USP_THREADS_MUTEXES)
#ifdef __cplusplus
extern "C" {
#endif
extern struct k_mutex rac_api_mutex;
#ifdef __cplusplus
}
#endif
#if defined(SIM_SW_PLATFORM_CXX_FIX)
#define SMTC_SW_PLATFORM(call)                     \
    ({                                             \
        k_mutex_lock(&rac_api_mutex, K_FOREVER);   \
        auto __result = (call);                    \
        k_mutex_unlock(&rac_api_mutex);            \
        __result;                                  \
    })
#else
#define SMTC_SW_PLATFORM(call)                     \
    ({                                             \
        k_mutex_lock(&rac_api_mutex, K_FOREVER);   \
        __auto_type __result = (call);             \
        k_mutex_unlock(&rac_api_mutex);            \
        __result;                                  \
    })
#endif
#define SMTC_SW_PLATFORM_VOID(call)                \
    ({                                             \
        k_mutex_lock(&rac_api_mutex, K_FOREVER);   \
        (call);                                    \
        k_mutex_unlock(&rac_api_mutex);            \
    })
#else
#define SMTC_SW_PLATFORM(call) call
#define SMTC_SW_PLATFORM_VOID(call) call
#endif
#else
#define SMTC_SW_PLATFORM_INIT() do { } while (0)
#define SMTC_SW_PLATFORM(call) call
#define SMTC_SW_PLATFORM_VOID(call) call
#endif
#endif
