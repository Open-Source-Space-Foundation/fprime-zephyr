// Host shim of usp_zephyr include/zephyr/usp/smtc_zephyr_usp_api.h (the one
// symbol RalSessionImpl.cpp uses).
#ifndef ARENA_C2_SMTC_ZEPHYR_USP_API_H
#define ARENA_C2_SMTC_ZEPHYR_USP_API_H
#include <stdint.h>
#include <smtc_rac_api.h>
#ifdef __cplusplus
extern "C" {
#endif
extern void zephyr_usp_initialization_wait(void);
#ifdef __cplusplus
}
#endif
#endif
