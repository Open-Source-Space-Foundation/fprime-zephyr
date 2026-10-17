#ifndef SHIM_ZEPHYR_KERNEL_H
#define SHIM_ZEPHYR_KERNEL_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include "zsim.h"
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#define BIT(n) (1UL << (n))
#define CONTAINER_OF(ptr, type, field) ((type*)(((char*)(ptr)) - offsetof(type, field)))
#define ARG_UNUSED(x) (void)(x)
#define K_PRIO_COOP(p) (-1 - (p))
#define K_PRIO_PREEMPT(p) (p)
#define __ASSERT(c, ...) ((void)0)
#define IS_ENABLED(x) 0
#define CLAMP(v, lo, hi) (((v) > (hi)) ? (hi) : (((v) < (lo)) ? (lo) : (v)))
#endif
