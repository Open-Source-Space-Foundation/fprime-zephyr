#include <zephyr/kernel.h>
static inline uint32_t sys_rand32_get(void) { return (uint32_t)zs_rand(); }
