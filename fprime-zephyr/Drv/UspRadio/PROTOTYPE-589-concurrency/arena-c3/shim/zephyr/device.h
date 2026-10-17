#ifndef SHIM_DEVICE_H
#define SHIM_DEVICE_H
#include <stdint.h>
#include <stdbool.h>
struct device { const char* name; const void* config; void* data; };
#ifdef __cplusplus
extern "C" {
#endif
extern struct device zs_dev_obj;
#ifdef __cplusplus
}
#endif
#define DT_CHOSEN(x) x
#define DEVICE_DT_GET(x) (&zs_dev_obj)
static inline bool device_is_ready(const struct device* d) { (void)d; return true; }
#endif
