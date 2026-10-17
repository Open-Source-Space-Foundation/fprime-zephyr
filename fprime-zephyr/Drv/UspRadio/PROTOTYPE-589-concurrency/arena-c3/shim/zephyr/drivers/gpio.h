#ifndef SHIM_GPIO_H
#define SHIM_GPIO_H
#include <stdint.h>
#include <zephyr/device.h>
#ifdef __cplusplus
extern "C" {
#endif
struct gpio_dt_spec { const struct device* port; uint8_t pin; uint16_t dt_flags; };
struct gpio_callback { void* x; };
/* implemented by the chip model */
int chip_gpio_get(const struct gpio_dt_spec* s);
int chip_gpio_set(const struct gpio_dt_spec* s, int v);
static inline int gpio_pin_get_dt(const struct gpio_dt_spec* s) { return chip_gpio_get(s); }
static inline int gpio_pin_set_dt(const struct gpio_dt_spec* s, int v) { return chip_gpio_set(s, v); }
#ifdef __cplusplus
}
#endif
#endif
