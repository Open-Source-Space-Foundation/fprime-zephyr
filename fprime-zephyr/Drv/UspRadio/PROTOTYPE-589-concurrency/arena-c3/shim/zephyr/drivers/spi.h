#ifndef SHIM_SPI_H
#define SHIM_SPI_H
#include <stdint.h>
#include <stddef.h>
#include <zephyr/drivers/gpio.h>
#ifdef __cplusplus
extern "C" {
#endif
struct spi_cs_control { struct gpio_dt_spec gpio; };
struct spi_config { uint32_t frequency; uint16_t operation; struct spi_cs_control cs; };
struct spi_dt_spec { const struct device* bus; struct spi_config config; };
struct spi_buf { void* buf; size_t len; };
struct spi_buf_set { const struct spi_buf* buffers; size_t count; };
int chip_spi_transceive(const struct spi_dt_spec* s, const struct spi_buf_set* tx, const struct spi_buf_set* rx);
static inline int spi_write_dt(const struct spi_dt_spec* s, const struct spi_buf_set* tx) { return chip_spi_transceive(s, tx, 0); }
static inline int spi_transceive_dt(const struct spi_dt_spec* s, const struct spi_buf_set* tx, const struct spi_buf_set* rx) { return chip_spi_transceive(s, tx, rx); }
#ifdef __cplusplus
}
#endif
#endif
