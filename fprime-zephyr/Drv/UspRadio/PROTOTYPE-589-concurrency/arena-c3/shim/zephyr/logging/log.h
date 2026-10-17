#ifndef SHIM_LOG_H
#define SHIM_LOG_H
#include <stdio.h>
#define LOG_MODULE_DECLARE(...)
#define LOG_MODULE_REGISTER(...)
#define LOG_ERR(...) do { fprintf(stderr, "LOG_ERR: "); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } while (0)
#define LOG_WRN(...) do { } while (0)
#define LOG_INF(...) do { } while (0)
#define LOG_DBG(...) do { } while (0)
#endif
