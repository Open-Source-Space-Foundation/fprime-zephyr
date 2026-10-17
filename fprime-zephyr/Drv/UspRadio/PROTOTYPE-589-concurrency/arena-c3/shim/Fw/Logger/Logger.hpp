// Fake Fw::Logger for the host sim: prints only when SIM_VERBOSE is set.
#pragma once
#include <cinttypes>
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
namespace Fw {
namespace Logger {
extern int g_log_count;
inline void log(const char* fmt, ...) {
    g_log_count++;
    if (!getenv("SIM_VERBOSE")) return;
    va_list ap; va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap);
}
}  // namespace Logger
}  // namespace Fw
