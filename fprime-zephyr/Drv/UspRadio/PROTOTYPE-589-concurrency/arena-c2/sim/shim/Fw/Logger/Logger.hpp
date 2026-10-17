// Host shim of Fw::Logger::log for RalSessionImpl.cpp.
#ifndef ARENA_C2_FW_LOGGER_HPP
#define ARENA_C2_FW_LOGGER_HPP
#include <cstdarg>
#include <cstdio>
#include <cinttypes>
#include <cerrno>
#include <cstdint>
#include <cstddef>

namespace Fw {
class Logger {
  public:
    static void log(const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        std::vfprintf(stderr, fmt, ap);
        va_end(ap);
    }
};
}  // namespace Fw
#endif
