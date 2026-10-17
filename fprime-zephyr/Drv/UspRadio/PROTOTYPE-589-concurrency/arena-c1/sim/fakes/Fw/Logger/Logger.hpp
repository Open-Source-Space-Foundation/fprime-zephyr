// Host fake of Fw::Logger: printf to stderr when SIM_VERBOSE is set.
#pragma once
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
namespace Fw {
struct Logger {
    static void log(const char* fmt, ...) {
        static const bool on = std::getenv("SIM_VERBOSE") != nullptr;
        logCount()++;
        if (!on) return;
        va_list ap;
        va_start(ap, fmt);
        std::vfprintf(stderr, fmt, ap);
        va_end(ap);
    }
    static int& logCount() { static int n = 0; return n; }
};
}  // namespace Fw
