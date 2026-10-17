// Option 2 has no USP thread (CONFIG_USP_MAIN_THREAD=n).  The engine runs
// from the component's engineKick handler; RalSessionSt reports pass counts
// through these hooks so the scenarios print the same keys.
#include "usp_thread.h"
#include <atomic>
std::atomic<uint64_t> g_opt2_passes{0};
std::atomic<uint64_t> g_opt2_bounded_exits{0};
void usp_thread_start() {}
uint64_t usp_thread_passes() { return g_opt2_passes.load(); }
uint64_t usp_thread_no_sleep_iterations() { return g_opt2_bounded_exits.load(); }
