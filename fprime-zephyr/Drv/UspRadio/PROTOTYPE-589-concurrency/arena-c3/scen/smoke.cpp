#include "rig.hpp"
static void main_fn(void*) {
    fprintf(stderr,"main_fn\n"); g_rig.build();
    BgLoad bg; bg_start(bg);
    bool ok = g_rig.boot(false);
    fprintf(stderr, "[%lld us] boot ok=%d\n", (long long)zs_now_us(), ok);
    for (int i = 0; i < 6; i++) chip_inject_frame(zs_now_us() + 100000 + i * 150000, 0, 40, 1000 + i);
    k_sleep(K_SECONDS(2));
    fprintf(stderr, "[%lld us] rx delivered=%zu\n", (long long)zs_now_us(), g_rig.rx.size());
    for (auto& r : g_rig.rx) fprintf(stderr, "  rx t=%lld tag=%u prof=%d\n", (long long)r.t, r.tag, r.prof);
    print_run_summary(stderr);
    zs_report(stderr);
    zs_stop();
}
int main() { fprintf(stderr,"start\n"); rig_threads_and_run(main_fn, 30000000); fprintf(stderr,"end now=%lld\n",(long long)zs_now_us()); return 0; }
