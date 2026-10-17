// zsim kernel implementation.  See zsim.h for the model.
#include "zsim.h"

#include <algorithm>
#include <chrono>
#include <unistd.h>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <map>
#include <queue>
#include <string>
#include <thread>
#include <vector>

extern "C" {

struct zs_thread {
    std::string name;
    int base_prio = 0, prio = 0;
    enum St { NEW, READY, RUNNING, BLOCKED, DEAD } st = NEW;
    std::condition_variable_any cv;
    std::thread th;
    k_thread_entry_t entry = nullptr;
    void *p1 = nullptr, *p2 = nullptr, *p3 = nullptr;
    int64_t rseq = 0;
    int64_t cpu_us = 0, slice_used = 0;
    uint64_t wgen = 0;
    bool timed_out = false;
    void* wait_obj = nullptr;
    int wait_kind = 0;  // 1 sem 2 mutex 3 condvar 4 sleep
    std::vector<k_mutex*> held;
};

struct Ev {
    int64_t at;
    uint64_t seq;
    std::function<void()> fn;
    bool operator>(const Ev& o) const { return at != o.at ? at > o.at : seq > o.seq; }
};

struct Reg { const char* name; k_thread_entry_t e; void *p1, *p2, *p3; int prio; };

static struct G_t {
    std::recursive_mutex M;
    std::vector<zs_thread*> all;
    zs_thread* cur = nullptr;
    int64_t now = 0;
    int64_t idle_us = 0;
    int64_t head_seq = 0, tail_seq = 0;
    uint64_t evseq = 0;
    std::priority_queue<Ev, std::vector<Ev>, std::greater<Ev>> events;
    int isr_depth = 0;
    bool stop = false;
    bool started = false;
    std::condition_variable_any done;
    int64_t slice_us = 20000;
    int slice_min_prio = 0;
    FILE* trace = nullptr;
    uint64_t rng = 0x9E3779B97F4A7C15ull;
    // system workqueue
    zs_thread* sysworkq = nullptr;
    struct k_sem wq_sem;
    k_work* wq_head = nullptr;
    k_work* wq_tail = nullptr;
    k_work_handler_t unused = nullptr;
} G;

static std::vector<Reg>& regs() { static std::vector<Reg> r; return r; }
struct MWait { long n = 0; int64_t max_us = 0, sum_us = 0; };
static std::map<std::string, MWait>& mwaits() { static std::map<std::string, MWait> m; return m; }
struct MHold { long n = 0; int64_t max_us = 0, sum_us = 0; };
static std::map<std::string, MHold>& mholds() { static std::map<std::string, MHold> m; return m; }
static std::map<const void*, std::string>& mnames() { static std::map<const void*, std::string> m; return m; }
static std::map<const void*, int64_t>& macq() { static std::map<const void*, int64_t> m; return m; }
void zs_name_mutex(const void* m, const char* name) { mnames()[m] = name; }
static std::string mname(const void* m) { auto it = mnames().find(m); if (it != mnames().end()) return it->second; char b[32]; snprintf(b, sizeof b, "mutex@%p", m); return b; }
static bool g_no_pi = false;
void zs_set_no_pi(int v) { g_no_pi = v; }
static const int64_t TICK_US = 100;  // CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000

typedef std::unique_lock<std::recursive_mutex> Lk;

static void fire_events_due();
static void switch_away(Lk& lk, zs_thread* self);

static zs_thread*& curref() { return G.cur; }

static void mark_ready(zs_thread* t, bool head = false) {
    t->st = zs_thread::READY;
    t->rseq = head ? --G.head_seq : ++G.tail_seq;
}

static zs_thread* pick_best() {
    zs_thread* best = nullptr;
    for (auto* t : G.all) {
        if (t->st != zs_thread::READY) continue;
        if (!best || t->prio < best->prio || (t->prio == best->prio && t->rseq < best->rseq)) best = t;
    }
    return best;
}

static bool higher_ready_than(zs_thread* self) {
    for (auto* t : G.all)
        if (t->st == zs_thread::READY && t->prio < self->prio) return true;
    return false;
}

static void park_forever(Lk& lk, zs_thread* self) {
    for (;;) self->cv.wait(lk, [] { return false; });
}

static void switch_away(Lk& lk, zs_thread* self) {
    for (;;) {
        if (G.stop) park_forever(lk, self);
        zs_thread* nx = pick_best();
        if (nx && getenv("ZS_DEBUG2")) fprintf(stderr, "[%lld] switch %s -> %s\n", (long long)G.now, self->name.c_str(), nx->name.c_str());
        if (nx) {
            nx->st = zs_thread::RUNNING;
            nx->slice_used = 0;
            G.cur = nx;
            if (nx == self) return;
            nx->cv.notify_one();
            self->cv.wait(lk, [&] { return G.cur == self || G.stop; });
            if (G.stop) park_forever(lk, self);
            return;
        }
        if (G.events.empty()) {  // nothing can ever run again
            G.stop = true;
            G.done.notify_all();
            park_forever(lk, self);
        }
        int64_t at = G.events.top().at;
        if (at > G.now) {
            G.idle_us += at - G.now;
            G.now = at;
        }
        fire_events_due();
    }
}

static void fire_events_due() {
    while (!G.events.empty() && G.events.top().at <= G.now && !G.stop) {
        Ev e = G.events.top();
        G.events.pop();
        G.isr_depth++;
        e.fn();
        G.isr_depth--;
    }
}

static void add_event(int64_t at, std::function<void()> fn) {
    G.events.push(Ev{at < G.now ? G.now : at, ++G.evseq, std::move(fn)});
}

// After making a thread ready from thread context: yield if a higher-priority thread is ready.
static void maybe_preempt(Lk& lk, zs_thread* self) {
    if (G.isr_depth > 0 || !self) return;
    if (higher_ready_than(self)) {
        mark_ready(self, true);
        switch_away(lk, self);
    }
}

static void recompute_prio(zs_thread* t) {
    int p = t->base_prio;
    for (auto* m : t->held)
        for (int i = 0; i < m->nwq; i++) p = std::min(p, m->wq[i]->prio);
    t->prio = p;
}

static void thread_body(zs_thread* t) {
    Lk lk(G.M);
    t->cv.wait(lk, [&] { return G.cur == t || G.stop; });
    if (G.stop) park_forever(lk, t);
    lk.unlock();
    t->entry(t->p1, t->p2, t->p3);
    lk.lock();
    t->st = zs_thread::DEAD;
    switch_away(lk, t);
}

static zs_thread* make_thread(const char* name, int prio, k_thread_entry_t e, void* p1, void* p2, void* p3) {
    auto* t = new zs_thread();
    t->name = name ? name : "t";
    t->base_prio = t->prio = prio;
    t->entry = e;
    t->p1 = p1; t->p2 = p2; t->p3 = p3;
    G.all.push_back(t);
    mark_ready(t);
    t->th = std::thread(thread_body, t);
    t->th.detach();
    return t;
}

void zs_init(void) {}

void zs_register_thread(const char* name, k_thread_entry_t entry, void* p1, void* p2, void* p3, int prio) {
    regs().push_back(Reg{name, entry, p1, p2, p3, prio});
}

k_tid_t zs_spawn(const char* name, int prio, k_thread_entry_t entry, void* p1, void* p2, void* p3) {
    Lk lk(G.M);
    zs_thread* t = make_thread(name, prio, entry, p1, p2, p3);
    if (G.started) maybe_preempt(lk, G.cur);
    return t;
}

k_tid_t k_thread_create(struct k_thread* th, k_thread_stack_t*, size_t, k_thread_entry_t entry, void* p1, void* p2,
                        void* p3, int prio, uint32_t, k_timeout_t) {
    k_tid_t t = zs_spawn("k_thread", prio, entry, p1, p2, p3);
    if (th) th->t = t;
    return t;
}
int k_thread_name_set(k_tid_t t, const char* n) { if (t) t->name = n; return 0; }
k_tid_t k_current_get(void) { return G.cur; }

static void sysworkq_main(void*, void*, void*) {
    for (;;) {
        k_sem_take(&G.wq_sem, K_FOREVER);
        k_work* w;
        {
            Lk lk(G.M);
            w = G.wq_head;
            if (!w) continue;
            G.wq_head = w->next;
            if (!G.wq_head) G.wq_tail = nullptr;
            w->pending = 0;
        }
        w->handler(w);
    }
}

int k_work_submit(struct k_work* w) {
    Lk lk(G.M);
    if (w->pending) return 0;
    w->pending = 1;
    w->next = nullptr;
    if (G.wq_tail) G.wq_tail->next = w; else G.wq_head = w;
    G.wq_tail = w;
    lk.unlock();
    k_sem_give(&G.wq_sem);
    return 1;
}

static void watchdog() {
    int64_t last = -1;
    int same = 0;
    for (;;) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        int64_t n = G.now;
        if (n == last && !G.stop) {
            if (++same >= 2) {
                fprintf(stderr, "ZSIM WATCHDOG: virtual time stuck at %lld us. threads:\n", (long long)n);
                for (auto* t : G.all)
                    fprintf(stderr, "  %-16s prio %3d st=%d wait_kind=%d cpu=%lld cur=%d\n", t->name.c_str(), t->prio, (int)t->st, t->wait_kind, (long long)t->cpu_us, G.cur == t);
                fflush(stderr);
                _exit(97);
            }
        } else same = 0;
        last = n;
    }
}

void zs_run(int64_t t_end_us, const char* const* skip) {
    std::thread(watchdog).detach();
    Lk lk(G.M);
    k_sem_init(&G.wq_sem, 0, K_SEM_MAX_LIMIT);
    G.sysworkq = make_thread("sysworkq", -1, sysworkq_main, nullptr, nullptr, nullptr);
    for (auto& r : regs()) {
        bool sk = false;
        for (const char* const* s = skip; s && *s; ++s) if (!strcmp(*s, r.name)) sk = true;
        if (!sk) make_thread(r.name, r.prio, r.e, r.p1, r.p2, r.p3);
    }
    if (getenv("ZS_DEBUG")) fprintf(stderr, "zs_run: %zu registered, %zu threads\n", regs().size(), G.all.size());
    add_event(t_end_us, [] { G.stop = true; G.done.notify_all(); });
    G.started = true;
    zs_thread* nx = pick_best();
    nx->st = zs_thread::RUNNING;
    G.cur = nx;
    nx->cv.notify_one();
    G.done.wait(lk, [] { return G.stop; });
}

void zs_stop(void) {
    Lk lk(G.M);
    G.stop = true;
    G.done.notify_all();
}

int64_t zs_now_us(void) { return G.now; }
bool k_is_in_isr(void) { return G.isr_depth > 0; }
int64_t k_uptime_get(void) { return G.now / 1000; }
uint32_t k_uptime_get_32(void) { return (uint32_t)(G.now / 1000); }
uint32_t k_uptime_seconds(void) { return (uint32_t)(G.now / 1000000); }
const char* zs_thread_name(void) { return G.isr_depth > 0 ? "isr" : (G.cur ? G.cur->name.c_str() : "?"); }
int zs_thread_prio(void) { return G.cur ? G.cur->prio : 99; }
void k_oops(void) { fprintf(stderr, "k_oops at t=%lld us\n", (long long)G.now); abort(); }

void zs_set_timeslice(int64_t us, int min_prio) { G.slice_us = us; G.slice_min_prio = min_prio; }
uint64_t zs_rand(void) {
    uint64_t x = G.rng;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    return G.rng = x;
}
void zs_seed(uint64_t s) { G.rng = s ? s : 1; for (int i = 0; i < 8; i++) zs_rand(); }

void zs_event_at(int64_t at_us, void (*fn)(void*), void* arg) {
    Lk lk(G.M);
    add_event(at_us, [fn, arg] { fn(arg); });
}

void zs_trace_enable(const char* path) { G.trace = path ? fopen(path, "w") : stderr; }
void zs_trace(const char* fmt, ...) {
    if (!G.trace) return;
    Lk lk(G.M);
    va_list ap;
    va_start(ap, fmt);
    fprintf(G.trace, "%10lld %-10s ", (long long)G.now, zs_thread_name());
    vfprintf(G.trace, fmt, ap);
    fputc('\n', G.trace);
    va_end(ap);
}

int64_t zs_thread_cpu_us(const char* name) {
    int64_t s = 0;
    for (auto* t : G.all) if (t->name == name) s += t->cpu_us;
    return s;
}
int64_t zs_idle_us(void) { return G.idle_us; }
void zs_dump_threads(void* f) {
    FILE* o = f ? (FILE*)f : stdout;
    static const char* st[] = {"NEW", "READY", "RUNNING", "BLOCKED", "DEAD"};
    static const char* wk[] = {"-", "sem", "mutex", "condvar", "sleep"};
    for (auto* t : G.all) {
        std::string on;
        if (t->wait_kind == 2 && t->wait_obj) { auto* m = (k_mutex*)t->wait_obj; on = " (owner=" + (m->owner ? m->owner->name : std::string("none")) + ")"; }
        fprintf(o, "  thread %-20s prio %3d %-8s waiting-on=%s%s\n", t->name.c_str(), t->prio, st[t->st], wk[t->wait_kind], on.c_str());
    }
}
void zs_reset_mutex_stats(void) { mwaits().clear(); mholds().clear(); }
void zs_report_mutex(void* f) {
    FILE* o = f ? (FILE*)f : stdout;
    fprintf(o, "mutex contention (waiter, mutex): n, max_ms, mean_ms\n");
    for (auto& kv : mwaits()) fprintf(o, "  %-52s n=%ld max=%.2f mean=%.2f\n", kv.first.c_str(), kv.second.n, kv.second.max_us / 1000.0, kv.second.sum_us / 1000.0 / kv.second.n);
    fprintf(o, "mutex hold times (holder, mutex): n, max_ms, mean_ms\n");
    for (auto& kv : mholds()) fprintf(o, "  %-52s n=%ld max=%.2f mean=%.2f\n", kv.first.c_str(), kv.second.n, kv.second.max_us / 1000.0, kv.second.sum_us / 1000.0 / kv.second.n);
}
double zs_mutex_wait_max_ms(const char* waiter_prefix) {
    int64_t mx = 0;
    for (auto& kv : mwaits()) if (kv.first.compare(0, strlen(waiter_prefix), waiter_prefix) == 0) mx = std::max(mx, kv.second.max_us);
    return mx / 1000.0;
}
long zs_mutex_wait_count(const char* waiter_prefix) {
    long n = 0;
    for (auto& kv : mwaits()) if (kv.first.compare(0, strlen(waiter_prefix), waiter_prefix) == 0) n += kv.second.n;
    return n;
}
double zs_mutex_hold_max_ms(const char* holder_prefix) {
    int64_t mx = 0;
    for (auto& kv : mholds()) if (kv.first.compare(0, strlen(holder_prefix), holder_prefix) == 0) mx = std::max(mx, kv.second.max_us);
    return mx / 1000.0;
}
void zs_report(void* f) {
    FILE* o = f ? (FILE*)f : stdout;
    fprintf(o, "cpu by thread (prio, us):\n");
    for (auto* t : G.all) fprintf(o, "  %-14s prio %3d  %12lld\n", t->name.c_str(), t->base_prio, (long long)t->cpu_us);
    fprintf(o, "  %-14s          %12lld\n", "(idle)", (long long)G.idle_us);
}

void zs_burn(int64_t us) {
    if (us <= 0) return;
    Lk lk(G.M);
    zs_thread* self = G.cur;
    if (G.isr_depth > 0) {  // ISR work: advance time, no preemption
        G.now += us;
        return;
    }
    int64_t rem = us;
    while (rem > 0) {
        int64_t step = rem;
        if (!G.events.empty()) step = std::min(step, std::max<int64_t>(G.events.top().at - G.now, 0));
        bool slicing = G.slice_us > 0 && self->prio >= G.slice_min_prio;
        if (slicing) step = std::min(step, std::max<int64_t>(G.slice_us - self->slice_used, 0));
        if (step > 0) {
            G.now += step;
            self->cpu_us += step;
            self->slice_used += step;
            rem -= step;
        }
        fire_events_due();
        if (higher_ready_than(self)) {
            mark_ready(self, true);
            switch_away(lk, self);
        } else if (slicing && self->slice_used >= G.slice_us) {
            bool peer = false;
            for (auto* t : G.all) if (t != self && t->st == zs_thread::READY && t->prio == self->prio) peer = true;
            self->slice_used = 0;
            if (peer) {
                mark_ready(self, false);
                switch_away(lk, self);
            }
        }
    }
}

void k_busy_wait(uint32_t us) { zs_burn(us); }

// Block current thread until woken (st must be set BLOCKED by caller beforehand via block()).
static void block_self(Lk& lk, zs_thread* self) {
    self->st = zs_thread::BLOCKED;
    switch_away(lk, self);
}

static void arm_timeout(zs_thread* t, int64_t us, void (*onto)(zs_thread*)) {
    uint64_t gen = ++t->wgen;
    add_event(G.now + us, [t, gen, onto] {
        if (t->wgen != gen || t->st != zs_thread::BLOCKED) return;
        t->timed_out = true;
        onto(t);
        mark_ready(t);
    });
}

static void remove_waiter(zs_thread* t) {
    auto rm = [&](zs_thread** wq, int& n) {
        for (int i = 0; i < n; i++)
            if (wq[i] == t) { for (int j = i; j + 1 < n; j++) wq[j] = wq[j + 1]; n--; return; }
    };
    if (t->wait_kind == 1) { auto* s = (k_sem*)t->wait_obj; rm(s->wq, s->nwq); }
    else if (t->wait_kind == 2) {
        auto* m = (k_mutex*)t->wait_obj; rm(m->wq, m->nwq);
        if (m->owner) recompute_prio(m->owner);
    } else if (t->wait_kind == 3) { auto* c = (k_condvar*)t->wait_obj; rm(c->wq, c->nwq); }
    t->wait_obj = nullptr; t->wait_kind = 0;
}

static zs_thread* pop_best_waiter(zs_thread** wq, int& n) {
    int bi = 0;
    for (int i = 1; i < n; i++) if (wq[i]->prio < wq[bi]->prio) bi = i;
    zs_thread* t = wq[bi];
    for (int j = bi; j + 1 < n; j++) wq[j] = wq[j + 1];
    n--;
    return t;
}

int k_sem_init(struct k_sem* s, unsigned init, unsigned limit) {
    s->count = init; s->limit = limit; s->nwq = 0;
    return 0;
}

int k_sem_take(struct k_sem* s, k_timeout_t to) {
    Lk lk(G.M);
    zs_thread* self = G.cur;
    if (s->count > 0) { s->count--; return 0; }
    if (to.us == 0) return -EBUSY;
    if (G.isr_depth > 0) return -EBUSY;
    s->wq[s->nwq++] = self;
    self->wait_obj = s; self->wait_kind = 1; self->timed_out = false;
    if (to.us > 0) arm_timeout(self, to.us, [](zs_thread* t) { remove_waiter(t); });
    else ++self->wgen;
    block_self(lk, self);
    self->wait_obj = nullptr; self->wait_kind = 0;
    return self->timed_out ? -EAGAIN : 0;
}

void k_sem_give(struct k_sem* s) {
    Lk lk(G.M);
    if (s->nwq > 0) {
        zs_thread* w = pop_best_waiter(s->wq, s->nwq);
        ++w->wgen;  // cancel timeout
        w->timed_out = false;
        w->wait_obj = nullptr; w->wait_kind = 0;
        mark_ready(w);
        maybe_preempt(lk, G.cur);
    } else if (s->count < s->limit) {
        s->count++;
    }
}
void k_sem_reset(struct k_sem* s) { Lk lk(G.M); s->count = 0; }
unsigned k_sem_count_get(struct k_sem* s) { return s->count; }

int k_mutex_init(struct k_mutex* m) { m->owner = nullptr; m->lock_count = 0; m->nwq = 0; return 0; }

static void boost_chain(zs_thread* owner, int prio) {
    while (owner && owner->prio > prio) {
        owner->prio = prio;
        if (owner->st == zs_thread::BLOCKED && owner->wait_kind == 2) owner = ((k_mutex*)owner->wait_obj)->owner;
        else break;
    }
}

static int64_t g_kcall_us = 1;  // modelled CPU cost of an uncontended kernel mutex call
void zs_set_kcall_cost(int64_t us) { g_kcall_us = us; }
int k_mutex_lock(struct k_mutex* m, k_timeout_t to) {
    if (G.isr_depth == 0 && G.cur) zs_burn(g_kcall_us);
    Lk lk(G.M);
    zs_thread* self = G.cur;
    if (G.isr_depth > 0) { fprintf(stderr, "k_mutex_lock from ISR\n"); abort(); }
    if (!m->owner) { m->owner = self; m->lock_count = 1; self->held.push_back(m); macq()[m] = G.now; return 0; }
    if (m->owner == self) { m->lock_count++; return 0; }
    if (to.us == 0) return -EBUSY;
    m->wq[m->nwq++] = self;
    self->wait_obj = m; self->wait_kind = 2; self->timed_out = false;
    int64_t t_block = G.now;
    std::string key = self->name + " waits for " + mname(m);
    if (!g_no_pi) boost_chain(m->owner, self->prio);
    if (to.us > 0) arm_timeout(self, to.us, [](zs_thread* t) { remove_waiter(t); });
    else ++self->wgen;
    block_self(lk, self);
    self->wait_obj = nullptr; self->wait_kind = 0;
    { auto& w = mwaits()[key]; w.n++; int64_t d = G.now - t_block; w.sum_us += d; if (d > w.max_us) w.max_us = d; }
    return self->timed_out ? -EAGAIN : 0;
}

int k_mutex_unlock(struct k_mutex* m) {
    if (G.isr_depth == 0 && G.cur) zs_burn(g_kcall_us);
    Lk lk(G.M);
    zs_thread* self = G.cur;
    if (m->owner != self) { fprintf(stderr, "k_mutex_unlock by non-owner\n"); abort(); }
    if (--m->lock_count > 0) return 0;
    { auto& h = mholds()[self->name + " holds " + mname(m)]; int64_t d = G.now - macq()[m]; h.n++; h.sum_us += d; if (d > h.max_us) h.max_us = d; }
    self->held.erase(std::find(self->held.begin(), self->held.end(), m));
    m->owner = nullptr;
    if (m->nwq > 0) {
        zs_thread* w = pop_best_waiter(m->wq, m->nwq);
        ++w->wgen;
        w->timed_out = false;
        m->owner = w; m->lock_count = 1; w->held.push_back(m); macq()[m] = G.now;
        recompute_prio(w);
        mark_ready(w);
    }
    recompute_prio(self);
    maybe_preempt(lk, self);
    return 0;
}

int k_condvar_wait(struct k_condvar* cv, struct k_mutex* m, k_timeout_t to) {
    Lk lk(G.M);
    zs_thread* self = G.cur;
    unsigned saved = m->lock_count;
    // release mutex fully
    m->lock_count = 1;
    lk.unlock();
    k_mutex_unlock(m);
    lk.lock();
    cv->wq[cv->nwq++] = self;
    self->wait_obj = cv; self->wait_kind = 3; self->timed_out = false;
    if (to.us > 0) arm_timeout(self, to.us, [](zs_thread* t) { remove_waiter(t); });
    else ++self->wgen;
    block_self(lk, self);
    self->wait_obj = nullptr; self->wait_kind = 0;
    bool to_ = self->timed_out;
    lk.unlock();
    k_mutex_lock(m, K_FOREVER);
    m->lock_count = saved;
    return to_ ? -EAGAIN : 0;
}
int k_condvar_broadcast(struct k_condvar* cv) {
    Lk lk(G.M);
    while (cv->nwq > 0) {
        zs_thread* w = pop_best_waiter(cv->wq, cv->nwq);
        ++w->wgen; w->timed_out = false; w->wait_obj = nullptr; w->wait_kind = 0;
        mark_ready(w);
    }
    maybe_preempt(lk, G.cur);
    return 0;
}
int k_condvar_signal(struct k_condvar* cv) {
    Lk lk(G.M);
    if (cv->nwq > 0) {
        zs_thread* w = pop_best_waiter(cv->wq, cv->nwq);
        ++w->wgen; w->timed_out = false; w->wait_obj = nullptr; w->wait_kind = 0;
        mark_ready(w);
        maybe_preempt(lk, G.cur);
    }
    return 0;
}

static int64_t tick_ceil(int64_t t) { return (t + TICK_US - 1) / TICK_US * TICK_US; }

static void sleep_until(Lk& lk, zs_thread* self, int64_t wake) {
    uint64_t gen = ++self->wgen;
    self->wait_kind = 4;
    add_event(wake, [self, gen] {
        if (self->wgen != gen || self->st != zs_thread::BLOCKED) return;
        mark_ready(self);
    });
    block_self(lk, self);
    self->wait_kind = 0;
}

int32_t k_sleep(k_timeout_t t) {
    Lk lk(G.M);
    zs_thread* self = G.cur;
    if (t.us < 0) { for (;;) { sleep_until(lk, self, G.now + (int64_t)1e12); } }
    int64_t dur = tick_ceil(t.us);
    if (dur == 0) dur = TICK_US;
    int64_t wake = tick_ceil(G.now) + dur;
    sleep_until(lk, self, wake);
    return 0;
}
int32_t k_msleep(int32_t ms) { return k_sleep(K_MSEC(ms)); }
int32_t k_usleep(int32_t us) { return k_sleep(K_USEC(us)); }

void zs_block_us(int64_t us) {
    Lk lk(G.M);
    zs_thread* self = G.cur;
    if (us <= 0) return;
    sleep_until(lk, self, G.now + us);
}

void k_yield(void) {
    Lk lk(G.M);
    zs_thread* self = G.cur;
    mark_ready(self, false);
    switch_away(lk, self);
}

void k_timer_start(struct k_timer* t, k_timeout_t dur, k_timeout_t) {
    Lk lk(G.M);
    uint64_t gen = ++t->gen;
    add_event(G.now + tick_ceil(dur.us), [t, gen] {
        if (t->gen != gen) return;
        if (t->expiry) t->expiry(t);
    });
}
void k_timer_stop(struct k_timer* t) { Lk lk(G.M); ++t->gen; }

}  // extern "C"
