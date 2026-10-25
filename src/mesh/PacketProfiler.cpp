// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#include "PacketProfiler.h"

#ifdef PROFILE_PACKETS
namespace prof
{
static const char *const stageNames[NUM_STAGES] = {"api_to_radio", "encode",        "tx_queue", "tx_air",        "isr_latency",
                                                   "rx_handle",    "router_queue",  "router_handle", "decode",   "modules",
                                                   "nodedb_update", "phone_queue",  "api_emit", "log",           "loop_busy",
                                                   "loop_pre",      "loop_threads", "loop_gap",     "xeddsa_sign",   "xeddsa_verify",
                                                   "ack_proof",     "dh_shared"};

struct Stat {
    uint32_t n, sum, max;
};

struct ThreadStat {
    const void *thread;
    const char *name;
    Stat st;
};

struct Pending {
    uint32_t id;
    uint32_t t;
    uint8_t slot;
    bool used;
};

static const uint8_t THREAD_SLOTS = 40, PENDING_SLOTS = 24;
static const uint32_t TAIL_MSEC = 300, IDLE_DUMP_MSEC = 3000;

static Stat stats[NUM_STAGES];
static ThreadStat threads[THREAD_SLOTS];
static Pending pending[PENDING_SLOTS];
static uint32_t windowStart, lastActivity;
volatile uint32_t isrAt;
uint32_t txStartAt;

// Background costs (logs, threads, loop) only count while packets are flowing
static bool inWindow()
{
    return windowStart && millis() - lastActivity < TAIL_MSEC;
}

static void record(Stat &st, uint32_t us)
{
    st.n++;
    st.sum += us;
    if (us > st.max)
        st.max = us;
}

void add(Stage s, uint32_t us)
{
    if (s == LOG || (s >= LOOP_BUSY && s <= LOOP_GAP)) {
        if (!inWindow())
            return;
    } else {
        lastActivity = millis();
        if (!windowStart)
            windowStart = lastActivity;
    }
    record(stats[s], us);
}

void mark(uint32_t id, Slot slot)
{
    Pending *free = nullptr, *oldest = &pending[0];
    for (auto &p : pending) {
        if (!p.used && !free)
            free = &p;
        if (p.used && (int32_t)(p.t - oldest->t) < 0)
            oldest = &p;
    }
    Pending *e = free ? free : oldest;
    *e = {id, micros(), slot, true};
}

void markEnd(uint32_t id, Slot slot, Stage s)
{
    for (auto &p : pending)
        if (p.used && p.id == id && p.slot == slot) {
            p.used = false;
            add(s, micros() - p.t);
            return;
        }
}

void threadRan(const void *thread, const char *name, uint32_t us)
{
    // a run that outlasts the tail (a stall) must still count, so only require an open window
    if (!windowStart)
        return;
    for (auto &t : threads) {
        if (t.thread == thread || !t.thread) {
            t.thread = thread;
            t.name = name;
            record(t.st, us);
            return;
        }
    }
}

static void printStat(const char *kind, const char *name, const Stat &st)
{
    Serial.printf("PROF %s=%s n=%lu avg_us=%lu max_us=%lu total_ms=%lu\r\n", kind, name, (unsigned long)st.n,
                  (unsigned long)(st.sum / st.n), (unsigned long)st.max, (unsigned long)(st.sum / 1000));
}

void loopTick()
{
    if (!windowStart || millis() - lastActivity < IDLE_DUMP_MSEC)
        return;
    Serial.printf("PROF begin window_ms=%lu\r\n", (unsigned long)(lastActivity - windowStart + TAIL_MSEC));
    for (uint8_t i = 0; i < NUM_STAGES; i++)
        if (stats[i].n)
            printStat("stage", stageNames[i], stats[i]);
    for (auto &t : threads)
        if (t.thread && t.st.n)
            printStat("thread", t.name, t.st);
    Serial.printf("PROF end\r\n");
    Serial.flush();
    memset(stats, 0, sizeof(stats));
    memset(threads, 0, sizeof(threads));
    memset(pending, 0, sizeof(pending));
    windowStart = 0;
}
} // namespace prof
#endif
