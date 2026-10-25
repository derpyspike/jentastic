// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#pragma once

#include <Arduino.h>

// Per-packet path profiler, compiled in with -DPROFILE_PACKETS. Stats are printed to the USB serial as plain
// "PROF ..." lines once packet activity has been idle for a few seconds, then reset.
#ifdef PROFILE_PACKETS
namespace prof
{
enum Stage : uint8_t {
    API_TO_RADIO,
    ENCODE,
    TX_QUEUE,
    TX_AIR,
    ISR_LATENCY,
    RX_HANDLE,
    ROUTER_QUEUE,
    ROUTER_HANDLE,
    DECODE,
    MODULES,
    NODEDB_UPDATE,
    PHONE_QUEUE,
    API_EMIT,
    LOG,
    LOOP_BUSY,
    LOOP_PRE,
    LOOP_THREADS,
    LOOP_GAP,
    XEDDSA_SIGN,
    XEDDSA_VERIFY,
    ACK_PROOF,
    DH_SHARED,
    NUM_STAGES
};

enum Slot : uint8_t { SLOT_TXQ, SLOT_RQ, SLOT_PQ };

void add(Stage s, uint32_t us);
void mark(uint32_t id, Slot slot);
void markEnd(uint32_t id, Slot slot, Stage s);
void threadRan(const void *thread, const char *name, uint32_t us);
void loopTick();

extern volatile uint32_t isrAt;
extern uint32_t txStartAt;

struct Scope {
    Stage s;
    uint32_t t0;
    explicit Scope(Stage s) : s(s), t0(micros()) {}
    ~Scope() { add(s, micros() - t0); }
};
} // namespace prof
#define PROF_SCOPE(s) prof::Scope _profScope(prof::s)
#define PROF_DO(x) x
#else
#define PROF_SCOPE(s)
#define PROF_DO(x)
#endif
