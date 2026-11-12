// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez

#include "NexusExtStorage.h"

#if NEXUS_ENABLE_EXTERNAL_FLASH && defined(ARCH_NRF52)

#include "ExternalFlash.h"
#include "UptimeClock.h"
#include "concurrency/Lock.h"
#include "concurrency/LockGuard.h"
#include "mesh/Throttle.h"
#include <littlefs/lfs.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <strings.h>

namespace nexus::extstorage
{
namespace
{
constexpr uint32_t BLOCK_SIZE = 4096;
constexpr uint32_t PROG_SIZE = 256;
constexpr uint32_t READ_SIZE = 16;
constexpr uint32_t EXT_BYTES = 512u * 1024;
constexpr uint8_t ERASE_BLOCKS_PER_STEP = 2;
constexpr uint32_t PROBE_RETRY_MS = 30000;

enum State : uint8_t { ST_UNCHECKED, ST_ERASING, ST_FORMATTING, ST_READY, ST_CORRUPT, ST_DISABLED };

struct Partition {
    const char *name;
    uint32_t base;
    uint32_t bytes;
    lfs_t lfs;
    lfs_config cfg;
    State state;
    bool mounted;
    bool eraseSuppressed;
    uint32_t eraseNext;
    uint32_t eraseTotal;
};

Partition parts[2] = {
    {"ext", 0, 0, {}, {}, ST_UNCHECKED, false, false, 0, 0},
    {"sec", 0, 0, {}, {}, ST_UNCHECKED, false, false, 0, 0},
};

bool started = false;
uint32_t nextProbeRetry = 0;

concurrency::Lock &fsLock()
{
    static concurrency::Lock lock;
    return lock;
}

void appendf(char *out, size_t cap, size_t &pos, const char *fmt, ...) __attribute__((format(printf, 4, 5)));
void appendf(char *out, size_t cap, size_t &pos, const char *fmt, ...)
{
    if (pos >= cap)
        return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out + pos, cap - pos, fmt, ap);
    va_end(ap);
    if (n > 0)
        pos = ((size_t)n < cap - pos) ? pos + (size_t)n : cap - 1;
}

int fsRead(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    const auto *p = static_cast<const Partition *>(c->context);
    return extflash::read(p->base + (uint32_t)block * BLOCK_SIZE + (uint32_t)off, buffer, size) ? 0 : LFS_ERR_IO;
}

int fsProg(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    const auto *p = static_cast<const Partition *>(c->context);
    return extflash::program(p->base + (uint32_t)block * BLOCK_SIZE + (uint32_t)off, buffer, size) ? 0 : LFS_ERR_IO;
}

int fsErase(const struct lfs_config *c, lfs_block_t block)
{
    const auto *p = static_cast<const Partition *>(c->context);
    if (p->eraseSuppressed)
        return 0;
    return extflash::erase4k(p->base + (uint32_t)block * BLOCK_SIZE) ? 0 : LFS_ERR_IO;
}

int fsSync(const struct lfs_config *) { return 0; }

void initPartitions()
{
    const uint32_t cap = extflash::capacity();
    parts[0].base = 0;
    parts[0].bytes = cap < EXT_BYTES ? cap : EXT_BYTES;
    parts[1].base = parts[0].bytes;
    parts[1].bytes = cap > parts[0].bytes ? cap - parts[0].bytes : 0;

    for (auto &p : parts) {
        if (p.mounted) {
            (void)lfs_unmount(&p.lfs);
            p.mounted = false;
        }
        p.cfg.context = &p;
        p.cfg.read = fsRead;
        p.cfg.prog = fsProg;
        p.cfg.erase = fsErase;
        p.cfg.sync = fsSync;
        p.cfg.read_size = READ_SIZE;
        p.cfg.prog_size = PROG_SIZE;
        p.cfg.block_size = BLOCK_SIZE;
        p.cfg.block_count = p.bytes / BLOCK_SIZE;
        p.cfg.lookahead = 128;
        p.cfg.read_buffer = nullptr;
        p.cfg.prog_buffer = nullptr;
        p.cfg.lookahead_buffer = nullptr;
        p.cfg.file_buffer = nullptr;
        p.mounted = false;
        p.eraseSuppressed = false;
        p.eraseNext = 0;
        p.eraseTotal = p.cfg.block_count;
        p.state = (p.cfg.block_count >= 8) ? ST_UNCHECKED : ST_DISABLED;
    }
}

bool ensureProbe()
{
    if (!started) {
        started = true;
        if (!extflash::begin()) {
            nextProbeRetry = Time::getMillis() + PROBE_RETRY_MS;
            return false;
        }
        initPartitions();
    }
    if (!extflash::ready()) {
        if (!extflash::retry()) {
            nextProbeRetry = Time::getMillis() + PROBE_RETRY_MS;
            return false;
        }
        initPartitions();
    }
    return true;
}

void checkPartition(Partition &p)
{
    if (lfs_mount(&p.lfs, &p.cfg) == 0) {
        p.mounted = true;
        p.state = ST_READY;
        return;
    }

    uint8_t head[16] = {};
    if (!extflash::read(p.base, head, sizeof(head)))
        return; // stay UNCHECKED and retry

    bool blank = true;
    for (uint8_t b : head) {
        if (b != 0xFF) {
            blank = false;
            break;
        }
    }
    if (blank) {
        p.state = ST_ERASING;
        p.eraseNext = 0;
        p.eraseTotal = p.cfg.block_count;
    } else {
        p.state = ST_CORRUPT;
        LOG_WARN("ExtStorage: %s mount failed and is not blank", p.name);
    }
}

void eraseStep(Partition &p)
{
    for (uint8_t i = 0; i < ERASE_BLOCKS_PER_STEP && p.eraseNext < p.eraseTotal; i++, p.eraseNext++) {
        if (!extflash::erase4k(p.base + p.eraseNext * BLOCK_SIZE)) {
            p.state = ST_CORRUPT;
            LOG_WARN("ExtStorage: %s erase failed at block %u", p.name, (unsigned)p.eraseNext);
            return;
        }
    }
    if (p.eraseNext >= p.eraseTotal)
        p.state = ST_FORMATTING;
}

void doFormat(Partition &p)
{
    p.eraseSuppressed = true;
    int err = lfs_format(&p.lfs, &p.cfg);
    p.eraseSuppressed = false;

    if (err == 0 && lfs_mount(&p.lfs, &p.cfg) == 0) {
        p.mounted = true;
        p.state = ST_READY;
        LOG_INFO("ExtStorage: %s formatted (%luKB)", p.name, (unsigned long)(p.bytes / 1024));
    } else {
        p.mounted = false;
        p.state = ST_CORRUPT;
        LOG_WARN("ExtStorage: %s format failed (%d)", p.name, err);
    }
}

bool step()
{
    bool busy = false;
    for (auto &p : parts) {
        switch (p.state) {
        case ST_UNCHECKED:
            checkPartition(p);
            busy = true;
            break;
        case ST_ERASING:
            eraseStep(p);
            busy = true;
            break;
        case ST_FORMATTING:
            doFormat(p);
            busy = true;
            break;
        default:
            break;
        }
    }
    return busy;
}

uint32_t usedBytes(Partition &p, const char *path)
{
    lfs_dir_t dir;
    if (lfs_dir_open(&p.lfs, &dir, path) != 0)
        return 0;

    uint32_t total = 0;
    lfs_info info = {};
    while (lfs_dir_read(&p.lfs, &dir, &info) > 0) {
        if (info.type == LFS_TYPE_REG) {
            total += info.size;
        } else if (info.type == LFS_TYPE_DIR && strcmp(info.name, ".") != 0 && strcmp(info.name, "..") != 0) {
            char child[96];
            if (snprintf(child, sizeof(child), "%s/%s", path, info.name) < (int)sizeof(child))
                total += usedBytes(p, child);
        }
    }
    lfs_dir_close(&p.lfs, &dir);
    return total;
}

void appendPartitionStatus(char *out, size_t cap, size_t &pos, Partition &p)
{
    switch (p.state) {
    case ST_READY: {
        const uint32_t totalKB = p.bytes / 1024;
        const uint32_t usedKB = usedBytes(p, "/") / 1024;
        const uint32_t freeKB = totalKB > usedKB ? totalKB - usedKB : 0;
        appendf(out, cap, pos, "%s: U:%lu F:%lu T:%luKB\n", p.name, (unsigned long)usedKB, (unsigned long)freeKB,
                (unsigned long)totalKB);
        break;
    }
    case ST_ERASING: {
        const uint32_t pct = p.eraseTotal ? (p.eraseNext * 100u) / p.eraseTotal : 0;
        appendf(out, cap, pos, "%s: formatting %lu%%\n", p.name, (unsigned long)pct);
        break;
    }
    case ST_FORMATTING:
        appendf(out, cap, pos, "%s: formatting 100%%\n", p.name);
        break;
    case ST_UNCHECKED:
        appendf(out, cap, pos, "%s: probing\n", p.name);
        break;
    case ST_CORRUPT:
        appendf(out, cap, pos, "%s: corrupt (EXT FORMAT)\n", p.name);
        break;
    case ST_DISABLED:
        appendf(out, cap, pos, "%s: disabled\n", p.name);
        break;
    }
}

int partitionIndex(const char *which)
{
    if (!which || !*which || strcasecmp(which, "ext") == 0)
        return 0;
    if (strcasecmp(which, "secondary") == 0 || strcasecmp(which, "sec") == 0)
        return 1;
    return -1;
}

bool idOk()
{
#if defined(MESH_TRACKER_X1)
    return extflash::jedec() == 0xC84017; // GD25Q64E
#elif defined(SEEED_WIO_TRACKER_L1) || defined(SEEED_XIAO_NRF52840_KIT)
    return extflash::jedec() == 0x856015; // P25Q16H
#else
    return true;
#endif
}
} // namespace

bool loop()
{
    concurrency::LockGuard guard(&fsLock());

    if (!started) {
        started = true;
        if (!extflash::begin()) {
            nextProbeRetry = Time::getMillis() + PROBE_RETRY_MS;
            return false;
        }
        initPartitions();
    }

    if (!extflash::ready()) {
        const uint32_t now = Time::getMillis();
        if (nextProbeRetry != 0 && Throttle::deadlinePassedAt(now, nextProbeRetry)) {
            nextProbeRetry = 0;
            if (!extflash::retry()) {
                nextProbeRetry = now + PROBE_RETRY_MS;
                return false;
            }
            initPartitions();
        } else {
            return false;
        }
    }

    return step();
}

void status(char *out, size_t cap)
{
    concurrency::LockGuard guard(&fsLock());
    if (!ensureProbe()) {
        snprintf(out, cap, "JT EXT: no QSPI chip (probe failed)");
        return;
    }

    uint8_t probe[8] = {};
    const bool readOk = extflash::read(0, probe, sizeof(probe));

    size_t pos = 0;
    appendf(out, cap, pos, "JT EXT: JEDEC:%06lX %luKB id:%s rd:%s\n", (unsigned long)extflash::jedec(),
            (unsigned long)(extflash::capacity() / 1024), idOk() ? "ok" : "MISMATCH", readOk ? "ok" : "FAIL");
    for (auto &p : parts)
        appendPartitionStatus(out, cap, pos, p);
    if (pos && pos < cap && out[pos - 1] == '\n')
        out[pos - 1] = '\0';
}

void format(const char *which, char *out, size_t cap)
{
    concurrency::LockGuard guard(&fsLock());
    if (!ensureProbe()) {
        snprintf(out, cap, "JT EXT: no QSPI chip (probe failed)");
        return;
    }

    const bool all = !which || !*which || strcasecmp(which, "all") == 0 || strcasecmp(which, "both") == 0;
    const int idx = all ? -1 : partitionIndex(which);
    if (!all && idx < 0) {
        snprintf(out, cap, "JT EXT: FORMAT ext|secondary|all");
        return;
    }

    for (uint8_t i = 0; i < 2; i++) {
        if (!all && idx != (int)i)
            continue;
        Partition &p = parts[i];
        if (p.state == ST_DISABLED)
            continue;
        if (p.mounted) {
            (void)lfs_unmount(&p.lfs);
            p.mounted = false;
        }
        p.state = ST_ERASING;
        p.eraseNext = 0;
        p.eraseTotal = p.cfg.block_count;
        p.eraseSuppressed = false;
    }

    snprintf(out, cap, "JT EXT: formatting %s (EXT shows progress)", all ? "ext+secondary" : parts[idx].name);
}

void selfTest(const char *which, char *out, size_t cap)
{
    concurrency::LockGuard guard(&fsLock());
    if (!ensureProbe()) {
        snprintf(out, cap, "JT EXT: no QSPI chip (probe failed)");
        return;
    }

    const int idx = partitionIndex(which);
    if (idx < 0) {
        snprintf(out, cap, "JT EXT: TEST [ext|secondary]");
        return;
    }

    Partition &p = parts[idx];
    if (p.state != ST_READY || !p.mounted) {
        snprintf(out, cap, "JT EXT: %s not ready (%s)", p.name,
                 p.state == ST_ERASING || p.state == ST_FORMATTING ? "formatting"
                 : p.state == ST_CORRUPT                          ? "corrupt"
                                                                  : "probing");
        return;
    }

    static const char payload[] = "nexus-ext-ok-0123456789";
    const size_t len = sizeof(payload) - 1;
    const char *path = "/selftest";
    lfs_file_t file = {};
    uint8_t readback[sizeof(payload)] = {};

    int err = lfs_file_open(&p.lfs, &file, path, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (err) {
        snprintf(out, cap, "JT EXT: %s test open failed (%d)", p.name, err);
        return;
    }
    lfs_ssize_t wrote = lfs_file_write(&p.lfs, &file, payload, len);
    err = lfs_file_close(&p.lfs, &file);
    if (err || wrote != (lfs_ssize_t)len) {
        (void)lfs_remove(&p.lfs, path);
        snprintf(out, cap, "JT EXT: %s test write failed (%d)", p.name, err ? err : (int)wrote);
        return;
    }

    err = lfs_file_open(&p.lfs, &file, path, LFS_O_RDONLY);
    if (err) {
        (void)lfs_remove(&p.lfs, path);
        snprintf(out, cap, "JT EXT: %s test reopen failed (%d)", p.name, err);
        return;
    }
    lfs_ssize_t got = lfs_file_read(&p.lfs, &file, readback, len);
    (void)lfs_file_close(&p.lfs, &file);
    (void)lfs_remove(&p.lfs, path);
    if (got != (lfs_ssize_t)len || memcmp(readback, payload, len) != 0) {
        snprintf(out, cap, "JT EXT: %s test readback failed", p.name);
        return;
    }
    snprintf(out, cap, "JT EXT: %s test ok (%uB)", p.name, (unsigned)len);
}
} // namespace nexus::extstorage

#endif // NEXUS_ENABLE_EXTERNAL_FLASH && ARCH_NRF52
