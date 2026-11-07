// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#include "JenWANEngine.h"

#if NEXUS_ENABLE_JENWAN

#include "Channels.h"
#include "JenWANPayload.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RadioLibInterface.h"
#include "Router.h"
#include "Throttle.h"
#include "UptimeClock.h"
#include "concurrency/LockGuard.h"
#include "freertosinc.h"
#include <RadioLib.h>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

namespace jenwan
{
namespace
{
static_assert(RADIOLIB_LORAWAN_NONCES_BUF_SIZE <= NONCES_MAX, "JenWAN nonces buffer too small");
static_assert(RADIOLIB_LORAWAN_SESSION_BUF_SIZE <= SESSION_MAX, "JenWAN session buffer too small");

uint64_t euiToU64(const uint8_t eui[EUI_LEN])
{
    uint64_t v = 0;
    for (size_t i = 0; i < EUI_LEN; i++)
        v = (v << 8) | eui[i];
    return v;
}

uint32_t addrToU32(const uint8_t addr[4])
{
    return ((uint32_t)addr[0] << 24) | ((uint32_t)addr[1] << 16) | ((uint32_t)addr[2] << 8) | addr[3];
}

const LoRaWANBand_t *bandFor(Band band)
{
    switch (band) {
    case Band::EU868:
        return &EU868;
    case Band::US915:
        return &US915;
    case Band::EU433:
        return &EU433;
    case Band::AU915:
        return &AU915;
    case Band::CN470:
        return &CN470;
    case Band::AS923:
        return &AS923;
    case Band::KR920:
        return &KR920;
    case Band::IN865:
        return &IN865;
    default:
        return nullptr;
    }
}

int b64Value(char c)
{
    if (c >= 'A' && c <= 'Z')
        return c - 'A';
    if (c >= 'a' && c <= 'z')
        return c - 'a' + 26;
    if (c >= '0' && c <= '9')
        return c - '0' + 52;
    if (c == '+' || c == '-')
        return 62;
    if (c == '/' || c == '_')
        return 63;
    return -1;
}

bool decodeBase64(const char *in, uint8_t *out, size_t cap, size_t &got)
{
    uint32_t buf = 0;
    int bits = 0;
    size_t n = 0;
    while (in && *in) {
        if (*in == '=')
            break;
        int v = b64Value(*in++);
        if (v < 0) {
            if (isspace((unsigned char)in[-1]))
                continue;
            return false;
        }
        buf = (buf << 6) | (uint32_t)v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (n >= cap)
                return false;
            out[n++] = (uint8_t)(buf >> bits);
        }
    }
    got = n;
    return true;
}

bool decodeHex(const char *in, uint8_t *out, size_t cap, size_t &got)
{
    if (in[0] == '0' && (in[1] == 'x' || in[1] == 'X'))
        in += 2;
    int hi = -1;
    size_t n = 0;
    for (; *in; ++in) {
        int v;
        if (*in >= '0' && *in <= '9')
            v = *in - '0';
        else if (*in >= 'a' && *in <= 'f')
            v = *in - 'a' + 10;
        else if (*in >= 'A' && *in <= 'F')
            v = *in - 'A' + 10;
        else
            return false;
        if (hi < 0)
            hi = v;
        else {
            if (n >= cap)
                return false;
            out[n++] = (uint8_t)((hi << 4) | v);
            hi = -1;
        }
    }
    if (hi >= 0)
        return false;
    got = n;
    return true;
}

bool decodeBytes(const char *in, uint8_t *out, size_t cap, size_t &got)
{
    if (!in || !*in)
        return false;
    if (strncasecmp(in, "b64:", 4) == 0 || strncasecmp(in, "base64:", 7) == 0) {
        const char *p = (in[0] == 'b' || in[0] == 'B') ? in + 4 : in + 7;
        return decodeBase64(p, out, cap, got);
    }
    if (strncasecmp(in, "hex:", 4) == 0)
        return decodeHex(in + 4, out, cap, got);
    if (in[0] == '0' && (in[1] == 'x' || in[1] == 'X'))
        return decodeHex(in, out, cap, got);

    // Bare input: hex when it is even-length hex, base64 otherwise.
    bool isHex = true;
    size_t len = strlen(in);
    for (size_t i = 0; i < len; i++) {
        char c = in[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) {
            isHex = false;
            break;
        }
    }
    if (isHex && (len % 2) == 0)
        return decodeHex(in, out, cap, got);
    return decodeBase64(in, out, cap, got);
}

char *nextToken(char *&p)
{
    while (*p && isspace((unsigned char)*p))
        p++;
    if (!*p)
        return nullptr;
    char *start = p;
    while (*p && !isspace((unsigned char)*p))
        p++;
    if (*p)
        *p++ = '\0';
    return start;
}

bool parseOnOff(const char *s, bool &on)
{
    if (!s)
        return false;
    if (strcasecmp(s, "ON") == 0 || strcasecmp(s, "1") == 0) {
        on = true;
        return true;
    }
    if (strcasecmp(s, "OFF") == 0 || strcasecmp(s, "0") == 0) {
        on = false;
        return true;
    }
    return false;
}

int sourceFromName(const char *s)
{
    if (!s)
        return -1;
    if (strcasecmp(s, "DEVICE") == 0 || strcasecmp(s, "DEV") == 0)
        return SRC_DEVICE;
    if (strcasecmp(s, "POWER") == 0 || strcasecmp(s, "PWR") == 0)
        return SRC_POWER;
    if (strcasecmp(s, "ENV") == 0 || strcasecmp(s, "ENVIRONMENT") == 0)
        return SRC_ENV;
    if (strcasecmp(s, "POS") == 0 || strcasecmp(s, "POSITION") == 0)
        return SRC_POS;
    return -1;
}

const char *sourceName(int s)
{
    switch (s) {
    case SRC_DEVICE:
        return "DEV";
    case SRC_POWER:
        return "PWR";
    case SRC_ENV:
        return "ENV";
    case SRC_POS:
        return "POS";
    default:
        return "?";
    }
}

void hexTail(const uint8_t *data, size_t len, char *out, size_t cap)
{
    if (!data || len < 2 || cap < 5) {
        if (cap)
            out[0] = '\0';
        return;
    }
    snprintf(out, cap, "%02X%02X", data[len - 2], data[len - 1]);
}

bool hasKeys(const Config &c)
{
    if (c.join_mode == (uint8_t)JoinMode::OTAA)
        return (c.key_flags & 0x01) != 0;
    if (c.join_mode == (uint8_t)JoinMode::ABP)
        return (c.key_flags & 0x02) != 0;
    return false;
}
} // namespace

Engine engine;

bool Engine::begin()
{
    const char *reason = nullptr;
    if (!jenwan::load(cfg, &reason)) {
        LOG_INFO("JenWAN: no usable /jenwan.bin (%s); defaults", reason ? reason : "unknown");
    } else {
        LOG_INFO("JenWAN: loaded /jenwan.bin (%s), enabled=%u band=%s", reason ? reason : "OK", (unsigned)cfg.enabled,
                 bandName((Band)cfg.band));
    }
    activated = false;

#ifdef HAS_FREE_RTOS
    if (!taskStarted) {
#ifdef ARCH_ESP32
        const BaseType_t rc = xTaskCreate(taskEntry, "JenWAN", 4096, this, tskIDLE_PRIORITY + 1, (TaskHandle_t *)&taskHandle);
#else
        const BaseType_t rc = xTaskCreate(taskEntry, "JenWAN", 1024, this, tskIDLE_PRIORITY + 1, (TaskHandle_t *)&taskHandle);
#endif
        taskStarted = (rc == pdPASS);
        if (!taskStarted)
            LOG_ERROR("JenWAN: window task failed to start");
    }
#endif
    return true;
}

void Engine::taskEntry(void *arg)
{
    static_cast<Engine *>(arg)->taskLoop();
}

void Engine::taskLoop()
{
    for (;;) {
#ifdef HAS_FREE_RTOS
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#endif
        processRequest();
    }
}

bool Engine::requestWindow(WindowKind kind, uint8_t sources, uint8_t port, const uint8_t *data, size_t len)
{
    if (windowActive || windowRequested)
        return false;
    {
        concurrency::LockGuard guard(&cfgLock);
        requestKind = kind;
        requestSources = sources;
        requestPort = port;
        requestLen = (len > sizeof(requestData)) ? sizeof(requestData) : len;
        if (data && requestLen)
            memcpy(requestData, data, requestLen);
        windowActive = true;
        windowRequested = true;
    }
    if (taskStarted && taskHandle) {
#ifdef HAS_FREE_RTOS
        xTaskNotifyGive((TaskHandle_t)taskHandle);
        return true;
#endif
    }
    // Non-FreeRTOS fallback: run the window inline.
    processRequest();
    return true;
}

void Engine::processRequest()
{
    WindowKind kind;
    uint8_t sources;
    uint8_t port;
    uint8_t data[MAX_MANUAL];
    size_t len;
    {
        concurrency::LockGuard guard(&cfgLock);
        kind = requestKind;
        sources = requestSources;
        port = requestPort;
        len = requestLen;
        if (len)
            memcpy(data, requestData, len);
        requestLen = 0;
        requestKind = WindowKind::NONE;
        windowRequested = false;
    }
    if (kind == WindowKind::NONE) {
        windowActive = false;
        return;
    }

    const bool ok = runWindow(kind, sources, port, len ? data : nullptr, len);
    const uint32_t now = Time::getMillis();
    if (ok) {
        backoffMs = 60000;
        nextAttemptMs = 0;
        if (kind == WindowKind::UPLINK && sources) {
            for (uint8_t s = 0; s < NUM_SOURCES; s++) {
                if (sources & sourceBit(s))
                    lastSentMs[s] = now;
            }
        }
    } else {
        if (backoffMs == 0)
            backoffMs = 60000;
        else if (backoffMs < 900000)
            backoffMs *= 2;
        nextAttemptMs = now + backoffMs;
    }
    windowActive = false;
}

bool Engine::ensureNode(const Config &local)
{
    if (needRestart) {
        destroyNode();
        needRestart = false;
    }
    if (node)
        return true;

    RadioLibInterface *rli = RadioLibInterface::instance;
    if (!rli)
        return false;
    PhysicalLayer *phy = rli->getPhysicalLayer();
    if (!phy)
        return false;

    band = bandFor((Band)local.band);
    if (!band)
        return false;

    node = new (std::nothrow) LoRaWANNode(phy, band, local.sub_band);
    if (!node)
        return false;

    int16_t st = RADIOLIB_ERR_UNKNOWN;
    if (local.join_mode == (uint8_t)JoinMode::OTAA) {
        st = node->beginOTAA(euiToU64(local.join_eui), euiToU64(local.dev_eui), local.nwk_key, local.app_key);
    } else if (local.join_mode == (uint8_t)JoinMode::ABP) {
        st = node->beginABP(addrToU32(local.dev_addr), local.nwk_s_key, local.nwk_s_key, local.nwk_s_key, local.app_s_key);
    }
    if (st != RADIOLIB_ERR_NONE) {
        LOG_WARN("JenWAN: begin%d failed (%d)", (int)local.join_mode, (int)st);
        destroyNode();
        return false;
    }

    node->setDutyCycle((local.flags & (1u << CFG_DUTY)) != 0);
    node->setADR((local.flags & (1u << CFG_ADR)) != 0);

    // beginOTAA/beginABP clear both buffers, so restore them afterwards.
    if (local.nonces_len == RADIOLIB_LORAWAN_NONCES_BUF_SIZE)
        (void)node->setBufferNonces(local.nonces);
    if (local.session_len == RADIOLIB_LORAWAN_SESSION_BUF_SIZE)
        (void)node->setBufferSession(local.session);
    activated = node->isActivated();
    return true;
}

void Engine::destroyNode()
{
    if (node) {
        delete node;
        node = nullptr;
    }
    band = nullptr;
    activated = false;
}

bool Engine::doJoin(const Config &local)
{
    if (!node)
        return false;
    if (node->isActivated()) {
        activated = true;
        return true;
    }
    int16_t st;
    if (local.join_mode == (uint8_t)JoinMode::OTAA)
        st = node->activateOTAA();
    else if (local.join_mode == (uint8_t)JoinMode::ABP)
        st = node->activateABP();
    else
        return false;

    lastState = st;
    activated = node->isActivated();
    if (activated || st == RADIOLIB_LORAWAN_NEW_SESSION || st == RADIOLIB_LORAWAN_SESSION_RESTORED) {
        updateSession();
        return true;
    }
    LOG_WARN("JenWAN: join failed (%d)", (int)st);
    return false;
}

void Engine::updateSession()
{
    if (!node)
        return;
    const uint8_t *nonces = node->getBufferNonces();
    const uint8_t *session = node->getBufferSession();
    concurrency::LockGuard guard(&cfgLock);
    if (nonces) {
        memcpy(cfg.nonces, nonces, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
        cfg.nonces_len = RADIOLIB_LORAWAN_NONCES_BUF_SIZE;
    }
    if (session) {
        memcpy(cfg.session, session, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
        cfg.session_len = RADIOLIB_LORAWAN_SESSION_BUF_SIZE;
    }
}

bool Engine::runWindow(WindowKind kind, uint8_t sources, uint8_t port, const uint8_t *data, size_t len)
{
    RadioLibInterface *rli = RadioLibInterface::instance;
    if (!rli || !rli->beginExternalRadioUse()) {
        lastState = RADIOLIB_ERR_UNKNOWN;
        return false;
    }

    Config local;
    {
        concurrency::LockGuard guard(&cfgLock);
        local = cfg;
    }

    bool ok = false;
    if (ensureNode(local)) {
        if (!node->isActivated()) {
            if (!doJoin(local)) {
                rli->endExternalRadioUse();
                return false;
            }
        }
        if (node->isActivated()) {
            uint8_t payload[LPP_MAX_PAYLOAD];
            size_t plen = 0;
            if (kind == WindowKind::UPLINK) {
                if (data && len) {
                    plen = (len > sizeof(payload)) ? sizeof(payload) : len;
                    memcpy(payload, data, plen);
                } else {
                    plen = encodeSources(sources, payload, sizeof(payload));
                }
            }
            if (plen) {
                uint8_t down[64];
                size_t downLen = sizeof(down);
                const bool confirmed = (local.flags & (1u << CFG_CONFIRMED)) != 0;
                int16_t st = node->sendReceive(payload, plen, port, down, &downLen, confirmed, nullptr, nullptr);
                lastState = st;
                lastPort = port;
                lastLen = (uint8_t)plen;
                if (st >= 0) {
                    ok = true;
                    lastUplinkMs = Time::getMillis();
                    uplinkCount++;
                    if (st > 0 && downLen)
                        handleDownlink(down, downLen);
                    updateSession();
                } else {
                    LOG_WARN("JenWAN: uplink failed (%d)", (int)st);
                }
            } else if (kind == WindowKind::JOIN) {
                ok = true;
            }
        }
    }

    rli->endExternalRadioUse();
    if (ok)
        save();
    return ok;
}

uint8_t Engine::dueSources(uint32_t nowMs) const
{
    uint8_t due = 0;
    for (uint8_t s = 0; s < NUM_SOURCES; s++) {
        if (!(cfg.sources & sourceBit(s)))
            continue;
        if (s == SRC_POS && !(cfg.flags & (1u << CFG_POS)))
            continue;
        const uint32_t interval = cfg.intervals[s];
        if (interval < 60)
            continue;
        if (lastSentMs[s] == 0 || Throttle::hasElapsed(lastSentMs[s], interval * 1000))
            due |= sourceBit(s);
    }
    return due;
}

uint32_t Engine::nextDueMs(uint32_t nowMs) const
{
    uint32_t best = 0;
    for (uint8_t s = 0; s < NUM_SOURCES; s++) {
        if (!(cfg.sources & sourceBit(s)))
            continue;
        if (s == SRC_POS && !(cfg.flags & (1u << CFG_POS)))
            continue;
        const uint32_t interval = cfg.intervals[s];
        if (interval < 60)
            continue;
        const uint32_t due = lastSentMs[s] ? lastSentMs[s] + interval * 1000 : nowMs;
        if (best == 0 || (int32_t)(due - best) < 0)
            best = due;
    }
    if (best == 0)
        return 0;
    const int32_t delta = (int32_t)(best - nowMs);
    return delta > 0 ? (uint32_t)delta : 0;
}

void Engine::handleDownlink(const uint8_t *data, size_t len)
{
    if (!data || !len)
        return;
    uint8_t mode;
    {
        concurrency::LockGuard guard(&cfgLock);
        mode = cfg.downlink;
    }
    if (mode == (uint8_t)DownlinkMode::OFF)
        return;

    const size_t n = len > MAX_DOWNLINK ? MAX_DOWNLINK : len;
    {
        concurrency::LockGuard guard(&cfgLock);
        DownlinkEntry &e = downlinks[downlinkHead];
        e.len = (uint8_t)n;
        memcpy(e.data, data, n);
        downlinkHead = (uint8_t)((downlinkHead + 1) % DOWNLINK_SLOTS);
        if (downlinkCount < DOWNLINK_SLOTS)
            downlinkCount = (uint8_t)(downlinkCount + 1);
    }

    if (mode != (uint8_t)DownlinkMode::TEXT)
        return;

    bool printable = true;
    for (size_t i = 0; i < n; i++) {
        const uint8_t c = data[i];
        if (c != '\r' && c != '\n' && (c < 0x20 || c > 0x7E)) {
            printable = false;
            break;
        }
    }
    if (!printable)
        return;

    concurrency::LockGuard guard(&cfgLock);
    const size_t m = n > sizeof(downlinkText) - 1 ? sizeof(downlinkText) - 1 : n;
    memcpy(downlinkText, data, m);
    downlinkText[m] = '\0';
    downlinkTextPending = true;
}

bool Engine::sendPendingDownlinkText()
{
    char text[sizeof(downlinkText)];
    uint8_t chan;
    {
        concurrency::LockGuard guard(&cfgLock);
        if (!downlinkTextPending)
            return false;
        memcpy(text, downlinkText, sizeof(text));
        downlinkTextPending = false;
        chan = cfg.downlink_chan;
        if (chan >= MAX_NUM_CHANNELS)
            chan = (uint8_t)channels.getPrimaryIndex();
    }

    meshtastic_MeshPacket *p = router ? router->allocForSending() : nullptr;
    if (!p)
        return false;
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->to = NODENUM_BROADCAST;
    p->channel = chan;
    const size_t len = strlen(text);
    const size_t maxLen = sizeof(p->decoded.payload.bytes);
    const size_t copy = len > maxLen ? maxLen : len;
    memcpy(p->decoded.payload.bytes, text, copy);
    p->decoded.payload.size = copy;
    if (service)
        service->sendToMesh(p);
    return true;
}

void Engine::loop()
{
    if (!cfg.enabled)
        return;
    if (windowActive || windowRequested)
        return;

    if (downlinkTextPending)
        sendPendingDownlinkText();

    const uint32_t now = Time::getMillis();
    if (nextAttemptMs && !Throttle::deadlinePassedAt(now, nextAttemptMs))
        return;

    if (!hasKeys(cfg))
        return;

    if (!activated) {
        requestWindow(WindowKind::JOIN);
        return;
    }

    const uint8_t due = dueSources(now);
    if (due)
        requestWindow(WindowKind::UPLINK, due, cfg.port);
}

bool Engine::save()
{
    Config copy;
    {
        concurrency::LockGuard guard(&cfgLock);
        copy = cfg;
    }
    return jenwan::save(copy);
}

void Engine::appendStatus(char *reply, size_t cap)
{
    const uint32_t now = Time::getMillis();
    const uint32_t next = nextDueMs(now);
    const uint32_t ago = lastUplinkMs ? (now - lastUplinkMs) / 1000 : 0;
    char devfp[5] = "----";
    if (cfg.join_mode == (uint8_t)JoinMode::OTAA)
        hexTail(cfg.dev_eui, EUI_LEN, devfp, sizeof(devfp));
    else if (cfg.join_mode == (uint8_t)JoinMode::ABP)
        hexTail(cfg.dev_addr, 4, devfp, sizeof(devfp));

    const char *mode = cfg.join_mode == (uint8_t)JoinMode::OTAA  ? "OTAA"
                       : cfg.join_mode == (uint8_t)JoinMode::ABP ? "ABP"
                                                                 : "none";
    const char *dl = cfg.downlink == (uint8_t)DownlinkMode::STORE  ? "STORE"
                     : cfg.downlink == (uint8_t)DownlinkMode::TEXT ? "TEXT"
                                                                   : "OFF";

    snprintf(reply, cap,
             "JT JW:%s %s SUB%u %s %s dev=%s\n"
             "DEV/%lus POS/%lus ENV/%lus PWR/%lus src=%X\n"
             "last=%s p%u %uB %lus ago next=%lus\n"
             "dl=%s ch%u duty=%u adr=%u conf=%u ups=%lu",
             cfg.enabled ? "ON" : "OFF", bandName((Band)cfg.band), (unsigned)cfg.sub_band, mode, activated ? "joined" : "idle",
             devfp, (unsigned long)cfg.intervals[SRC_DEVICE], (unsigned long)cfg.intervals[SRC_POS],
             (unsigned long)cfg.intervals[SRC_ENV], (unsigned long)cfg.intervals[SRC_POWER], (unsigned)cfg.sources,
             lastState >= 0 ? "OK" : "ERR", (unsigned)lastPort, (unsigned)lastLen, (unsigned long)ago,
             (unsigned long)(next / 1000), dl, (unsigned)cfg.downlink_chan, (cfg.flags & (1u << CFG_DUTY)) ? 1u : 0u,
             (cfg.flags & (1u << CFG_ADR)) ? 1u : 0u, (cfg.flags & (1u << CFG_CONFIRMED)) ? 1u : 0u, (unsigned long)uplinkCount);
}

bool Engine::handleCommand(const char *args, char *reply, size_t cap)
{
    if (!reply || cap == 0)
        return false;
    reply[0] = '\0';

    char buf[192];
    if (!args)
        args = "";
    strncpy(buf, args, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *p = buf;
    char *sub = nextToken(p);
    if (!sub || !*sub) {
        appendStatus(reply, cap);
        return true;
    }

    if (strcasecmp(sub, "ON") == 0) {
        cfg.enabled = 1;
        save();
        snprintf(reply, cap, "JT JW: ON. Configure sources with JW SOURCE/INTERVAL.");
        return true;
    }
    if (strcasecmp(sub, "OFF") == 0) {
        cfg.enabled = 0;
        save();
        snprintf(reply, cap, "JT JW: OFF.");
        return true;
    }
    if (strcasecmp(sub, "STATUS") == 0) {
        appendStatus(reply, cap);
        return true;
    }
    if (strcasecmp(sub, "HELP") == 0) {
        snprintf(reply, cap,
                 "JT JW: ON/OFF|STATUS|SAVE|CLEAR|RESTART\n"
                 "REGION <band>[SUB n]|OTAA <j> <d> <a>[nwk]|ABP <a> <n> <k>\n"
                 "SOURCE|INTERVAL|PORT|UPLINK|TELEMETRY|DOWNLINK|CONFIRMED|DUTY|ADR");
        return true;
    }
    if (strcasecmp(sub, "SAVE") == 0) {
        snprintf(reply, cap, "JT JW: %s.", save() ? "saved" : "save failed");
        return true;
    }
    if (strcasecmp(sub, "RESTART") == 0) {
        needRestart = true;
        activated = false;
        snprintf(reply, cap, "JT JW: restarting on next window.");
        return true;
    }
    if (strcasecmp(sub, "CLEAR") == 0) {
        Config fresh;
        defaults(fresh);
        {
            concurrency::LockGuard guard(&cfgLock);
            cfg = fresh;
        }
        needRestart = true;
        activated = false;
        save();
        snprintf(reply, cap, "JT JW: cleared (keys and session wiped).");
        return true;
    }
    if (strcasecmp(sub, "REGION") == 0) {
        char *name = nextToken(p);
        Band b;
        if (!name || !bandFromName(name, b)) {
            snprintf(reply, cap, "JT JW: region? EU868 US915 EU433 AU915 CN470 AS923 KR920 IN865");
            return true;
        }
        int subBand = 0;
        char *next = nextToken(p);
        if (next && strcasecmp(next, "SUB") == 0) {
            char *v = nextToken(p);
            subBand = v ? atoi(v) : 0;
        }
        if ((b == Band::US915 || b == Band::AU915) && (subBand < 1 || subBand > 8)) {
            snprintf(reply, cap, "JT JW: %s needs SUB 1..8", bandName(b));
            return true;
        }
        if (b != Band::US915 && b != Band::AU915)
            subBand = 0;
        cfg.band = (uint8_t)b;
        cfg.sub_band = (uint8_t)subBand;
        needRestart = true;
        activated = false;
        save();
        snprintf(reply, cap, "JT JW: region %s%s%u.", bandName(b), subBand ? " SUB " : "", (unsigned)subBand);
        return true;
    }
    if (strcasecmp(sub, "OTAA") == 0) {
        char *join = nextToken(p);
        char *dev = nextToken(p);
        char *app = nextToken(p);
        char *nwk = nextToken(p);
        uint8_t je[EUI_LEN], de[EUI_LEN], ak[KEY_LEN], nk[KEY_LEN];
        size_t jn = 0, dn = 0, an = 0, nn = 0;
        if (!join || !dev || !app || !decodeBytes(join, je, sizeof(je), jn) || jn != EUI_LEN ||
            !decodeBytes(dev, de, sizeof(de), dn) || dn != EUI_LEN || !decodeBytes(app, ak, sizeof(ak), an) || an != KEY_LEN) {
            snprintf(reply, cap, "JT JW: OTAA <joinEui8B> <devEui8B> <appKey16B> [nwkKey16B]");
            return true;
        }
        if (nwk) {
            if (!decodeBytes(nwk, nk, sizeof(nk), nn) || nn != KEY_LEN) {
                snprintf(reply, cap, "JT JW: bad nwkKey (needs 16 bytes)");
                return true;
            }
        } else {
            memcpy(nk, ak, sizeof(nk));
        }
        {
            concurrency::LockGuard guard(&cfgLock);
            memcpy(cfg.join_eui, je, sizeof(je));
            memcpy(cfg.dev_eui, de, sizeof(de));
            memcpy(cfg.app_key, ak, sizeof(ak));
            memcpy(cfg.nwk_key, nk, sizeof(nk));
            cfg.join_mode = (uint8_t)JoinMode::OTAA;
            cfg.key_flags |= 0x01;
            cfg.nonces_len = 0;
            cfg.session_len = 0;
        }
        needRestart = true;
        activated = false;
        save();
        char fp[5];
        hexTail(de, sizeof(de), fp, sizeof(fp));
        snprintf(reply, cap, "JT JW: OTAA keys set (dev=%s). JW JOIN to join.", fp);
        return true;
    }
    if (strcasecmp(sub, "ABP") == 0) {
        char *addr = nextToken(p);
        char *nwk = nextToken(p);
        char *app = nextToken(p);
        uint8_t a[4], nk[KEY_LEN], ak[KEY_LEN];
        size_t an = 0, nn = 0, akn = 0;
        if (!addr || !nwk || !app || !decodeBytes(addr, a, sizeof(a), an) || an != 4 || !decodeBytes(nwk, nk, sizeof(nk), nn) ||
            nn != KEY_LEN || !decodeBytes(app, ak, sizeof(ak), akn) || akn != KEY_LEN) {
            snprintf(reply, cap, "JT JW: ABP <devAddr4B> <nwkSKey16B> <appSKey16B>");
            return true;
        }
        {
            concurrency::LockGuard guard(&cfgLock);
            memcpy(cfg.dev_addr, a, sizeof(a));
            memcpy(cfg.nwk_s_key, nk, sizeof(nk));
            memcpy(cfg.app_s_key, ak, sizeof(ak));
            cfg.join_mode = (uint8_t)JoinMode::ABP;
            cfg.key_flags |= 0x02;
            cfg.nonces_len = 0;
            cfg.session_len = 0;
        }
        needRestart = true;
        activated = false;
        save();
        snprintf(reply, cap, "JT JW: ABP keys set.");
        return true;
    }
    if (strcasecmp(sub, "JOIN") == 0) {
        char *what = nextToken(p);
        if (what && strcasecmp(what, "CLEAR") == 0) {
            {
                concurrency::LockGuard guard(&cfgLock);
                cfg.nonces_len = 0;
                cfg.session_len = 0;
                memset(cfg.nonces, 0, sizeof(cfg.nonces));
                memset(cfg.session, 0, sizeof(cfg.session));
            }
            needRestart = true;
            activated = false;
            save();
            snprintf(reply, cap, "JT JW: session cleared.");
            return true;
        }
        if (!hasKeys(cfg)) {
            snprintf(reply, cap, "JT JW: no keys; use JW OTAA or JW ABP first.");
            return true;
        }
        snprintf(reply, cap, requestWindow(WindowKind::JOIN) ? "JT JW: join queued." : "JT JW: window busy.");
        return true;
    }
    if (strcasecmp(sub, "SOURCE") == 0) {
        char *name = nextToken(p);
        char *state = nextToken(p);
        const int src = sourceFromName(name);
        bool on = false;
        if (src < 0 || !parseOnOff(state, on)) {
            snprintf(reply, cap, "JT JW: SOURCE DEV|PWR|ENV|POS ON|OFF");
            return true;
        }
        if (on)
            cfg.sources |= sourceBit((uint8_t)src);
        else
            cfg.sources &= (uint8_t)~sourceBit((uint8_t)src);
        if (src == SRC_POS) {
            if (on)
                cfg.flags |= (uint8_t)(1u << CFG_POS);
            else
                cfg.flags &= (uint8_t)~(1u << CFG_POS);
        }
        save();
        snprintf(reply, cap, "JT JW: %s %s (interval %lus).", sourceName(src), on ? "ON" : "OFF",
                 (unsigned long)cfg.intervals[src]);
        return true;
    }
    if (strcasecmp(sub, "INTERVAL") == 0) {
        char *name = nextToken(p);
        char *val = nextToken(p);
        const int src = sourceFromName(name);
        const long sec = val ? atol(val) : -1;
        if (src < 0 || sec < 0 || (sec != 0 && sec < 60) || sec > 604800) {
            snprintf(reply, cap, "JT JW: INTERVAL <DEV|PWR|ENV|POS> <60..604800|0>");
            return true;
        }
        cfg.intervals[src] = (uint32_t)sec;
        save();
        snprintf(reply, cap, "JT JW: %s interval %lus.", sourceName(src), (unsigned long)sec);
        return true;
    }
    if (strcasecmp(sub, "PORT") == 0) {
        char *val = nextToken(p);
        const int v = val ? atoi(val) : 0;
        if (v < 1 || v > 223) {
            snprintf(reply, cap, "JT JW: PORT 1..223");
            return true;
        }
        cfg.port = (uint8_t)v;
        save();
        snprintf(reply, cap, "JT JW: port %d.", v);
        return true;
    }
    if (strcasecmp(sub, "FORMAT") == 0) {
        char *val = nextToken(p);
        if (!val || strcasecmp(val, "CAYENNE") != 0) {
            snprintf(reply, cap, "JT JW: FORMAT CAYENNE (COMPACT reserved)");
            return true;
        }
        cfg.format = (uint8_t)PayloadFormat::CAYENNE;
        save();
        snprintf(reply, cap, "JT JW: format CayenneLPP.");
        return true;
    }
    if (strcasecmp(sub, "CONFIRMED") == 0 || strcasecmp(sub, "DUTY") == 0 || strcasecmp(sub, "ADR") == 0) {
        char *val = nextToken(p);
        bool on = false;
        if (!parseOnOff(val, on)) {
            snprintf(reply, cap, "JT JW: %s ON|OFF", sub);
            return true;
        }
        const uint8_t bit = strcasecmp(sub, "CONFIRMED") == 0 ? CFG_CONFIRMED : strcasecmp(sub, "DUTY") == 0 ? CFG_DUTY : CFG_ADR;
        if (on)
            cfg.flags |= (uint8_t)(1u << bit);
        else
            cfg.flags &= (uint8_t)~(1u << bit);
        needRestart = true;
        save();
        snprintf(reply, cap, "JT JW: %s %s.", sub, on ? "ON" : "OFF");
        return true;
    }
    if (strcasecmp(sub, "DOWNLINK") == 0) {
        char *val = nextToken(p);
        if (!val) {
            concurrency::LockGuard guard(&cfgLock);
            if (downlinkCount == 0) {
                snprintf(reply, cap, "JT JW: no stored downlinks.");
                return true;
            }
            size_t pos = 0;
            const uint8_t first = (uint8_t)((downlinkHead + DOWNLINK_SLOTS - downlinkCount) % DOWNLINK_SLOTS);
            for (uint8_t i = 0; i < downlinkCount && pos < cap; i++) {
                const DownlinkEntry &e = downlinks[(first + i) % DOWNLINK_SLOTS];
                pos += snprintf(reply + pos, cap - pos, "%s%u:", i ? " " : "", (unsigned)e.len);
                for (uint8_t j = 0; j < e.len && pos + 3 < cap; j++)
                    pos += snprintf(reply + pos, cap - pos, "%02X", e.data[j]);
            }
            return true;
        }
        DownlinkMode mode;
        if (strcasecmp(val, "OFF") == 0)
            mode = DownlinkMode::OFF;
        else if (strcasecmp(val, "STORE") == 0)
            mode = DownlinkMode::STORE;
        else if (strcasecmp(val, "TEXT") == 0)
            mode = DownlinkMode::TEXT;
        else {
            snprintf(reply, cap, "JT JW: DOWNLINK OFF|STORE|TEXT [CHAN n]");
            return true;
        }
        int ch = cfg.downlink_chan;
        char *chanTok = nextToken(p);
        if (chanTok && strcasecmp(chanTok, "CHAN") == 0) {
            char *v = nextToken(p);
            ch = v ? atoi(v) : -1;
            if (ch < 0 || ch >= MAX_NUM_CHANNELS) {
                snprintf(reply, cap, "JT JW: bad channel.");
                return true;
            }
        }
        cfg.downlink = (uint8_t)mode;
        cfg.downlink_chan = (uint8_t)ch;
        save();
        snprintf(reply, cap, "JT JW: downlink %s ch%d.", val, ch);
        return true;
    }
    if (strcasecmp(sub, "UPLINK") == 0) {
        if (!cfg.enabled) {
            snprintf(reply, cap, "JT JW: OFF (JW ON first).");
            return true;
        }
        char *portTok = nextToken(p);
        char *rest = p;
        while (*rest && isspace((unsigned char)*rest))
            rest++;
        const int port = portTok ? atoi(portTok) : 0;
        if (port < 1 || port > 223 || !*rest) {
            snprintf(reply, cap, "JT JW: UPLINK <port> <hex|b64:..|txt:..>");
            return true;
        }
        uint8_t data[MAX_MANUAL];
        size_t len = 0;
        if (strncasecmp(rest, "txt:", 4) == 0) {
            len = strlen(rest + 4);
            if (len > sizeof(data))
                len = sizeof(data);
            memcpy(data, rest + 4, len);
        } else if (!decodeBytes(rest, data, sizeof(data), len) || len == 0) {
            snprintf(reply, cap, "JT JW: bad payload (use hex, b64: or txt:).");
            return true;
        }
        snprintf(reply, cap,
                 requestWindow(WindowKind::UPLINK, 0, (uint8_t)port, data, len) ? "JT JW: uplink queued."
                                                                                : "JT JW: window busy.");
        return true;
    }
    if (strcasecmp(sub, "TELEMETRY") == 0) {
        if (!cfg.enabled) {
            snprintf(reply, cap, "JT JW: OFF (JW ON first).");
            return true;
        }
        uint8_t src = cfg.sources;
        if (!(cfg.flags & (1u << CFG_POS)))
            src &= (uint8_t)~sourceBit(SRC_POS);
        if (!src) {
            snprintf(reply, cap, "JT JW: no sources enabled.");
            return true;
        }
        snprintf(reply, cap,
                 requestWindow(WindowKind::UPLINK, src, cfg.port) ? "JT JW: telemetry queued." : "JT JW: window busy.");
        return true;
    }

    snprintf(reply, cap, "JT JW: unknown '%s' (JW HELP).", sub);
    return true;
}
} // namespace jenwan

#endif // NEXUS_ENABLE_JENWAN
