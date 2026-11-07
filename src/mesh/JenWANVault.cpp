// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#include "JenWANVault.h"

#if NEXUS_ENABLE_JENWAN

#include "FSCommon.h"
#include "SPILock.h"
#include "SafeFile.h"
#include <ErriezCRC32.h>
#include <cstring>

namespace jenwan
{
namespace
{
constexpr const char *VAULT_PATH = "/jenwan.bin";

bool validateHeader(const char *magic, uint32_t version, size_t fileSize, const char **reason)
{
    auto reject = [reason](const char *why) {
        if (reason)
            *reason = why;
        return false;
    };
    if (memcmp(magic, MAGIC, sizeof(MAGIC)) != 0)
        return reject("bad magic");
    if (version != VERSION)
        return reject("unsupported version");
    if (fileSize != sizeof(Vault))
        return reject("size mismatch");
    if (reason)
        *reason = "OK";
    return true;
}
} // namespace

void defaults(Config &out)
{
    memset(&out, 0, sizeof(out));
    out.enabled = 0;
    out.band = (uint8_t)Band::UNSET;
    out.sub_band = 0;
    out.join_mode = (uint8_t)JoinMode::NONE;
    out.flags = (uint8_t)(1u << CFG_DUTY) | (uint8_t)(1u << CFG_ADR);
    out.sources = 0;
    out.format = (uint8_t)PayloadFormat::CAYENNE;
    out.downlink = (uint8_t)DownlinkMode::OFF;
    out.downlink_chan = 0;
    out.port = 1;
}

bool load(Config &out, const char **reason)
{
    defaults(out);
    if (reason)
        *reason = "no file";
#ifdef FSCom
    concurrency::LockGuard guard(spiLock);
    auto file = FSCom.open(VAULT_PATH, FILE_O_READ);
    if (!file)
        return false;

    Vault v;
    memset(&v, 0, sizeof(v));
    const size_t fileSize = file.size();
    if (fileSize != sizeof(Vault)) {
        file.close();
        if (reason)
            *reason = "invalid size";
        return false;
    }
    const size_t read = file.read(reinterpret_cast<uint8_t *>(&v), fileSize);
    file.close();
    if (read != fileSize) {
        if (reason)
            *reason = "short read";
        return false;
    }
    if (!validateHeader(v.magic, v.version, fileSize, reason))
        return false;
    if (crc32Buffer(&v, offsetof(Vault, crc32)) != v.crc32) {
        if (reason)
            *reason = "CRC mismatch";
        return false;
    }
    out = v.config;
    if (out.nonces_len > NONCES_MAX)
        out.nonces_len = 0;
    if (out.session_len > SESSION_MAX)
        out.session_len = 0;
    if (out.sub_band > 8)
        out.sub_band = 0;
    if (out.port < 1 || out.port > 223)
        out.port = 1;
    if (out.format != (uint8_t)PayloadFormat::CAYENNE)
        out.format = (uint8_t)PayloadFormat::CAYENNE;
    if (reason)
        *reason = "OK";
    return true;
#else
    (void)reason;
    return false;
#endif
}

bool save(const Config &in)
{
#ifdef FSCom
    Vault v;
    memset(&v, 0, sizeof(v));
    memcpy(v.magic, MAGIC, sizeof(MAGIC));
    v.version = VERSION;
    v.config = in;
    if (v.config.nonces_len > NONCES_MAX)
        v.config.nonces_len = 0;
    if (v.config.session_len > SESSION_MAX)
        v.config.session_len = 0;
    v.crc32 = crc32Buffer(&v, offsetof(Vault, crc32));

    concurrency::LockGuard guard(spiLock);
    SafeFile file(VAULT_PATH, true);
    const size_t written = file.write(reinterpret_cast<const uint8_t *>(&v), sizeof(v));
    return written == sizeof(v) && file.close();
#else
    (void)in;
    return false;
#endif
}

const char *bandName(Band band)
{
    switch (band) {
    case Band::EU868:
        return "EU868";
    case Band::US915:
        return "US915";
    case Band::EU433:
        return "EU433";
    case Band::AU915:
        return "AU915";
    case Band::CN470:
        return "CN470";
    case Band::AS923:
        return "AS923";
    case Band::KR920:
        return "KR920";
    case Band::IN865:
        return "IN865";
    case Band::UNSET:
    default:
        return "UNSET";
    }
}

bool bandFromName(const char *name, Band &out)
{
    if (!name || !*name)
        return false;
    struct Entry {
        const char *name;
        Band band;
    };
    static const Entry entries[] = {
        {"EU868", Band::EU868}, {"US915", Band::US915}, {"EU433", Band::EU433}, {"AU915", Band::AU915},
        {"CN470", Band::CN470}, {"AS923", Band::AS923}, {"KR920", Band::KR920}, {"IN865", Band::IN865},
    };
    for (const auto &e : entries) {
        if (strcasecmp(name, e.name) == 0) {
            out = e.band;
            return true;
        }
    }
    return false;
}
} // namespace jenwan

#endif // NEXUS_ENABLE_JENWAN
