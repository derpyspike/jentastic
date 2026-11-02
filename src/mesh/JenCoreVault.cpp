// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#include "JenCoreVault.h"

#if NEXUS_ENABLE_JENCORE

#include "FSCommon.h"
#include "SPILock.h"
#include "SafeFile.h"
#include <ErriezCRC32.h>
#include <cstring>

namespace jenCoreVault
{
namespace
{
constexpr const char *VAULT_PATH = "/jencore.bin";

/// v1 was an unused 20-byte header carrying only features_mask. Accept it so a
/// tree from before the policy existed still boots with clean defaults.
struct __attribute__((packed)) VaultV1 {
    char magic[4];
    uint32_t version;
    uint64_t features_mask;
    uint32_t crc32;
};

nexus::jen_core_policy::Policy currentPolicy;

bool validateHeader(const char *magic, uint32_t version, size_t fileSize, const char **reason)
{
    auto reject = [reason](const char *why) {
        if (reason)
            *reason = why;
        return false;
    };
    if (memcmp(magic, MAGIC, sizeof(MAGIC)) != 0)
        return reject("bad magic");
    if (version != VERSION && version != 1)
        return reject("unsupported version");
    if (version == VERSION && fileSize != sizeof(Vault))
        return reject("size mismatch");
    if (version == 1 && fileSize != sizeof(VaultV1))
        return reject("size mismatch");
    if (reason)
        *reason = "OK";
    return true;
}
} // namespace

nexus::jen_core_policy::Policy &policy()
{
    return currentPolicy;
}

bool load(nexus::jen_core_policy::Policy &out, const char **reason)
{
    if (reason)
        *reason = "no file";
    memset(&out, 0, sizeof(out));
#ifdef FSCom
    concurrency::LockGuard guard(spiLock);
    auto file = FSCom.open(VAULT_PATH, FILE_O_READ);
    if (!file)
        return false;

    const size_t fileSize = file.size();
    if (fileSize != sizeof(Vault) && fileSize != sizeof(VaultV1)) {
        file.close();
        if (reason)
            *reason = "invalid size";
        return false;
    }

    Vault v;
    memset(&v, 0, sizeof(v));
    const size_t read = file.read(reinterpret_cast<uint8_t *>(&v), fileSize);
    file.close();
    if (read != fileSize) {
        if (reason)
            *reason = "short read";
        return false;
    }

    if (!validateHeader(v.magic, v.version, fileSize, reason))
        return false;

    if (v.version == 1) {
        const auto *v1 = reinterpret_cast<const VaultV1 *>(&v);
        if (crc32Buffer(v1, offsetof(VaultV1, crc32)) != v1->crc32) {
            if (reason)
                *reason = "CRC mismatch";
            return false;
        }
        return true; // clean defaults; the v1 features_mask was never wired
    }

    if (crc32Buffer(&v, offsetof(Vault, crc32)) != v.crc32) {
        if (reason)
            *reason = "CRC mismatch";
        return false;
    }

    out = v.policy;
    nexus::jen_core_policy::sanitize(out);
    if (reason)
        *reason = "OK";
    return true;
#else
    (void)reason;
    return false;
#endif
}

bool save(const nexus::jen_core_policy::Policy &in)
{
#ifdef FSCom
    Vault v;
    memset(&v, 0, sizeof(v));
    memcpy(v.magic, MAGIC, sizeof(MAGIC));
    v.version = VERSION;
    v.policy = in;
    nexus::jen_core_policy::sanitize(v.policy);
    v.crc32 = crc32Buffer(&v, offsetof(Vault, crc32));

    SafeFile file(VAULT_PATH, true);
    const size_t written = file.write(reinterpret_cast<const uint8_t *>(&v), sizeof(v));
    return written == sizeof(v) && file.close();
#else
    (void)in;
    return false;
#endif
}

bool loadPolicy(const char **reason)
{
    const bool ok = load(currentPolicy, reason);
    if (!ok)
        memset(&currentPolicy, 0, sizeof(currentPolicy));
    return ok;
}

bool savePolicy()
{
    return save(currentPolicy);
}
} // namespace jenCoreVault

#endif
