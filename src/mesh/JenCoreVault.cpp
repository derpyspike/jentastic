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

bool validate(const Vault &v, size_t fileSize, const char **reason)
{
    auto reject = [reason](const char *why) {
        if (reason)
            *reason = why;
        return false;
    };
    if (fileSize != sizeof(Vault))
        return reject("size mismatch");
    if (memcmp(v.magic, MAGIC, sizeof(MAGIC)) != 0)
        return reject("bad magic");
    if (v.version != VERSION)
        return reject("unsupported version");
    if (crc32Buffer(&v, offsetof(Vault, crc32)) != v.crc32)
        return reject("CRC mismatch");
    if (reason)
        *reason = "OK";
    return true;
}
} // namespace

bool load(Vault &out, const char **reason)
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
    const size_t read = fileSize <= sizeof(out) ? file.read(reinterpret_cast<uint8_t *>(&out), fileSize) : 0;
    file.close();
    if (fileSize != sizeof(out) || read != fileSize) {
        if (reason)
            *reason = "invalid size or short read";
        return false;
    }
    return validate(out, fileSize, reason);
#else
    (void)reason;
    return false;
#endif
}

bool save(Vault &in)
{
    memcpy(in.magic, MAGIC, sizeof(MAGIC));
    in.version = VERSION;
    in.crc32 = crc32Buffer(&in, offsetof(Vault, crc32));
#ifdef FSCom
    SafeFile file(VAULT_PATH, true);
    const size_t written = file.write(reinterpret_cast<const uint8_t *>(&in), sizeof(in));
    return written == sizeof(in) && file.close();
#else
    return false;
#endif
}
} // namespace jenCoreVault

#endif
