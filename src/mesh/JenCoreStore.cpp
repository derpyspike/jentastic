// SPDX-License-Identifier: AGPL-3.0-only
// Copyright (C) 2026 Jennifer Sanchez
//
// Licensed under the GNU Affero General Public License v3.
// See LICENSE-AGPL for the full license text.
//

#include "JenCoreStore.h"

#include "FSCommon.h"
#include "HardwareRNG.h"
#include "SPILock.h"
#include "SafeFile.h"
#include "Throttle.h"
#include "UptimeClock.h"

#if NEXUS_ENABLE_JENCORE_COMPANION
#include "JenCoreCompanion.h"
#endif

namespace
{
#if NEXUS_ENABLE_JENCORE_COMPANION
constexpr uint32_t CONTACTS_SAVE_MIN_MS = 10 * 60 * 1000; // flash wear: at most one save per 10 minutes
uint32_t contactsFingerprint = 0;                         // of what is on flash, so a save happens only on a change
uint32_t lastContactsSaveMs = 0;
bool contactsLoaded = false;
#endif
} // namespace

namespace jenCoreStore
{
#if NEXUS_ENABLE_JENCORE_SNIFFER
bool loadOrCreateSeed(uint8_t seed[SEED_LEN], bool forceNew)
{
#ifdef FSCom
    static const char *path = "/prefs/jencore_identity.bin";
    static const char *legacyPath = "/prefs/meshcore_identity.bin";
    if (!forceNew) {
        concurrency::LockGuard g(spiLock);
        const char *readPath = FSCom.exists(path) ? path : (FSCom.exists(legacyPath) ? legacyPath : nullptr);
        if (readPath) {
            auto f = FSCom.open(readPath, FILE_O_READ);
            if (f) {
                bool ok = f.size() == SEED_LEN && f.read(seed, SEED_LEN) == SEED_LEN;
                f.close();
                if (ok)
                    return true;
                LOG_WARN("JenCore identity: %s is corrupt, creating a new key", readPath);
            }
        }
    }
#endif
    if (!HardwareRNG::fill(seed, SEED_LEN, true)) {
        LOG_ERROR("JenCore identity: no hardware entropy, not creating a key");
        return false;
    }
#ifdef FSCom
    // SafeFile: atomic replace under spiLock. A bare FILE_O_WRITE appends on nRF52 LittleFS.
    SafeFile out(path, true);
    out.write(seed, SEED_LEN);
    if (!out.close())
        LOG_ERROR("JenCore identity: could not save %s, the key will change on reboot", path);
#endif
    LOG_INFO("JenCore identity created");
    return true;
}
#endif

#if NEXUS_ENABLE_JENCORE_COMPANION
void loadContactsOnce(JenCoreCompanion &companion)
{
    if (contactsLoaded)
        return;
    contactsLoaded = true;

#ifdef FSCom
    static const char *path = "/prefs/jencore_contacts.bin";
    static const char *legacyPath = "/prefs/meshcore_contacts.bin";
    uint8_t image[JenCoreCompanion::SAVED_IMAGE_MAX];
    size_t n = 0;
    {
        concurrency::LockGuard g(spiLock);
        const char *readPath = FSCom.exists(path) ? path : (FSCom.exists(legacyPath) ? legacyPath : nullptr);
        if (readPath) {
            auto f = FSCom.open(readPath, FILE_O_READ);
            if (f) {
                n = f.size() <= sizeof(image) ? f.read(image, f.size()) : 0;
                f.close();
            }
        }
    }
    if (n) {
        const size_t loaded = companion.importContacts(image, n);
        LOG_INFO("JenCore contacts: %u restored from flash", (unsigned)loaded);
    }
#endif

    contactsFingerprint = companion.contactsFingerprint();
    lastContactsSaveMs = Time::getMillis();
}

void saveContactsIfChanged(JenCoreCompanion &companion)
{
    if (!contactsLoaded || !Throttle::hasElapsed(lastContactsSaveMs, CONTACTS_SAVE_MIN_MS))
        return;

    const uint32_t fingerprint = companion.contactsFingerprint();
    if (fingerprint == contactsFingerprint)
        return;

#ifdef FSCom
    static const char *path = "/prefs/jencore_contacts.bin";
    uint8_t image[JenCoreCompanion::SAVED_IMAGE_MAX];
    const size_t n = companion.exportContacts(image, sizeof(image));
    if (n) {
        SafeFile out(path, true); // atomic replace
        out.write(image, n);
        if (!out.close())
            LOG_WARN("JenCore contacts: could not save %s", path);
    }
#endif

    contactsFingerprint = fingerprint;
    lastContactsSaveMs = Time::getMillis();
}
#endif
} // namespace jenCoreStore
