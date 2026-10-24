#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/** A JenCore peer we can exchange direct messages with, learned from its signed ADVERT. */
struct JenCoreContact {
    static constexpr size_t NAME_LEN = 40; // meshtastic_User.long_name

    uint8_t pub[32] = {};
    char name[NAME_LEN] = {};
    uint32_t nodeNum = 0;
    uint8_t secret[32] = {}; // ECDH shared secret, valid once hasSecret
    bool hasSecret = false;
    uint32_t lastUsed = 0; // LRU stamp; 0 marks a free slot
};

/**
 * Fixed-size contact table, least recently used entry evicted first. Lives in RAM; the companion saves the
 * favourite ones to flash.
 */
template <size_t N> class JenCoreContacts
{
  public:
    /// Node number standing in for a JenCore name: 'M' in the top byte, 24 bits hashed from the name. A contact and
    /// the sender of a channel text with the same name share it, so the client shows one node, not two.
    static uint32_t nodeNumFor(const char *name)
    {
        uint32_t h = 2166136261u; // FNV-1a
        for (const char *p = name; p && *p; p++)
            h = (h ^ (uint8_t)*p) * 16777619u;
        return 0x4D000000 | ((h ^ (h >> 24)) & 0x00FFFFFF);
    }

    /// Node number for a repeater, which has no contact entry: hashed from its public key.
    static uint32_t nodeNumForKey(const uint8_t pub[32])
    {
        uint32_t h = 2166136261u; // FNV-1a
        for (size_t i = 0; i < 32; i++)
            h = (h ^ pub[i]) * 16777619u;
        return 0x4D000000 | ((h ^ (h >> 24)) & 0x00FFFFFF);
    }

    /// Add the contact, or refresh its name. A full table gives up its least recently used entry.
    JenCoreContact *upsert(const uint8_t pub[32], const char *name)
    {
        JenCoreContact *c = findByPub(pub);
        if (!c) {
            c = &entries[victim()];
            *c = JenCoreContact();
            memcpy(c->pub, pub, 32);
            c->nodeNum = freeNodeNum(name);
        }
        if (name) {
            strncpy(c->name, name, sizeof(c->name) - 1);
            c->name[sizeof(c->name) - 1] = 0;
        }
        touch(c);
        return c;
    }

    JenCoreContact *findByPub(const uint8_t pub[32])
    {
        for (auto &e : entries)
            if (e.lastUsed && memcmp(e.pub, pub, 32) == 0)
                return &e;
        return nullptr;
    }

    /// The contact with exactly this name, or null when none or several have it (a name alone cannot pick one).
    JenCoreContact *findUniqueByName(const char *name)
    {
        JenCoreContact *found = nullptr;
        for (auto &e : entries) {
            if (e.lastUsed && strcmp(e.name, name) == 0) {
                if (found)
                    return nullptr;
                found = &e;
            }
        }
        return found;
    }

    JenCoreContact *findByNodeNum(uint32_t nodeNum)
    {
        for (auto &e : entries)
            if (e.lastUsed && e.nodeNum == nodeNum)
                return &e;
        return nullptr;
    }

    /// Contacts whose public key starts with hash (the 1-byte hash collides, so there can be several).
    size_t findByHash(uint8_t hash, JenCoreContact **out, size_t max)
    {
        size_t n = 0;
        for (auto &e : entries)
            if (e.lastUsed && e.pub[0] == hash && n < max)
                out[n++] = &e;
        return n;
    }

    size_t count() const
    {
        size_t n = 0;
        for (const auto &e : entries)
            if (e.lastUsed)
                n++;
        return n;
    }

    void touch(JenCoreContact *c) { c->lastUsed = ++clock; }

    template <typename F> void forEach(F fn)
    {
        for (auto &e : entries)
            if (e.lastUsed)
                fn(e);
    }

  private:
    JenCoreContact entries[N];
    uint32_t clock = 0;

    size_t victim() const
    {
        size_t best = 0;
        for (size_t i = 0; i < N; i++) {
            if (!entries[i].lastUsed)
                return i;
            if (entries[i].lastUsed < entries[best].lastUsed)
                best = i;
        }
        return best;
    }

    /// The hashed node number, moved on if another contact already owns it.
    uint32_t freeNodeNum(const char *name)
    {
        uint32_t num = nodeNumFor(name);
        while (findByNodeNum(num) || num == 0x4D430001) // 0x4D430001: the generic "JenCore" node
            num = 0x4D000000 | ((num + 1) & 0x00FFFFFF);
        return num;
    }
};
