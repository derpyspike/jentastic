// SPDX-License-Identifier: AGPL-3.0-only
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <strings.h>

namespace meshtastic::nexus {

inline uint32_t crc32(const void *data, size_t size)
{
    uint32_t crc = UINT32_MAX;
    const auto *bytes = static_cast<const uint8_t *>(data);
    for (size_t i = 0; i < size; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

template <typename Vault> bool decodeVault(Vault &out, size_t fileSize, size_t bytesRead, const char **reason)
{
    auto reject = [reason](const char *why) {
        if (reason)
            *reason = why;
        return false;
    };
    if (bytesRead != fileSize || fileSize < 12 || fileSize > sizeof(Vault))
        return reject("invalid size or short read");
    if (memcmp(out.magic, "JenT", 4) != 0)
        return reject("bad magic");
    size_t payloadSize;
    switch (out.version) {
    case 65:
        payloadSize = offsetof(Vault, features_mask);
        break;
    case 66:
        payloadSize = offsetof(Vault, nexusVariantBits);
        break;
    case 67:
        payloadSize = offsetof(Vault, volt_rules);
        break;
    case 68:
        payloadSize = offsetof(Vault, split_config);
        break;
    case 69:
        payloadSize = offsetof(Vault, multi_sf);
        break;
    case 70:
        payloadSize = offsetof(Vault, jen_config);
        break;
    case 71:
        payloadSize = offsetof(Vault, jen_regions);
        break;
    case 72:
        payloadSize = offsetof(Vault, tier_channels);
        break;
    case 73:
        payloadSize = offsetof(Vault, jen_advert);
        break;
    case 74:
        payloadSize = offsetof(Vault, jen_flood);
        break;
    case 75:
        payloadSize = offsetof(Vault, crc32);
        break;
    default:
        return reject("unsupported version");
    }
    if (fileSize != payloadSize + sizeof(uint32_t))
        return reject("size does not match version");
    auto *bytes = reinterpret_cast<uint8_t *>(&out);
    uint32_t storedCrc;
    memcpy(&storedCrc, bytes + payloadSize, sizeof(storedCrc));
    if (crc32(bytes, payloadSize) != storedCrc)
        return reject("CRC mismatch");
    // Strip the historic trailing CRC before introducing the appended fields.
    memset(bytes + payloadSize, 0, offsetof(Vault, crc32) - payloadSize);
    out.crc32 = storedCrc;
    if (reason)
        *reason = "OK";
    return true;
}

template <typename FS, typename ReadMode, typename WriteMode>
bool writeVault(FS &fs, const void *image, size_t size, ReadMode readMode, WriteMode writeMode)
{
    constexpr const char *TEMP = "/nexus.bin.tmp";
    if (fs.exists(TEMP) && !fs.remove(TEMP))
        return false;
    auto file = fs.open(TEMP, writeMode);
    if (!file)
        return false;
    const auto *bytes = static_cast<const uint8_t *>(image);
    size_t written = file.write(bytes, size);
    file.close();
    if (written != size)
        return false;
    auto check = fs.open(TEMP, readMode);
    if (!check)
        return false;
    bool valid = check.size() == size;
    for (size_t i = 0; valid && i < size; ++i)
        valid = check.read() == bytes[i];
    check.close();
    // The existing cross-platform SafeFile path uses the same atomic rename.
    return valid && fs.rename(TEMP, "/nexus.bin");
}

template <size_t Capacity> class ReplayRing
{
  public:
    bool check(uint32_t id)
    {
        for (size_t i = 0; i < count; ++i)
            if (ids[i] == id)
                return true;
        ids[next] = id;
        next = (next + 1) % Capacity;
        if (count < Capacity)
            ++count;
        return false;
    }
    size_t size() const { return count; }

  private:
    uint32_t ids[Capacity] = {};
    size_t next = 0;
    size_t count = 0;
};

class PagingWindow
{
  public:
    void reset(size_t skipBytes = 0)
    {
        offset = skipBytes;
        produced = used = 0;
    }
    void append(char *buffer, size_t capacity, const char *text)
    {
        size_t length = strlen(text);
        size_t skip = offset > produced ? std::min(length, offset - produced) : 0;
        size_t take = std::min(length - skip, capacity - used);
        memcpy(buffer + used, text + skip, take);
        used += take;
        produced += length;
    }
    size_t size() const { return used; }
    size_t end() const { return offset + used; }
    bool hasMore() const { return produced > end(); }
    void finish(const char *buffer)
    {
        if (!hasMore() || !used)
            return;
        size_t start = used - 1;
        while (start && (static_cast<uint8_t>(buffer[start]) & 0xC0) == 0x80)
            --start;
        uint8_t lead = static_cast<uint8_t>(buffer[start]);
        size_t length = lead < 0x80 ? 1 : lead < 0xE0 ? 2 : lead < 0xF0 ? 3 : 4;
        if (start + length > used)
            used = start;
    }

  private:
    size_t offset = 0;
    size_t produced = 0;
    size_t used = 0;
};

inline bool readUnsigned(const char *&cursor, uint32_t &value)
{
    if (*cursor < '0' || *cursor > '9')
        return false;
    value = 0;
    do {
        uint32_t digit = static_cast<uint32_t>(*cursor++ - '0');
        if (value > (UINT32_MAX - digit) / 10)
            return false;
        value = value * 10 + digit;
    } while (*cursor >= '0' && *cursor <= '9');
    return true;
}

inline bool parseCron(const char *args, uint32_t now, uint8_t &mode, uint32_t &value, const char *&command)
{
    const char *cursor;
    if (strncasecmp(args, "PER ", 4) == 0) {
        mode = 0;
        cursor = args + 4;
        if (!readUnsigned(cursor, value) || value < 60 || value > INT32_MAX / 1000)
            return false;
    } else if (strncasecmp(args, "SCH ", 4) == 0) {
        mode = 1;
        cursor = args + 4;
        uint32_t hour, minute;
        if (!readUnsigned(cursor, hour) || *cursor++ != ':' || !readUnsigned(cursor, minute) || hour > 23 || minute > 59)
            return false;
        value = hour * 3600 + minute * 60;
    } else if (strncasecmp(args, "AT ", 3) == 0) {
        mode = 2;
        cursor = args + 3;
        if (!readUnsigned(cursor, value) || now <= 1700000000 || value <= now)
            return false;
    } else {
        return false;
    }
    if (*cursor != ' ')
        return false;
    while (*cursor == ' ')
        ++cursor;
    command = cursor;
    return *command && strlen(command) < 64;
}

inline bool dailyDue(uint32_t now, uint32_t secondsOfDay, uint32_t lastRun)
{
    return now > 1700000000 && secondsOfDay < 86400 && now % 86400 >= secondsOfDay &&
           (lastRun == 0 || now / 86400 > lastRun / 86400);
}

inline bool parseSideDetectors(const char *args, uint8_t *sfs, uint8_t *syncwords, size_t capacity, uint8_t &count)
{
    count = 0;
    if (strcasecmp(args, "OFF") == 0 || strcmp(args, "0") == 0)
        return true;
    uint32_t low = 12, high = 5;
    while (*args) {
        uint32_t value;
        if (count == capacity || !readUnsigned(args, value) || value < 5 || value > 12)
            return false;
        for (uint8_t i = 0; i < count; ++i)
            if (sfs[i] == value)
                return false;
        uint8_t sw = 0;
        if (*args == ':') {
            ++args;
            if (strncasecmp(args, "jc", 2) == 0 || strncasecmp(args, "mc", 2) == 0) {
                sw = 0x12;
                args += 2;
            } else if (strncasecmp(args, "0x", 2) == 0) {
                char *endp = nullptr;
                unsigned long parsedSw = strtoul(args + 2, &endp, 16);
                if (endp == args + 2 || parsedSw > 0xFF)
                    return false;
                sw = static_cast<uint8_t>(parsedSw);
                args = endp;
            } else {
                uint32_t parsedSw = 0;
                if (!readUnsigned(args, parsedSw) || parsedSw > 0xFF)
                    return false;
                sw = static_cast<uint8_t>(parsedSw);
            }
        }
        sfs[count] = static_cast<uint8_t>(value);
        if (syncwords)
            syncwords[count] = sw;
        count++;
        low = std::min(low, value);
        high = std::max(high, value);
        if (*args && *args != ' ')
            return false;
        while (*args == ' ')
            ++args;
    }
    return count != 0 && high - low <= 4;
}

inline bool parseSideDetectors(const char *args, uint8_t *sfs, size_t capacity, uint8_t &count)
{
    return parseSideDetectors(args, sfs, nullptr, capacity, count);
}

} // namespace meshtastic::nexus
