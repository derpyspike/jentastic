#pragma once

#include <ctype.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** Channel-contention measurements of both protocols, as read from the radio at one instant. */
struct ContentionStats {
    uint32_t mtRxGood = 0, mtRxBad = 0, mtTxGood = 0, mtCadBusy = 0, mtTxAirtimeMs = 0, mtRxAirtimeMs = 0;
    uint32_t mcSeen = 0, mcDup = 0, mcRelayed = 0, mcDropped = 0, mcDeferChannel = 0, mcDeferRadio = 0;
    uint32_t mcTxAirtimeMs = 0, mcRxAirtimeMs = 0;
    float channelUtilPercent = 0, txUtilPercent = 0;
};

/** True for a message that is exactly the /stats command (leading and trailing blanks allowed). */
inline bool isStatsCommand(const char *msg)
{
    static const char cmd[] = "/stats";
    if (!msg)
        return false;
    while (*msg == ' ' || *msg == '\t')
        msg++;
    if (strncmp(msg, cmd, sizeof(cmd) - 1) != 0)
        return false;
    msg += sizeof(cmd) - 1;
    while (*msg != '\0') {
        if (!isspace(static_cast<unsigned char>(*msg)))
            return false;
        msg++;
    }
    return true;
}

/** One line for a text reply; typical values stay well under JenCore's 160-byte text limit. Returns its length. */
inline size_t formatContentionStats(char *buf, size_t len, const ContentionStats &s)
{
    if (!buf || len == 0)
        return 0;
    const int n =
        snprintf(buf, len,
                 "MT rx %lu bad %lu tx %lu cad %lu | MC seen %lu dup %lu relay %lu drop %lu busy %lu+%lu | "
                 "air s MT %lu/%lu MC %lu/%lu | ch %.0f%% tx %.1f%%",
                 (unsigned long)s.mtRxGood, (unsigned long)s.mtRxBad, (unsigned long)s.mtTxGood, (unsigned long)s.mtCadBusy,
                 (unsigned long)s.mcSeen, (unsigned long)s.mcDup, (unsigned long)s.mcRelayed, (unsigned long)s.mcDropped,
                 (unsigned long)s.mcDeferChannel, (unsigned long)s.mcDeferRadio, (unsigned long)(s.mtTxAirtimeMs / 1000),
                 (unsigned long)(s.mtRxAirtimeMs / 1000), (unsigned long)(s.mcTxAirtimeMs / 1000),
                 (unsigned long)(s.mcRxAirtimeMs / 1000), (double)s.channelUtilPercent, (double)s.txUtilPercent);
    if (n < 0)
        return 0;
    return (size_t)n < len ? (size_t)n : len - 1;
}
